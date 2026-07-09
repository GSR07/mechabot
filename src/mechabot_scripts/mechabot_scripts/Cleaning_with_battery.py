#!/usr/bin/env python3

import math
import time
import threading

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu, LaserScan, Image
from std_msgs.msg import Int32, Float32, String

from cv_bridge import CvBridge
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import json
import os
from pathlib import Path

from visualization_msgs.msg import Marker, MarkerArray


class VacuumCleanerWithBattery(Node):
    def __init__(self):
        super().__init__("spawning_with_battery")

        # ============================================================
        # Nav2
        # ============================================================
        self.nav = BasicNavigator()

        # ============================================================
        # Publishers
        # ============================================================
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.battery_int_pub = self.create_publisher(
            Int32,
            "/battery_status",
            10
        )

        self.battery_float_pub = self.create_publisher(
            Float32,
            "/vacuum/battery_percent",
            10
        )

        self.status_pub = self.create_publisher(
            String,
            "/vacuum/status",
            10
        )

        # ============================================================
        # Subscribers
        # ============================================================
        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            map_qos,
        )

        self.imu_sub = self.create_subscription(
            Imu,
            "/imu/out",
            self.imu_callback,
            10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10,
        )

        self.camera_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.camera_callback,
            10,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/wheel_controller/odom",
            self.odom_callback,
            10,
        )
        
        marker_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.spawn_marker_pub = self.create_publisher(
            MarkerArray,
            "/mechabot/spawn_marker",
            marker_qos,
        )

        # ============================================================
        # Camera / QR
        # ============================================================
        self.bridge = CvBridge()
        self.qr_decoder = cv2.QRCodeDetector()

        self.frame_lock = threading.Lock()
        self.qr_lock = threading.Lock()

        self.display_frame = None
        self.qr_detected = False
        self.qr_center_x = None
        self.last_qr_center_x = None
        self.last_qr_seen_time = 0.0
        self.qr_memory_time = 0.7
        self.last_qr_error = 0.0
        self.image_width = 640

        # ============================================================
        # Robot state
        # ============================================================
        self.yaw = 0.0
        self.front_distance = 100.0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_lock = threading.Lock()

        # ============================================================
        # Battery simulation
        # ============================================================
        self.battery = 100.0
        self.battery_lock = threading.Lock()

        self.battery_drain_per_second = 0.25
        self.charge_rate_per_second = 2.0
        self.low_battery_threshold = 45.0

        self.state = "IDLE"
        self.state_lock = threading.Lock()

        self.battery_timer = self.create_timer(
            1.0,
            self.battery_timer_callback
        )
        
        # Save final battery reading here
        self.final_battery_file = str(
            Path(__file__).resolve().parent / "final_battery_reading.json"
        )

        # Allows us to stop battery publishing after mission is done
        self.battery_publishing_enabled = True

        # ============================================================
        # Dock pose
        # Same as your existing docking setup
        # ============================================================
        self.dock_x = 1.5
        self.dock_y = 5.18
        self.dock_yaw = 1.57

        # Nav2 returns here first, not directly inside the dock.
        self.charge_standoff_distance = 0.55

        self.charge_x = self.dock_x - self.charge_standoff_distance * math.cos(self.dock_yaw)
        self.charge_y = self.dock_y - self.charge_standoff_distance * math.sin(self.dock_yaw)
        self.charge_yaw = self.dock_yaw

        # ============================================================
        # Final QR/lidar docking parameters
        # ============================================================
        self.docking_threshold = 0.16
        self.final_docking_speed = 0.025
        self.max_final_docking_distance = 0.70

        # ============================================================
        # Cleaning parameters
        # ============================================================
        self.map_msg = None
        self.map_lock = threading.Lock()

        self.coverage_spacing = 0.65
        self.clearance_radius = 0.40
        self.exclude_dock_radius = 1.20

        self.free_occupancy_threshold = 25
        self.unknown_is_obstacle = True

        self.cleaning_waypoints = []
        self.current_waypoint_index = 0

        self.get_logger().info("Vacuum Cleaner With Battery node initialized")

    # ============================================================
    # State
    # ============================================================

    def set_state(self, state):
        with self.state_lock:
            self.state = state

        msg = String()
        msg.data = state
        self.status_pub.publish(msg)

        self.get_logger().info(f"STATE: {state}")

    def get_state(self):
        with self.state_lock:
            return self.state

    # ============================================================
    # Callbacks
    # ============================================================

    def map_callback(self, msg):
        with self.map_lock:
            self.map_msg = msg

    def imu_callback(self, msg):
        q = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]

        _, _, self.yaw = euler_from_quaternion(q)

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)

        front = np.concatenate([ranges[-15:], ranges[:15]])
        front = front[np.isfinite(front)]
        front = front[(front > msg.range_min) & (front < msg.range_max)]

        if len(front) > 0:
            self.front_distance = float(np.min(front))

    def odom_callback(self, msg):
        with self.odom_lock:
            self.odom_x = msg.pose.pose.position.x
            self.odom_y = msg.pose.pose.position.y

    def camera_callback(self, msg):
        """
        Camera callback for QR docking.

        Important:
        QR detection is only needed during DOCKING.
        During CLEANING, we skip QR detection to avoid unnecessary OpenCV load/errors.
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Camera conversion failed: {e}")
            return

        if frame is None or frame.size == 0:
            return

        self.image_width = frame.shape[1]
        image_center_x = self.image_width // 2

        current_state = self.get_state()

        # Do not detect QR while cleaning.
        # QR is only needed when robot is docking/charging.
        if current_state not in ["DOCKING", "RETURNING_TO_DOCK", "CHARGING"]:
            with self.frame_lock:
                self.display_frame = frame.copy()
            return

        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Make image safe for OpenCV QR detector
            gray = np.ascontiguousarray(gray, dtype=np.uint8)

            detected, points = self.qr_decoder.detect(gray)

        except cv2.error as e:
            self.get_logger().warn(f"QR detector OpenCV error ignored: {e}")

            with self.qr_lock:
                self.qr_detected = False
                self.qr_center_x = None

            with self.frame_lock:
                self.display_frame = frame.copy()

            return

        except Exception as e:
            self.get_logger().warn(f"QR detector error ignored: {e}")

            with self.qr_lock:
                self.qr_detected = False
                self.qr_center_x = None

            with self.frame_lock:
                self.display_frame = frame.copy()

            return

        now = time.time()

        if detected and points is not None:
            try:
                points = points[0].astype(np.int32)

                center_x = int(points[:, 0].mean())
                center_y = int(points[:, 1].mean())

                with self.qr_lock:
                    self.qr_detected = True
                    self.qr_center_x = center_x
                    self.last_qr_center_x = center_x
                    self.last_qr_seen_time = now

                error = center_x - image_center_x

                cv2.polylines(frame, [points], True, (0, 255, 0), 2)
                cv2.circle(frame, (center_x, center_y), 7, (255, 0, 0), 2)
                cv2.circle(frame, (image_center_x, center_y), 5, (255, 255, 255), -1)

                cv2.putText(
                    frame,
                    f"QR detected | error: {error}",
                    (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

            except Exception as e:
                self.get_logger().warn(f"QR point processing error ignored: {e}")

                with self.qr_lock:
                    self.qr_detected = False
                    self.qr_center_x = None

        else:
            with self.qr_lock:
                if now - self.last_qr_seen_time < self.qr_memory_time:
                    self.qr_detected = True
                    self.qr_center_x = self.last_qr_center_x
                else:
                    self.qr_detected = False
                    self.qr_center_x = None

            cv2.putText(
                frame,
                "QR not detected",
                (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

        with self.frame_lock:
            self.display_frame = frame.copy()

    # ============================================================
    # Battery
    # ============================================================

    def battery_timer_callback(self):
        if not self.battery_publishing_enabled:
            return
        state = self.get_state()

        with self.battery_lock:
            if state in ["UNDOCKING", "CLEANING", "RETURNING_TO_DOCK", "DOCKING"]:
                self.battery -= self.battery_drain_per_second
                self.battery = max(0.0, self.battery)

            elif state == "CHARGING":
                self.battery += self.charge_rate_per_second
                self.battery = min(100.0, self.battery)

            battery_value = self.battery

        int_msg = Int32()
        int_msg.data = int(round(battery_value))
        self.battery_int_pub.publish(int_msg)

        float_msg = Float32()
        float_msg.data = float(battery_value)
        self.battery_float_pub.publish(float_msg)

        self.get_logger().info(f"Battery: {battery_value:.2f}%")

    def get_battery(self):
        with self.battery_lock:
            return self.battery

    def is_battery_low(self):
        return self.get_battery() <= self.low_battery_threshold
    
    def stop_battery_publisher_and_save(self):
        """
        Stop battery publishing after mission is finished
        and save the last battery reading.
        """
        with self.battery_lock:
            final_battery = float(self.battery)

        data = {
            "final_battery_percent": final_battery,
            "state": self.get_state(),
            "timestamp": time.time()
        }

        try:
            with open(self.final_battery_file, "w") as f:
                json.dump(data, f, indent=4)

            self.get_logger().info(
                f"Final battery saved: {final_battery:.2f}% -> {self.final_battery_file}"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to save final battery reading: {e}")

        self.battery_publishing_enabled = False

        try:
            self.battery_timer.cancel()
        except Exception:
            pass

        self.get_logger().info("Battery publisher stopped")
        
        
    # Marker
    def publish_spawn_marker(self):
        """
        Publish RViz marker showing robot spawn/dock area.
        """
        marker_array = MarkerArray()

        # Cylinder marker around spawn/dock position
        spawn_area = Marker()
        spawn_area.header.frame_id = "map"
        spawn_area.header.stamp = self.get_clock().now().to_msg()
        spawn_area.ns = "mechabot_spawn"
        spawn_area.id = 0
        spawn_area.type = Marker.CYLINDER
        spawn_area.action = Marker.ADD

        spawn_area.pose.position.x = self.dock_x
        spawn_area.pose.position.y = self.dock_y
        spawn_area.pose.position.z = 0.02

        spawn_area.pose.orientation.w = 1.0

        spawn_area.scale.x = 0.70
        spawn_area.scale.y = 0.70
        spawn_area.scale.z = 0.03

        spawn_area.color.r = 0.0
        spawn_area.color.g = 1.0
        spawn_area.color.b = 0.0
        spawn_area.color.a = 0.35

        spawn_area.lifetime.sec = 0

        marker_array.markers.append(spawn_area)

        # Text marker
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "mechabot_spawn"
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD

        text_marker.pose.position.x = self.dock_x
        text_marker.pose.position.y = self.dock_y
        text_marker.pose.position.z = 0.45

        text_marker.pose.orientation.w = 1.0

        text_marker.scale.z = 0.25

        text_marker.color.r = 0.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0

        text_marker.text = "Mechabot Spawn / Charging Dock"
        text_marker.lifetime.sec = 0

        marker_array.markers.append(text_marker)

        self.spawn_marker_pub.publish(marker_array)

        self.get_logger().info("Published spawn marker")

    # ============================================================
    # Utility
    # ============================================================

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def get_odom_position(self):
        with self.odom_lock:
            return self.odom_x, self.odom_y

    def get_qr_state(self):
        with self.qr_lock:
            now = time.time()

            if now - self.last_qr_seen_time < self.qr_memory_time:
                return True, self.last_qr_center_x

            return False, None

    def update_display(self):
        with self.frame_lock:
            if self.display_frame is not None:
                cv2.imshow("Vacuum QR Docking Camera", self.display_frame)
                cv2.waitKey(1)

    def velocity_publisher(self, x, z):
        if not rclpy.ok():
            return

        vel = Twist()
        vel.linear.x = float(x)
        vel.angular.z = float(z)

        try:
            self.vel_pub.publish(vel)
        except Exception:
            pass

    def stop_robot(self):
        self.velocity_publisher(0.0, 0.0)

    # ============================================================
    # Basic motion like auto_docking_with_battery.py
    # ============================================================

    def move_distance(self, distance, speed=0.15):
        self.get_logger().info(f"Moving {distance:.2f} m at {speed:.2f} m/s")

        duration = abs(distance / speed)
        start_time = time.time()

        direction_speed = speed if distance > 0 else -speed

        while rclpy.ok() and (time.time() - start_time) < duration:
            self.velocity_publisher(direction_speed, 0.0)
            self.update_display()
            time.sleep(0.02)

        self.stop_robot()
        time.sleep(0.5)

    def rotate_to_angle(self, target_angle, tolerance=0.035):
        self.get_logger().info(
            f"Rotating to {math.degrees(target_angle):.1f} degrees"
        )

        while rclpy.ok():
            self.update_display()

            angle_diff = self.normalize_angle(target_angle - self.yaw)

            if abs(angle_diff) < tolerance:
                break

            angular_speed = np.clip(angle_diff * 0.7, -0.35, 0.35)
            self.velocity_publisher(0.0, angular_speed)

            time.sleep(0.03)

        self.stop_robot()
        time.sleep(0.5)
        self.get_logger().info("Rotation complete")

    def undock_sequence(self, label="Undocking"):
        self.set_state("UNDOCKING")
        self.get_logger().info(f"===== {label.upper()} =====")

        self.move_distance(-0.60, speed=0.15)

        target_yaw = self.normalize_angle(self.yaw + math.pi)
        self.rotate_to_angle(target_yaw)

        self.clear_costmaps()
        time.sleep(1.0)

    # ============================================================
    # Nav2 helpers
    # ============================================================

    def create_pose_stamped(self, x, y, yaw):
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.nav.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    def set_initial_pose(self):
        self.get_logger().info("Setting robot initial pose at dock")

        initial_pose = self.create_pose_stamped(
            self.dock_x,
            self.dock_y,
            self.dock_yaw,
        )

        self.nav.setInitialPose(initial_pose)

        self.get_logger().info("Waiting for Nav2 to become active...")
        self.nav.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active")
        self.publish_spawn_marker()

    def clear_costmaps(self):
        try:
            self.nav.clearAllCostmaps()
            self.get_logger().info("Cleared Nav2 costmaps")
        except Exception as e:
            self.get_logger().warn(f"Could not clear costmaps: {e}")

    def navigate_to_pose(
        self,
        x,
        y,
        yaw,
        label="goal",
        monitor_battery=True,
        emergency_stop=True,
        timeout=180.0,
    ):
        pose = self.create_pose_stamped(x, y, yaw)

        self.get_logger().info(
            f"Navigating to {label}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )

        self.nav.goToPose(pose)
        start_time = time.time()

        while rclpy.ok() and not self.nav.isTaskComplete():
            self.update_display()

            if monitor_battery and self.is_battery_low():
                self.get_logger().warn(
                    f"Low battery reached: {self.get_battery():.2f}%"
                )
                self.nav.cancelTask()
                self.stop_robot()
                time.sleep(1.0)
                return "LOW_BATTERY"

            if emergency_stop and self.front_distance < 0.12:
                self.get_logger().error(
                    f"Emergency stop: obstacle too close: {self.front_distance:.2f} m"
                )
                self.nav.cancelTask()
                self.stop_robot()
                self.clear_costmaps()
                time.sleep(1.0)
                return "COLLISION_RISK"

            if time.time() - start_time > timeout:
                self.get_logger().warn(
                    f"Navigation timeout while going to {label}"
                )
                self.nav.cancelTask()
                self.stop_robot()
                time.sleep(1.0)
                return "TIMEOUT"

            time.sleep(0.1)

        result = self.nav.getResult()
        self.get_logger().info(f"Navigation result for {label}: {result}")

        if result == TaskResult.SUCCEEDED:
            return "SUCCEEDED"

        return "FAILED"

    # ============================================================
    # Map / coverage generation
    # ============================================================

    def wait_for_map(self, timeout=20.0):
        self.get_logger().info("Waiting for /map...")

        start_time = time.time()

        while rclpy.ok() and (time.time() - start_time) < timeout:
            with self.map_lock:
                if self.map_msg is not None:
                    self.get_logger().info("Map received")
                    return True

            self.update_display()
            time.sleep(0.2)

        self.get_logger().error("Map not received")
        return False

    def grid_to_world(self, mx, my, map_msg):
        resolution = map_msg.info.resolution

        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y

        q = map_msg.info.origin.orientation
        _, _, origin_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        local_x = (mx + 0.5) * resolution
        local_y = (my + 0.5) * resolution

        world_x = (
            origin_x
            + local_x * math.cos(origin_yaw)
            - local_y * math.sin(origin_yaw)
        )

        world_y = (
            origin_y
            + local_x * math.sin(origin_yaw)
            + local_y * math.cos(origin_yaw)
        )

        return world_x, world_y, origin_yaw

    def distance_to_dock(self, x, y):
        return math.sqrt(
            (x - self.dock_x) ** 2 +
            (y - self.dock_y) ** 2
        )

    def is_forbidden_cleaning_zone(self, x, y):
        # Avoid dock/spawn/charging area
        if self.distance_to_dock(x, y) < self.exclude_dock_radius:
            return True

        # Avoid bottom-left clutter zone seen in RViz.
        # Tune/remove this later if needed.
        if -10.0 <= x <= -6.0 and -7.0 <= y <= -3.5:
            return True

        return False

    def generate_cleaning_waypoints_from_map(self):
        with self.map_lock:
            map_msg = self.map_msg

        if map_msg is None:
            self.get_logger().error("No map available")
            return []

        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution

        data = np.array(map_msg.data, dtype=np.int16).reshape((height, width))

        if self.unknown_is_obstacle:
            free_mask = (data >= 0) & (data <= self.free_occupancy_threshold)
        else:
            free_mask = data <= self.free_occupancy_threshold

        free_image = free_mask.astype(np.uint8) * 255

        distance_cells = cv2.distanceTransform(
            free_image,
            cv2.DIST_L2,
            5,
        )

        distance_meters = distance_cells * resolution

        safe_mask = free_mask & (distance_meters >= self.clearance_radius)

        row_step = max(1, int(self.coverage_spacing / resolution))
        min_segment_cells = max(3, int(self.coverage_spacing / resolution))

        waypoints = []
        left_to_right = True

        for my in range(0, height, row_step):
            row_safe = safe_mask[my, :]
            safe_indices = np.where(row_safe)[0]

            if len(safe_indices) == 0:
                continue

            segments = []
            start = safe_indices[0]
            previous = safe_indices[0]

            for idx in safe_indices[1:]:
                if idx == previous + 1:
                    previous = idx
                else:
                    if previous - start + 1 >= min_segment_cells:
                        segments.append((start, previous))
                    start = idx
                    previous = idx

            if previous - start + 1 >= min_segment_cells:
                segments.append((start, previous))

            if not segments:
                continue

            if not left_to_right:
                segments = list(reversed(segments))

            for x_start, x_end in segments:
                if left_to_right:
                    first_x = x_start
                    second_x = x_end
                    yaw = 0.0
                else:
                    first_x = x_end
                    second_x = x_start
                    yaw = math.pi

                wx1, wy1, origin_yaw = self.grid_to_world(first_x, my, map_msg)
                wx2, wy2, _ = self.grid_to_world(second_x, my, map_msg)

                final_yaw = self.normalize_angle(yaw + origin_yaw)

                if not self.is_forbidden_cleaning_zone(wx1, wy1):
                    waypoints.append((wx1, wy1, final_yaw))

                if not self.is_forbidden_cleaning_zone(wx2, wy2):
                    waypoints.append((wx2, wy2, final_yaw))

            left_to_right = not left_to_right

        filtered = []

        for wp in waypoints:
            if not filtered:
                filtered.append(wp)
                continue

            last = filtered[-1]
            d = math.sqrt(
                (wp[0] - last[0]) ** 2 +
                (wp[1] - last[1]) ** 2
            )

            if d >= self.coverage_spacing * 0.5:
                filtered.append(wp)

        self.get_logger().info(
            f"Generated {len(filtered)} cleaning waypoints"
        )

        return filtered

    # ============================================================
    # QR/lidar final docking for charging
    # ============================================================

    def acquire_qr(self, timeout=12.0):
        self.get_logger().info("Acquiring QR marker before charging...")

        start_time = time.time()

        while rclpy.ok() and time.time() - start_time < timeout:
            self.update_display()

            qr_detected, qr_center_x = self.get_qr_state()

            if qr_detected and qr_center_x is not None:
                self.stop_robot()
                self.get_logger().info("QR marker acquired")
                return True

            if self.front_distance <= 0.30:
                self.stop_robot()
                self.get_logger().warn("Too close to dock/wall, QR not found")
                return False

            self.velocity_publisher(0.02, 0.0)
            time.sleep(0.05)

        self.stop_robot()
        self.get_logger().warn("QR acquisition timeout")
        return False

    def align_with_qr(self, timeout=15.0):
        self.get_logger().info("Aligning with QR before charging...")

        start_time = time.time()

        while rclpy.ok() and time.time() - start_time < timeout:
            self.update_display()

            qr_detected, qr_center_x = self.get_qr_state()

            if qr_detected and qr_center_x is not None:
                error = qr_center_x - (self.image_width / 2)
                self.last_qr_error = error

                if abs(error) < 10:
                    self.stop_robot()
                    self.get_logger().info("Aligned with QR")
                    return True

                angular_speed = -error * 0.0015
                angular_speed = float(np.clip(angular_speed, -0.12, 0.12))

                self.velocity_publisher(0.0, angular_speed)

            else:
                self.get_logger().warn("QR temporarily lost. Searching...")

                if self.last_qr_error > 0:
                    search_speed = -0.08
                elif self.last_qr_error < 0:
                    search_speed = 0.08
                else:
                    search_speed = 0.08

                self.velocity_publisher(0.0, search_speed)

            time.sleep(0.04)

        self.stop_robot()
        self.get_logger().warn("QR alignment timeout")
        return False

    def dock_forward_for_charging(self, timeout=90.0):
        self.get_logger().info("Final docking for charging...")

        start_time = time.time()
        start_x, start_y = self.get_odom_position()

        while rclpy.ok() and time.time() - start_time < timeout:
            self.update_display()

            current_x, current_y = self.get_odom_position()

            moved_distance = math.sqrt(
                (current_x - start_x) ** 2 +
                (current_y - start_y) ** 2
            )

            qr_detected, qr_center_x = self.get_qr_state()

            self.get_logger().info(
                f"Charging dock | lidar={self.front_distance:.3f} m | "
                f"moved={moved_distance:.3f} m"
            )

            if self.front_distance <= self.docking_threshold:
                self.stop_robot()
                self.get_logger().info(
                    "Charging dock reached: stopped by lidar"
                )
                return True

            if moved_distance >= self.max_final_docking_distance:
                self.stop_robot()
                self.get_logger().warn(
                    "Docking stopped by max distance. "
                    "Lidar did not reach threshold."
                )
                return False

            if qr_detected and qr_center_x is not None:
                error = qr_center_x - (self.image_width / 2)
                angular_correction = -error * 0.0010
                angular_correction = float(
                    np.clip(angular_correction, -0.07, 0.07)
                )
            else:
                angular_correction = 0.0

            self.velocity_publisher(
                self.final_docking_speed,
                angular_correction
            )

            time.sleep(0.03)

        self.stop_robot()
        self.get_logger().warn("Charging docking timeout")
        return False

    # ============================================================
    # Charging
    # ============================================================

    def return_to_dock_and_charge(self, resume_after_charge=True):
        self.set_state("RETURNING_TO_DOCK")

        self.get_logger().warn(
            f"Returning to dock. Battery={self.get_battery():.2f}%"
        )

        self.clear_costmaps()

        result = "FAILED"

        for attempt in range(1, 4):
            self.get_logger().info(f"Dock return attempt {attempt}/3")

            result = self.navigate_to_pose(
                self.charge_x,
                self.charge_y,
                self.charge_yaw,
                label="charging pre-dock pose",
                monitor_battery=False,
                emergency_stop=False,
                timeout=180.0,
            )

            if result == "SUCCEEDED":
                break

            self.clear_costmaps()
            time.sleep(1.0)

        if result != "SUCCEEDED":
            self.get_logger().error("Could not reach charging pre-dock pose")
            self.stop_robot()
            self.set_state("ERROR")
            return False

        self.stop_robot()
        time.sleep(1.0)
        with self.qr_lock:
            self.qr_detected = False
            self.qr_center_x = None
            self.last_qr_center_x = None
            self.last_qr_seen_time = 0.0
            self.last_qr_error = 0.0

        self.set_state("DOCKING")

        qr_found = self.acquire_qr(timeout=12.0)

        if not qr_found:
            self.get_logger().error("QR not found. Cannot start charging.")
            self.set_state("ERROR")
            return False

        aligned = self.align_with_qr(timeout=15.0)

        if not aligned:
            self.get_logger().error("QR alignment failed. Cannot start charging.")
            self.set_state("ERROR")
            return False

        docked = self.dock_forward_for_charging(timeout=40.0)

        if not docked:
            self.get_logger().error("Final charging dock failed.")
            self.set_state("ERROR")
            return False

        self.stop_robot()
        time.sleep(1.0)

        self.set_state("CHARGING")
        self.get_logger().info("Charging started")

        while rclpy.ok() and self.get_battery() < 100.0:
            self.update_display()
            self.get_logger().info(f"Charging... {self.get_battery():.2f}%")
            time.sleep(1.0)

        with self.battery_lock:
            self.battery = 100.0

        self.get_logger().info("Battery fully charged")

        if resume_after_charge:
            self.undock_sequence(label="Undocking after charge")
            self.set_state("CLEANING")

        return True

    # ============================================================
    # Cleaning
    # ============================================================

    def clean_waypoint(self, index, waypoint):
        x, y, yaw = waypoint

        label = f"cleaning waypoint {index + 1}/{len(self.cleaning_waypoints)}"

        result = self.navigate_to_pose(
            x,
            y,
            yaw,
            label=label,
            monitor_battery=True,
            emergency_stop=True,
            timeout=180.0,
        )

        return result

    def run_cleaning_sequence(self):
        self.get_logger().info("===== VACUUM CLEANER STARTED =====")

        self.set_initial_pose()

        if not self.wait_for_map(timeout=20.0):
            self.set_state("ERROR")
            return

        self.cleaning_waypoints = self.generate_cleaning_waypoints_from_map()

        if not self.cleaning_waypoints:
            self.get_logger().error("No cleaning waypoints generated")
            self.set_state("ERROR")
            return

        # Same starting behavior as auto_docking_with_battery:
        # move backward, rotate 180 degrees, then start.
        self.undock_sequence(label="Initial undocking")
        self.set_state("CLEANING")

        while rclpy.ok() and self.current_waypoint_index < len(self.cleaning_waypoints):
            self.update_display()

            if self.is_battery_low():
                charged = self.return_to_dock_and_charge(
                    resume_after_charge=True
                )

                if not charged:
                    return

            waypoint = self.cleaning_waypoints[self.current_waypoint_index]
            result = self.clean_waypoint(
                self.current_waypoint_index,
                waypoint
            )

            if result == "SUCCEEDED":
                self.get_logger().info(
                    f"Cleaned waypoint {self.current_waypoint_index + 1}"
                )
                self.current_waypoint_index += 1

            elif result == "LOW_BATTERY":
                charged = self.return_to_dock_and_charge(
                    resume_after_charge=True
                )

                if not charged:
                    return

                # Do not increment index.
                # Resume same waypoint after charging.

            elif result == "COLLISION_RISK":
                self.get_logger().warn(
                    f"Skipping unsafe waypoint {self.current_waypoint_index + 1}"
                )
                self.clear_costmaps()
                self.current_waypoint_index += 1

            elif result in ["FAILED", "TIMEOUT"]:
                self.get_logger().warn(
                    f"Skipping unreachable waypoint {self.current_waypoint_index + 1}"
                )
                self.clear_costmaps()
                self.current_waypoint_index += 1

            time.sleep(0.2)

        self.get_logger().info("Cleaning route completed")

        # Final return to dock and charge.
        # Do not undock again after final cleaning completion.
        self.return_to_dock_and_charge(resume_after_charge=False)

        self.set_state("DONE")
        self.stop_robot()
        self.stop_battery_publisher_and_save()

        self.get_logger().info("===== VACUUM CLEANING COMPLETE =====")


def main(args=None):
    rclpy.init(args=args)

    node = VacuumCleanerWithBattery()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    executor_thread = threading.Thread(
        target=executor.spin,
        daemon=True,
    )
    executor_thread.start()

    try:
        node.run_cleaning_sequence()

        while rclpy.ok():
            node.update_display()
            time.sleep(0.2)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")

    finally:
        try:
            if rclpy.ok():
                node.stop_robot()
                time.sleep(0.2)
        except Exception:
            pass

        try:
            executor.shutdown()
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
