#!/usr/bin/env python3

import json
import math
import os
import random
import time
import threading
from pathlib import Path

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

from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu, LaserScan, Image
from std_msgs.msg import Int32, Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class VacuumCleanerWithBatteryRL(Node):
    def __init__(self):
        super().__init__("Rl_battery_docking_nodew.")

        self.nav = BasicNavigator()

        # ============================================================
        # Publishers
        # ============================================================
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.battery_int_pub = self.create_publisher(Int32, "/battery_status", 10)
        self.battery_float_pub = self.create_publisher(
            Float32,
            "/vacuum/battery_percent",
            10,
        )
        self.status_pub = self.create_publisher(String, "/vacuum/status", 10)

        marker_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/mechabot/rl_markers",
            marker_qos,
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

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            10,
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
        # Robot pose / state
        # ============================================================
        self.yaw = 0.0
        self.front_distance = 100.0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_lock = threading.Lock()

        self.map_x = 0.0
        self.map_y = 0.0
        self.map_pose_received = False
        self.map_pose_lock = threading.Lock()

        self.state = "IDLE"
        self.state_lock = threading.Lock()

        # ============================================================
        # Battery
        # ============================================================
        self.battery = 100.0
        self.battery_lock = threading.Lock()

        self.battery_drain_per_second = 0.25
        self.charge_rate_per_second = 2.0
        self.low_battery_threshold = 45.0

        self.battery_publishing_enabled = True
        self.final_battery_saved = False

        script_dir = Path(__file__).resolve().parent
        self.final_battery_file = str(script_dir / "final_battery_reading.json")

        self.battery_timer = self.create_timer(
            1.0,
            self.battery_timer_callback,
        )

        # ============================================================
        # Dock pose
        # ============================================================
        self.dock_x = 1.5
        self.dock_y = 5.18
        self.dock_yaw = 1.57

        # Far approach pose.
        # Robot first comes here using Nav2.
        self.dock_approach_distance = 1.30
        self.dock_approach_x = (
            self.dock_x
            - self.dock_approach_distance * math.cos(self.dock_yaw)
        )
        self.dock_approach_y = (
            self.dock_y
            - self.dock_approach_distance * math.sin(self.dock_yaw)
        )
        self.dock_approach_yaw = self.dock_yaw

        # Pre-dock distance.
        # Robot reaches this using manual straight movement, not Nav2.
        self.charge_standoff_distance = 0.60
        self.charge_x = (
            self.dock_x
            - self.charge_standoff_distance * math.cos(self.dock_yaw)
        )
        self.charge_y = (
            self.dock_y
            - self.charge_standoff_distance * math.sin(self.dock_yaw)
        )
        self.charge_yaw = self.dock_yaw

        # ============================================================
        # Final QR / lidar docking
        # ============================================================
        self.docking_threshold = 0.15
        self.final_docking_speed = 0.025
        self.close_docking_speed = 0.008
        self.slowdown_lidar_distance = 0.28
        self.max_final_docking_distance = 1.00

        # ============================================================
        # Cleaning
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

        # ============================================================
        # RL memory
        # ============================================================
        self.rl_memory_file = str(script_dir / "cleaning_rl_memory.json")

        self.q_table = {}
        self.bad_zones = []

        self.rl_actions = [
            "NAVIGATE",
            "CLEAR_AND_NAVIGATE",
            "SKIP",
        ]

        self.rl_alpha = 0.35
        self.rl_gamma = 0.80
        self.rl_epsilon = 0.08

        self.bad_zone_radius = 0.75
        self.rl_grid_size = 0.50

        self.load_rl_memory()

        self.marker_timer = self.create_timer(
            2.0,
            self.publish_markers,
        )

        self.get_logger().info(
            "Vacuum Cleaner With Battery + RL node initialized"
        )

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

    def set_error_and_stop(self, reason):
        self.get_logger().error(reason)
        self.set_state("ERROR")
        self.stop_robot()
        self.save_rl_memory()
        self.stop_battery_publisher_and_save()

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

    def amcl_callback(self, msg):
        with self.map_pose_lock:
            self.map_x = msg.pose.pose.position.x
            self.map_y = msg.pose.pose.position.y
            self.map_pose_received = True

    def camera_callback(self, msg):
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

        # QR detection only during DOCKING.
        # This prevents OpenCV QR detector crashes during cleaning.
        if current_state != "DOCKING":
            with self.frame_lock:
                self.display_frame = frame.copy()
            return

        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
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
        if self.final_battery_saved:
            return

        with self.battery_lock:
            final_battery = float(self.battery)

        data = {
            "final_battery_percent": final_battery,
            "state": self.get_state(),
            "timestamp": time.time(),
        }

        try:
            os.makedirs(os.path.dirname(self.final_battery_file), exist_ok=True)

            with open(self.final_battery_file, "w") as f:
                json.dump(data, f, indent=4)

            self.get_logger().info(
                f"Final battery saved: {final_battery:.2f}% -> {self.final_battery_file}"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to save final battery reading: {e}")

        self.battery_publishing_enabled = False
        self.final_battery_saved = True

        try:
            self.battery_timer.cancel()
        except Exception:
            pass

        self.get_logger().info("Battery publisher stopped")

    # ============================================================
    # Utility
    # ============================================================

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def get_odom_position(self):
        with self.odom_lock:
            return self.odom_x, self.odom_y

    def get_robot_map_position(self):
        with self.map_pose_lock:
            if self.map_pose_received:
                return self.map_x, self.map_y

        return self.get_odom_position()

    def get_qr_state(self):
        with self.qr_lock:
            now = time.time()

            if now - self.last_qr_seen_time < self.qr_memory_time:
                return True, self.last_qr_center_x

            return False, None

    def reset_qr_state(self):
        with self.qr_lock:
            self.qr_detected = False
            self.qr_center_x = None
            self.last_qr_center_x = None
            self.last_qr_seen_time = 0.0
            self.last_qr_error = 0.0

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
    # RViz markers
    # ============================================================

    def publish_markers(self):
        marker_array = MarkerArray()

        # Spawn / dock marker
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
        text_marker.text = "Spawn / Charging Dock"
        text_marker.lifetime.sec = 0
        marker_array.markers.append(text_marker)

        # Dock approach marker
        approach_marker = Marker()
        approach_marker.header.frame_id = "map"
        approach_marker.header.stamp = self.get_clock().now().to_msg()
        approach_marker.ns = "dock_approach"
        approach_marker.id = 2
        approach_marker.type = Marker.SPHERE
        approach_marker.action = Marker.ADD
        approach_marker.pose.position.x = self.dock_approach_x
        approach_marker.pose.position.y = self.dock_approach_y
        approach_marker.pose.position.z = 0.15
        approach_marker.pose.orientation.w = 1.0
        approach_marker.scale.x = 0.25
        approach_marker.scale.y = 0.25
        approach_marker.scale.z = 0.25
        approach_marker.color.r = 0.0
        approach_marker.color.g = 0.4
        approach_marker.color.b = 1.0
        approach_marker.color.a = 0.9
        approach_marker.lifetime.sec = 0
        marker_array.markers.append(approach_marker)

        # Pre-dock marker
        pre_dock_marker = Marker()
        pre_dock_marker.header.frame_id = "map"
        pre_dock_marker.header.stamp = self.get_clock().now().to_msg()
        pre_dock_marker.ns = "dock_prepose"
        pre_dock_marker.id = 3
        pre_dock_marker.type = Marker.SPHERE
        pre_dock_marker.action = Marker.ADD
        pre_dock_marker.pose.position.x = self.charge_x
        pre_dock_marker.pose.position.y = self.charge_y
        pre_dock_marker.pose.position.z = 0.15
        pre_dock_marker.pose.orientation.w = 1.0
        pre_dock_marker.scale.x = 0.22
        pre_dock_marker.scale.y = 0.22
        pre_dock_marker.scale.z = 0.22
        pre_dock_marker.color.r = 1.0
        pre_dock_marker.color.g = 1.0
        pre_dock_marker.color.b = 0.0
        pre_dock_marker.color.a = 0.9
        pre_dock_marker.lifetime.sec = 0
        marker_array.markers.append(pre_dock_marker)

        # Learned bad zones
        for i, zone in enumerate(self.bad_zones):
            bad = Marker()
            bad.header.frame_id = "map"
            bad.header.stamp = self.get_clock().now().to_msg()
            bad.ns = "rl_bad_zones"
            bad.id = 100 + i
            bad.type = Marker.CYLINDER
            bad.action = Marker.ADD
            bad.pose.position.x = float(zone["x"])
            bad.pose.position.y = float(zone["y"])
            bad.pose.position.z = 0.04
            bad.pose.orientation.w = 1.0

            radius = float(zone.get("radius", self.bad_zone_radius))
            bad.scale.x = radius * 2.0
            bad.scale.y = radius * 2.0
            bad.scale.z = 0.04

            bad.color.r = 1.0
            bad.color.g = 0.0
            bad.color.b = 0.0
            bad.color.a = 0.35
            bad.lifetime.sec = 0
            marker_array.markers.append(bad)

        self.marker_pub.publish(marker_array)

    # ============================================================
    # RL memory
    # ============================================================

    def load_rl_memory(self):
        if not os.path.exists(self.rl_memory_file):
            self.q_table = {}
            self.bad_zones = []
            self.get_logger().info("No previous RL memory found")
            return

        try:
            with open(self.rl_memory_file, "r") as f:
                data = json.load(f)

            self.q_table = data.get("q_table", {})
            self.bad_zones = data.get("bad_zones", [])

            self.get_logger().info(
                f"Loaded RL memory: {len(self.q_table)} states, "
                f"{len(self.bad_zones)} bad zones"
            )

        except Exception as e:
            self.q_table = {}
            self.bad_zones = []
            self.get_logger().warn(f"Could not load RL memory: {e}")

    def save_rl_memory(self):
        try:
            os.makedirs(os.path.dirname(self.rl_memory_file), exist_ok=True)

            data = {
                "q_table": self.q_table,
                "bad_zones": self.bad_zones,
                "updated_at": time.time(),
            }

            with open(self.rl_memory_file, "w") as f:
                json.dump(data, f, indent=4)

        except Exception as e:
            self.get_logger().error(f"Could not save RL memory: {e}")

    def discretize_position(self, x, y):
        gx = round(x / self.rl_grid_size) * self.rl_grid_size
        gy = round(y / self.rl_grid_size) * self.rl_grid_size
        return gx, gy

    def obstacle_band(self):
        if self.front_distance < 0.20:
            return "very_close"
        if self.front_distance < 0.40:
            return "close"
        if self.front_distance < 0.80:
            return "medium"
        return "clear"

    def battery_band(self):
        if self.get_battery() <= self.low_battery_threshold:
            return "low"
        return "ok"

    def make_rl_state(self, waypoint):
        x, y, _ = waypoint
        gx, gy = self.discretize_position(x, y)

        near_bad = 1 if self.is_near_bad_zone(x, y) else 0

        state = (
            f"x={gx:.1f}|y={gy:.1f}|"
            f"front={self.obstacle_band()}|"
            f"bad={near_bad}|"
            f"battery={self.battery_band()}"
        )

        return state

    def get_q_values(self, state):
        if state not in self.q_table:
            self.q_table[state] = {
                action: 0.0 for action in self.rl_actions
            }

        return self.q_table[state]

    def choose_rl_action(self, state, waypoint):
        x, y, _ = waypoint

        if self.is_near_bad_zone(x, y):
            return "SKIP"

        q_values = self.get_q_values(state)

        if random.random() < self.rl_epsilon:
            return random.choice(self.rl_actions)

        return max(q_values, key=q_values.get)

    def update_rl(self, state, action, reward, next_state=None):
        q_values = self.get_q_values(state)
        old_value = q_values[action]

        if next_state is None:
            future_value = 0.0
        else:
            next_q_values = self.get_q_values(next_state)
            future_value = max(next_q_values.values())

        new_value = old_value + self.rl_alpha * (
            reward + self.rl_gamma * future_value - old_value
        )

        self.q_table[state][action] = new_value

        self.get_logger().info(
            f"RL update | action={action} | reward={reward:.1f} | Q={new_value:.2f}"
        )

        self.save_rl_memory()

    def record_bad_zone(self, x=None, y=None, reason="unknown"):
        if x is None or y is None:
            x, y = self.get_robot_map_position()

        for zone in self.bad_zones:
            d = math.sqrt((x - zone["x"]) ** 2 + (y - zone["y"]) ** 2)

            if d < self.bad_zone_radius:
                zone["count"] = int(zone.get("count", 1)) + 1
                zone["reason"] = reason
                zone["updated_at"] = time.time()

                self.save_rl_memory()
                self.publish_markers()

                self.get_logger().warn(
                    f"Updated bad zone at x={zone['x']:.2f}, "
                    f"y={zone['y']:.2f}, reason={reason}"
                )
                return

        zone = {
            "x": float(x),
            "y": float(y),
            "radius": float(self.bad_zone_radius),
            "reason": reason,
            "count": 1,
            "created_at": time.time(),
            "updated_at": time.time(),
        }

        self.bad_zones.append(zone)
        self.save_rl_memory()
        self.publish_markers()

        self.get_logger().warn(
            f"Recorded new bad zone: x={x:.2f}, y={y:.2f}, reason={reason}"
        )

    def is_near_bad_zone(self, x, y):
        for zone in self.bad_zones:
            radius = float(zone.get("radius", self.bad_zone_radius))
            d = math.sqrt((x - zone["x"]) ** 2 + (y - zone["y"]) ** 2)

            if d < radius:
                return True

        return False

    # ============================================================
    # Basic motion
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

    def move_straight_with_odom_and_lidar(self, distance, speed=0.08, min_lidar=0.32):
        """
        Move straight using odom distance with lidar safety.
        Used only for dock approach lane.

        This prevents Nav2 from making a side/cross path near the dock.
        """

        self.get_logger().info(
            f"Straight dock-lane move: distance={distance:.2f} m, speed={speed:.2f} m/s"
        )

        start_x, start_y = self.get_odom_position()
        direction_speed = speed if distance > 0 else -speed
        target_distance = abs(distance)

        while rclpy.ok():
            self.update_display()

            current_x, current_y = self.get_odom_position()

            moved_distance = math.sqrt(
                (current_x - start_x) ** 2 +
                (current_y - start_y) ** 2
            )

            self.get_logger().info(
                f"Dock-lane move | moved={moved_distance:.3f} m | "
                f"lidar={self.front_distance:.3f} m"
            )

            if moved_distance >= target_distance:
                self.stop_robot()
                self.get_logger().info("Dock-lane straight move complete")
                return True

            if self.front_distance <= min_lidar:
                self.stop_robot()
                self.get_logger().warn(
                    f"Stopped dock-lane move early. Lidar={self.front_distance:.3f} m"
                )
                return True

            self.velocity_publisher(direction_speed, 0.0)
            time.sleep(0.03)

        self.stop_robot()
        return False

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

        self.publish_markers()

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
    # Dock path alignment
    # ============================================================

    def prepare_straight_docking_path(self):
        """
        Docking path:

        1. Nav2 goes to far dock approach pose.
        2. Robot rotates once to face the dock.
        3. Robot manually moves straight to pre-dock.
        4. Robot does NOT rotate again.
        5. QR docking starts immediately.

        This fixes the issue where the robot turned away from the QR at pre-dock.
        """

        self.get_logger().info("Preparing straight docking path...")

        self.clear_costmaps()

        # Step 1: Nav2 only to far approach pose
        result = self.navigate_to_pose(
            self.dock_approach_x,
            self.dock_approach_y,
            self.dock_approach_yaw,
            label="dock approach pose",
            monitor_battery=False,
            emergency_stop=False,
            timeout=180.0,
        )

        if result != "SUCCEEDED":
            self.get_logger().error("Failed to reach dock approach pose")
            return False

        self.stop_robot()
        time.sleep(0.5)

        # Step 2: rotate once at far approach pose
        self.rotate_to_angle(self.dock_yaw, tolerance=0.025)

        self.stop_robot()
        time.sleep(0.5)

        # Step 3: manually move straight to pre-dock
        straight_distance = (
            self.dock_approach_distance -
            self.charge_standoff_distance
        )

        moved_ok = self.move_straight_with_odom_and_lidar(
            distance=straight_distance,
            speed=0.08,
            min_lidar=0.32,
        )

        if not moved_ok:
            self.get_logger().error("Failed during straight dock-lane movement")
            return False

        self.stop_robot()
        time.sleep(0.5)

        # Important:
        # Do NOT rotate here.
        # The camera must keep facing the QR.
        self.get_logger().info(
            "Robot reached pre-dock. Starting QR docking without extra rotation."
        )

        return True

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
        if self.distance_to_dock(x, y) < self.exclude_dock_radius:
            return True

        # Manual exclusion for clutter area
        if -10.0 <= x <= -6.0 and -7.0 <= y <= -3.5:
            return True

        # Learned bad zones
        if self.is_near_bad_zone(x, y):
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
    # QR / lidar final docking
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

    def dock_forward_for_charging(self, timeout=70.0):
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
                    f"Charging dock reached: lidar={self.front_distance:.3f} m"
                )
                return True

            if moved_distance >= self.max_final_docking_distance:
                self.stop_robot()
                self.get_logger().warn(
                    "Docking stopped by max distance. "
                    "Robot did not reach 0.15 m lidar threshold."
                )
                return False

            if self.front_distance <= self.slowdown_lidar_distance:
                forward_speed = self.close_docking_speed
            else:
                forward_speed = self.final_docking_speed

            if qr_detected and qr_center_x is not None:
                error = qr_center_x - (self.image_width / 2)

                if self.front_distance <= 0.25:
                    angular_correction = -error * 0.0006
                    angular_correction = float(
                        np.clip(angular_correction, -0.03, 0.03)
                    )
                else:
                    angular_correction = -error * 0.0010
                    angular_correction = float(
                        np.clip(angular_correction, -0.07, 0.07)
                    )
            else:
                angular_correction = 0.0

            self.velocity_publisher(
                forward_speed,
                angular_correction,
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

        aligned_path = False

        for attempt in range(1, 4):
            self.get_logger().info(f"Straight dock approach attempt {attempt}/3")

            aligned_path = self.prepare_straight_docking_path()

            if aligned_path:
                break

            self.clear_costmaps()
            time.sleep(1.0)

        if not aligned_path:
            self.set_error_and_stop("Could not prepare straight docking path")
            return False

        self.stop_robot()
        time.sleep(1.0)

        self.reset_qr_state()
        self.set_state("DOCKING")

        qr_found = self.acquire_qr(timeout=12.0)

        if not qr_found:
            self.set_error_and_stop("QR not found. Cannot start charging.")
            return False

        aligned = self.align_with_qr(timeout=15.0)

        if not aligned:
            self.set_error_and_stop("QR alignment failed. Cannot start charging.")
            return False

        docked = self.dock_forward_for_charging(timeout=70.0)

        if not docked:
            self.set_error_and_stop("Final charging dock failed.")
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
    # Cleaning with RL
    # ============================================================

    def reward_for_result(self, result):
        if result == "SUCCEEDED":
            return 10.0

        if result == "LOW_BATTERY":
            return 0.0

        if result == "COLLISION_RISK":
            return -80.0

        if result == "TIMEOUT":
            return -35.0

        if result == "FAILED":
            return -30.0

        if result == "SKIPPED":
            return -2.0

        return -5.0

    def clean_waypoint_with_rl(self, index, waypoint):
        state = self.make_rl_state(waypoint)
        action = self.choose_rl_action(state, waypoint)

        x, y, yaw = waypoint

        self.get_logger().info(
            f"RL decision for waypoint {index + 1}/{len(self.cleaning_waypoints)}: "
            f"action={action}, state={state}"
        )

        if action == "SKIP":
            result = "SKIPPED"
            reward = self.reward_for_result(result)
            self.update_rl(state, action, reward)
            return result

        if action == "CLEAR_AND_NAVIGATE":
            self.clear_costmaps()
            time.sleep(0.5)

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

        reward = self.reward_for_result(result)
        self.update_rl(state, action, reward)

        if result == "COLLISION_RISK":
            rx, ry = self.get_robot_map_position()
            self.record_bad_zone(rx, ry, reason="collision_risk")

        elif result in ["FAILED", "TIMEOUT"]:
            self.record_bad_zone(x, y, reason=result.lower())

        return result

    def run_cleaning_sequence(self):
        self.get_logger().info("===== VACUUM CLEANER WITH RL STARTED =====")

        self.set_initial_pose()

        if not self.wait_for_map(timeout=20.0):
            self.set_error_and_stop("Map not received. Mission stopped.")
            return

        self.cleaning_waypoints = self.generate_cleaning_waypoints_from_map()

        if not self.cleaning_waypoints:
            self.set_error_and_stop("No cleaning waypoints generated. Mission stopped.")
            return

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

            result = self.clean_waypoint_with_rl(
                self.current_waypoint_index,
                waypoint,
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

            elif result in ["SKIPPED", "COLLISION_RISK", "FAILED", "TIMEOUT"]:
                self.get_logger().warn(
                    f"Moving past waypoint {self.current_waypoint_index + 1} "
                    f"due to result={result}"
                )
                self.clear_costmaps()
                self.current_waypoint_index += 1

            else:
                self.get_logger().warn(
                    f"Unknown result={result}, skipping waypoint"
                )
                self.current_waypoint_index += 1

            time.sleep(0.2)

        self.get_logger().info("Cleaning route completed")

        final_docked = self.return_to_dock_and_charge(resume_after_charge=False)

        if not final_docked:
            return

        self.set_state("DONE")
        self.stop_robot()

        self.save_rl_memory()
        self.stop_battery_publisher_and_save()

        self.get_logger().info("===== VACUUM CLEANING COMPLETE =====")


def main(args=None):
    rclpy.init(args=args)

    node = VacuumCleanerWithBatteryRL()

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
                node.save_rl_memory()
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
