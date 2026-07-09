#!/usr/bin/env python3

import time
import threading
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, Imu, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

from cv_bridge import CvBridge
import cv2
import numpy as np

from tf_transformations import euler_from_quaternion, quaternion_from_euler
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class SimpleDockingNode(Node):
    def __init__(self):
        super().__init__("simple_docking_node")

        # ----------------------------
        # Camera / QR
        # ----------------------------
        self.bridge = CvBridge()
        self.qr_decoder = cv2.QRCodeDetector()

        self.frame_lock = threading.Lock()
        self.qr_lock = threading.Lock()

        self.display_frame = None
        self.qr_detected = False
        self.qr_center_x = None
        self.image_width = 640

        # QR memory helps when QR is lost for a few frames
        self.last_qr_seen_time = 0.0
        self.last_qr_center_x = None
        self.qr_memory_time = 0.7
        self.last_qr_error = 0.0

        # QR size, used as extra final docking stop condition
        self.qr_box_width = 0
        self.qr_box_height = 0

        # ----------------------------
        # Nav2
        # ----------------------------
        self.nav = BasicNavigator()

        # ----------------------------
        # Publishers / Subscribers
        # ----------------------------
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.cam_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.camera_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            "/imu/out",
            self.imu_callback,
            10
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.lidar_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/wheel_controller/odom",
            self.odom_callback,
            10
        )

        self.battery_sub = self.create_subscription(
            Int32,
            "/battery_status",
            self.battery_callback,
            10
        )

        # ----------------------------
        # Robot state
        # ----------------------------
        self.yaw = 0.0
        self.front_distance = 100.0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_lock = threading.Lock()

        # ----------------------------
        # Docking parameters
        # ----------------------------
        self.max_final_docking_distance = 0.65
        self.target_qr_height = 9999
        self.final_docking_speed = 0.050
        self.docking_threshold = 0.160

        # Initial dock pose
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0

        # Battery
        self.battery = 100
        self.battery_lock = threading.Lock()

        self.get_logger().info("Simple Docking Node Initialized!")

    # ============================================================
    # Callbacks
    # ============================================================

    def battery_callback(self, msg):
        with self.battery_lock:
            self.battery = msg.data

    def get_battery(self):
        with self.battery_lock:
            return self.battery

    def camera_callback(self, msg):
        """
        Detect QR position only.

        Important:
        We use detect(), not detectAndDecode().
        For docking, we only need QR corner position, not QR text.
        This avoids OpenCV ECI decode warning prob0lems.
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Camera conversion failed: {e}")
            return

        if frame is None:
            return

        self.image_width = frame.shape[1]
        image_center_x = self.image_width // 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detected, points = self.qr_decoder.detect(gray)

        now = time.time()

        if detected and points is not None:
            points = points[0].astype(int)

            center_x = int(points[:, 0].mean())
            center_y = int(points[:, 1].mean())

            x, y, w, h = cv2.boundingRect(points)

            with self.qr_lock:
                self.qr_detected = True
                self.qr_center_x = center_x
                self.last_qr_center_x = center_x
                self.last_qr_seen_time = now
                self.qr_box_width = int(w)
                self.qr_box_height = int(h)

            error = center_x - image_center_x

            cv2.polylines(frame, [points], True, (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 7, (255, 0, 0), 2)
            cv2.circle(frame, (image_center_x, center_y), 5, (255, 255, 255), -1)

            cv2.putText(
                frame,
                f"QR detected | error: {error} | h: {h}",
                (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

        else:
            with self.qr_lock:
                # Keep QR valid briefly after temporary detection loss
                if now - self.last_qr_seen_time < self.qr_memory_time:
                    self.qr_detected = True
                    self.qr_center_x = self.last_qr_center_x
                else:
                    self.qr_detected = False
                    self.qr_center_x = None
                    self.qr_box_width = 0
                    self.qr_box_height = 0

            cv2.putText(
                frame,
                "QR not detected",
                (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        with self.frame_lock:
            self.display_frame = frame.copy()

    def imu_callback(self, msg):
        q = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]

        _, _, self.yaw = euler_from_quaternion(q)

    def lidar_callback(self, msg):
        """
        Wider front sector lidar check.

        The dock frame may not be exactly at 0 degrees.
        So we check a wider front area.
        """
        ranges = np.array(msg.ranges)

        # Wider front sector around 0 degrees
        front = np.concatenate([ranges[-12:], ranges[:12]])

        front = front[np.isfinite(front)]
        front = front[(front > msg.range_min) & (front < msg.range_max)]

        if len(front) > 0:
            self.front_distance = float(np.min(front))

    def odom_callback(self, msg):
        with self.odom_lock:
            self.odom_x = msg.pose.pose.position.x
            self.odom_y = msg.pose.pose.position.y

    # ============================================================
    # Utility functions
    # ============================================================

    def get_odom_position(self):
        with self.odom_lock:
            return self.odom_x, self.odom_y

    def get_qr_state(self):
        with self.qr_lock:
            now = time.time()

            if now - self.last_qr_seen_time < self.qr_memory_time:
                return True, self.last_qr_center_x

            return False, None

    def get_qr_box_size(self):
        with self.qr_lock:
            return self.qr_box_width, self.qr_box_height

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

    def update_display(self):
        with self.frame_lock:
            if self.display_frame is not None:
                cv2.imshow("QR Detection", self.display_frame)
                cv2.waitKey(1)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # ============================================================
    # Basic movement
    # ============================================================

    def move_distance(self, distance, speed=0.15):
        """
        Time-based movement.
        This is okay for simulation testing, but odom-based movement is better.
        """
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
        self.get_logger().info("Movement complete")

    def rotate_to_angle(self, target_angle, tolerance=0.035):
        self.get_logger().info(
            f"Rotating to {np.degrees(target_angle):.1f} degrees"
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

    # ============================================================
    # Docking functions
    # ============================================================

    def acquire_qr(self, timeout=10.0):
        """
        Move forward slowly after reaching pre-dock pose
        until QR becomes visible.
        """
        self.get_logger().info("Acquiring QR marker...")

        start_time = time.time()

        while rclpy.ok() and (time.time() - start_time) < timeout:
            self.update_display()

            qr_detected, qr_center_x = self.get_qr_state()

            if qr_detected and qr_center_x is not None:
                self.stop_robot()
                self.get_logger().info("QR marker acquired")
                return True

            if self.front_distance <= 0.30:
                self.stop_robot()
                self.get_logger().warn(
                    "Too close to dock/wall, but QR was not found"
                )
                return False

            self.velocity_publisher(0.025, 0.0)
            time.sleep(0.05)

        self.stop_robot()
        self.get_logger().warn("QR acquisition timeout")
        return False

    def align_with_qr(self, timeout=15.0):
        """
        Align robot heading with QR center.
        If QR is lost briefly, rotate slowly in the last useful direction.
        """
        self.get_logger().info("Aligning with QR code...")

        start_time = time.time()

        while rclpy.ok() and (time.time() - start_time) < timeout:
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
                angular_speed = np.clip(angular_speed, -0.12, 0.12)

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

    def dock_forward(self, timeout=15.0):
        """
        Final docking with multiple safety stop conditions:

        1. Lidar close distance
        2. Maximum odom travel distance
        3. QR becomes large/close

        This prevents the robot from driving into the dock frame
        if lidar does not detect the dock correctly.
        """
        self.get_logger().info("Docking forward safely...")

        start_time = time.time()
        start_x, start_y = self.get_odom_position()

        while rclpy.ok() and (time.time() - start_time) < timeout:
            self.update_display()

            current_x, current_y = self.get_odom_position()

            moved_distance = math.sqrt(
                (current_x - start_x) ** 2 +
                (current_y - start_y) ** 2
            )

            qr_detected, qr_center_x = self.get_qr_state()
            _, qr_box_height = self.get_qr_box_size()

            self.get_logger().info(
                f"Docking | lidar={self.front_distance:.3f} m | "
                f"moved={moved_distance:.3f} m | "
                f"qr_h={qr_box_height}"
            )

            # Stop 1: lidar sees dock/wall/frame
            if self.front_distance < self.docking_threshold:
                self.stop_robot()
                self.get_logger().info("Docked: stopped by lidar threshold")
                return True

            # Stop 2: safe max travel distance
            if moved_distance >= self.max_final_docking_distance:
                self.stop_robot()
                self.get_logger().warn(
                    "Stopped by max final docking distance. "
                    "Lidar did not detect dock reliably."
                )
                return True

            # Stop 3: QR is very close in camera
            if qr_box_height >= self.target_qr_height:
                self.stop_robot()
                self.get_logger().info("Docked: stopped by QR size")
                return True

            # QR steering correction
            if qr_detected and qr_center_x is not None:
                error = qr_center_x - (self.image_width / 2)
                angular_correction = -error * 0.0010
                angular_correction = np.clip(angular_correction, -0.08, 0.08)
            else:
                angular_correction = 0.0

            self.velocity_publisher(
                self.final_docking_speed,
                angular_correction
            )

            time.sleep(0.05)

        self.stop_robot()
        self.get_logger().warn("Docking timeout. Robot stopped safely.")
        return False

    # ============================================================
    # Nav2 helpers
    # ============================================================

    def create_pose_stamped(self, x, y, yaw):
        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, yaw)

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.nav.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w

        return pose

    def set_initial_pose(self, x, y, yaw):
        self.get_logger().info(
            f"Setting initial pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )

        initial_pose = self.create_pose_stamped(x, y, yaw)
        self.nav.setInitialPose(initial_pose)

        self.get_logger().info("Waiting for Nav2...")
        self.nav.waitUntilNav2Active()

        self.get_logger().info("Nav2 is active")

    def go_to_waypoint(self, x, y, yaw):
        self.get_logger().info(
            f"Going to waypoint: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )

        goal_pose = self.create_pose_stamped(x, y, yaw)
        self.nav.goToPose(goal_pose)

        while rclpy.ok() and not self.nav.isTaskComplete():
            current_battery = self.get_battery()

            if current_battery <= 5:
                self.get_logger().warn(
                    f"Battery critically low: {current_battery}%"
                )
                self.nav.cancelTask()
                self.stop_robot()
                time.sleep(1.0)
                return False

            self.update_display()
            time.sleep(0.1)

        result = self.nav.getResult()
        self.get_logger().info(f"Navigation result: {result}")

        return True

    def follow_waypoints(self, waypoints):
        self.get_logger().info(f"Following {len(waypoints)} waypoints")

        waypoint_poses = []

        for wp in waypoints:
            pose = self.create_pose_stamped(wp[0], wp[1], wp[2])
            waypoint_poses.append(pose)

        self.nav.followWaypoints(waypoint_poses)

        while rclpy.ok() and not self.nav.isTaskComplete():
            current_battery = self.get_battery()

            if current_battery <= 5:
                self.get_logger().warn(
                    f"Battery critically low: {current_battery}%"
                )
                self.nav.cancelTask()
                self.stop_robot()
                time.sleep(1.0)
                return False

            self.update_display()
            time.sleep(0.1)

        result = self.nav.getResult()
        self.get_logger().info(f"Waypoints result: {result}")
        
        if result == TaskResult.SUCCEEDED:
            return True
        
        self.get_logger().error(f"Waypoint navigation failed with result: {result}")
        return False

    # ============================================================
    # Full sequence
    # ============================================================

    def run_sequence(self):
        self.get_logger().info("===== STARTING AUTO DOCKING SEQUENCE =====")
        time.sleep(2.0)

        # --------------------------------------------------------
        # Setup initial pose
        # --------------------------------------------------------
        self.get_logger().info("[SETUP] Setting initial dock pose")

        self.initial_x = 1.5
        self.initial_y = 5.18
        self.initial_yaw = 1.57

        self.set_initial_pose(
            self.initial_x,
            self.initial_y,
            self.initial_yaw
        )

        time.sleep(2.0)

        # --------------------------------------------------------
        # Step 1: Undock
        # --------------------------------------------------------
        self.get_logger().info("[STEP 1] Undocking")

        self.move_distance(-0.6, speed=0.15)

        # --------------------------------------------------------
        # Step 2: Rotate away from dock
        # --------------------------------------------------------
        self.get_logger().info("[STEP 2] Rotating 180 degrees")

        current_yaw = self.yaw
        target_yaw = self.normalize_angle(current_yaw + math.pi)

        self.rotate_to_angle(target_yaw)

        # --------------------------------------------------------
        # Step 3: Patrol waypoints
        # --------------------------------------------------------
        self.get_logger().info("[STEP 3] Navigating to waypoints")

        waypoints = [
            (0.0, -3.5, -1.57),
            (1.0, -3.5, 0.0),
            (2.5, -2.5, 0.0),
            (2.5, -1.5, 0.0),
            (0.0, -1.5, 3.14),
            (-3.0, -2.5, 3.14),
            (-6.0, -3.5, 3.14),
            (-8.0, -4.5, 3.14),
            (-8.0, -1.5, 1.57),
            (-6.0, 0.5, 0.0),
            (-4.0, 2.0, 0.0), 
        ]

        navigation_success = self.follow_waypoints(waypoints)

        if not navigation_success:
            self.get_logger().warn(
                "Waypoint navigation was cancelled. Returning to dock."
            )
            self.stop_robot()
            return

        time.sleep(1.0)

        # --------------------------------------------------------
        # Step 4: Return to pre-dock staging pose
        # --------------------------------------------------------
        self.get_logger().info("[STEP 4] Returning to pre-dock staging pose")

        pre_dock_x = self.initial_x
        pre_dock_y = self.initial_y - 0.55
        pre_dock_yaw = self.initial_yaw

        reached_predock = self.go_to_waypoint(
            pre_dock_x,
            pre_dock_y,
            pre_dock_yaw
        )

        if not reached_predock:
            self.get_logger().error("Failed to reach pre-dock pose")
            self.stop_robot()
            return

        time.sleep(1.0)

        # --------------------------------------------------------
        # Step 5: Acquire QR
        # --------------------------------------------------------
        self.get_logger().info("[STEP 5] Acquiring QR marker")

        qr_found = self.acquire_qr(timeout=10.0)

        if not qr_found:
            self.get_logger().error("QR not found. Aborting docking.")
            self.stop_robot()
            return

        # --------------------------------------------------------
        # Step 6: Align QR
        # --------------------------------------------------------
        self.get_logger().info("[STEP 6] Aligning with QR")

        aligned = self.align_with_qr(timeout=15.0)

        if not aligned:
            self.get_logger().error("QR alignment failed. Aborting docking.")
            self.stop_robot()
            return

        time.sleep(0.5)

        # --------------------------------------------------------
        # Step 7: Final docking
        # --------------------------------------------------------
        self.get_logger().info("[STEP 7] Final docking")

        docked = self.dock_forward(timeout=40.0)

        if docked:
            self.get_logger().info("===== SEQUENCE COMPLETE: DOCKED =====")
        else:
            self.get_logger().warn(
                "===== SEQUENCE COMPLETE: DOCKING FAILED SAFELY ====="
            )

        self.stop_robot()


def main(args=None):
    rclpy.init(args=args)

    node = SimpleDockingNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    executor_thread = threading.Thread(
        target=executor.spin,
        daemon=True
    )
    executor_thread.start()

    try:
        node.run_sequence()

        while rclpy.ok():
            node.update_display()
            time.sleep(0.1)

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