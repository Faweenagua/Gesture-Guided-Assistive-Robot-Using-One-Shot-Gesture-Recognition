#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
import time

class TurtlebotGestureController(Node):
    def __init__(self):
        super().__init__('turtlebot_gesture_controller')

        # Subscribe to gesture recognition output
        self.subscription = self.create_subscription(
            String,
            '/gesture_recognition/result',
            self.gesture_callback,
            10
        )

        # Publisher for TurtleBot velocity commands
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Movement parameters
        self.linear_speed = 0.15
        self.angular_speed = 0.5  # rad/s

        self.get_logger().info("TurtleBot Gesture Controller Node Started.")

    # ----------------------------------------------------------------
    # Helper function: publish velocity once
    # ----------------------------------------------------------------
    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        msg = TwistStamped()
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self.cmd_pub.publish(msg)

    # ----------------------------------------------------------------
    # Helper function: rotate the robot by exact angle
    # ----------------------------------------------------------------
    def rotate_angle(self, angle_deg):
        """
        Rotate the robot by specified angle in degrees (+ left, – right)
        using time = angle / angular_speed
        """

        angle_rad = abs(angle_deg) * 3.14159 / 180.0
        rotation_time = angle_rad / self.angular_speed

        self.get_logger().info(
            f"Rotating {angle_deg}° → estimated time {rotation_time:.2f} sec"
        )

        start_time = time.time()

        # Choose rotation direction
        direction = 1.0 if angle_deg > 0 else -1.0  

        # Publish angular velocity for the required time
        while time.time() - start_time < rotation_time:
            self.publish_cmd(angular_z=direction * self.angular_speed)
            rclpy.spin_once(self, timeout_sec=0.01)

        # Stop
        self.publish_cmd(0.0, 0.0)

    # ----------------------------------------------------------------
    # Main callback
    # ----------------------------------------------------------------
    def gesture_callback(self, msg):
        gesture = msg.data.strip()
        self.get_logger().info(f"Gesture received: {gesture}")

        # --------------------------------------------------------------
        # Forward / Backward (continuous movement)
        # --------------------------------------------------------------
        if gesture == "forward":
            self.publish_cmd(linear_x=self.linear_speed)

        elif gesture == "backward":
            self.publish_cmd(linear_x=-self.linear_speed)

        # --------------------------------------------------------------
        # Left / Right → turn 90° (discrete)
        # --------------------------------------------------------------
        elif gesture == "left":
            self.rotate_angle(90)       # left turn

        elif gesture == "right":
            self.rotate_angle(-90)      # right turn

        # --------------------------------------------------------------
        # Stop
        # --------------------------------------------------------------
        elif gesture == "stop":
            self.publish_cmd(0.0, 0.0)

        else:
            self.get_logger().warn(f"Unknown gesture: {gesture}")
            self.publish_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotGestureController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
