#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

import math

class TagNavigator(Node):

    def __init__(self):
        super().__init__('tag_navigator')

        # Publisher to send movement commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create a buffer and listener for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to regularly check for the tag
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Parameters
        self.target_frame = 'goal'       # Frame ID of the AprilTag
        self.base_frame = 'base_link'    # Robot's base frame ID
        self.stop_distance = 0.20        # Desired stopping distance (in meters)
        self.searching = True            # Flag to indicate search mode

    def timer_callback(self):
        try:
            # Look up the transformation from the robot base to the target frame
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(self.base_frame,
                                                    self.target_frame,
                                                    now)
            self.searching = False

            # Calculate distance and angle to the tag
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            distance = math.hypot(x, y)
            angle = math.atan2(y, x)

            self.get_logger().info(f"Tag found at distance: {distance:.2f} m, angle: {math.degrees(angle):.2f} degrees")

            # Prepare the movement command
            twist = Twist()

            if distance > self.stop_distance:
                # Move towards the tag
                twist.linear.x = min(0.2, 0.5 * (distance - self.stop_distance))
                twist.angular.z = 2.0 * angle
            else:
                # Stop the robot
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("Reached stopping distance from the tag.")

            self.cmd_vel_pub.publish(twist)

        except (LookupException, ConnectivityException, ExtrapolationException):
            if not self.searching:
                self.get_logger().info("Tag lost. Searching...")
            self.searching = True
            # Rotate to search for the tag
            twist = Twist()
            twist.angular.z = 0.2  # Adjust rotation speed as needed
            self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    node = TagNavigator()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

