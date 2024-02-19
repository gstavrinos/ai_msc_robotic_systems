#!/usr/bin/env python3
import cv2
import math
import rclpy
import numpy as np
import message_filters
from cv_bridge import CvBridge

from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from tf2_ros import StaticTransformBroadcaster

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped

class Assignment1Node(Node):
    def __init__(self):
        self.bridge = CvBridge()
        super().__init__("tt_ball_locator")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring...")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating...")

        image_sub = message_filters.Subscriber(self, Image, "/head_front_camera/rgb/image_raw")
        point_sub = message_filters.Subscriber(self, PointCloud2, "/head_front_camera/depth_registered/points")

        ts = message_filters.ApproximateTimeSynchronizer([image_sub, point_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.camera_callback)
        
        return super().on_activate(state)

    def camera_callback(self, image_msg, points_msg):
        # TODO

        x = 0
        y = 0
        z = 0

        broadcaster = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "?"
        transform.child_frame_id = "??"
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    assignment1_node = Assignment1Node()
    rclpy.spin(assignment1_node)

if __name__ == "__main__":
    main()
