#!/usr/bin/env python3
import rclpy
import tf2_ros
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.time import Time
import threading
import time
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from math import pi, cos, sin, atan2
from std_srvs.srv import Empty


class Assignment2Node(Node):

    def __init__(self):
        super().__init__("tt_table_explorer")


    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring...")
        return TransitionCallbackReturn.SUCCESS


    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating...")
        self.client = self.create_client(Empty, "/tt_umpire/assignment2/i_feel_confident")
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_pose_callback,
            10
        )
        #self.table_explorer_thread = threading.Thread(target=self.run)
        #self.table_explorer_thread.start()

        return super().on_activate(state)

    def amcl_pose_callback(self, msg):
        print(msg)

    def run(self):
        # TODO
        self.request = Empty.Request()


def main():
    rclpy.init()
    assignment2_node = Assignment2Node()
    rclpy.spin(assignment2_node)

if __name__ == "__main__":
  main()
