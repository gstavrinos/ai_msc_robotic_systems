#!/usr/bin/env python3
import os
import launch
from launch_ros.actions import Node

def generate_launch_description():

    ball_locator_node = Node(
        package="tt_ball_locator",
        executable='tt_ball_locator.py',
        name="tt_ball_locator",
    )

    return launch.LaunchDescription([
        ball_locator_node,
    ])

