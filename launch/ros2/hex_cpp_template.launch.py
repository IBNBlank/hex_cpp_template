#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-03-29
################################################################

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    hex_cpp_template_param = os.path.join(
        get_package_share_directory("hex_cpp_template"), "config/ros2",
        "hex_cpp_template.yaml")

    hex_cpp_template = Node(name="hex_cpp_template",
                            package="hex_cpp_template",
                            executable="hex_cpp_template",
                            output="screen",
                            parameters=[hex_cpp_template_param
                       ],
                            remappings=[("/string_in", "/in"),
                                        ("/string_out", "/out"),
                                        ("/odom", "/odom")])

    return LaunchDescription([hex_cpp_template])
