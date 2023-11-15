#!/usr/bin/env python3
#
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files (the "Software"), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#

import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
from awscrt import mqtt, io
from awsiot import mqtt_connection_builder
from awsiot.greengrass_discovery import DiscoveryClient

RETRY_WAIT_TIME_SECONDS = 5


class IoTListener(Node):
    def __init__(self):
        super().__init__('mqtt_publisher')

        self.declare_parameter("path_for_config", "")
        self.declare_parameter("discover_endpoints", False)

        path_for_config = self.get_parameter("path_for_config").get_parameter_value().string_value

        with open(path_for_config) as f:
          cert_data = json.load(f)

        self.get_logger().info("Config we are loading is :\n{}".format(cert_data))

        self.get_logger().info("Connecting directly to endpoint")
        self.connect_to_endpoint(cert_data)

        self._cmd_vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )
        self.get_logger().info("Created publisher")
        
        self.init_subs()
        self.get_logger().info("Initialized subscription")

    def connect_to_endpoint(self, cert_data):
        self.mqtt_conn = mqtt_connection_builder.mtls_from_path(
            endpoint=cert_data["endpoint"],
            port= cert_data["port"],
            cert_filepath= cert_data["certificatePath"],
            pri_key_filepath= cert_data["privateKeyPath"],
            ca_filepath= cert_data["rootCAPath"],
            client_id= cert_data["clientID"],
            http_proxy_options=None,
        )
        connected_future = self.mqtt_conn.connect()
        connected_future.result()
        self.get_logger().info("Connected!")

    def init_subs(self):
        """Subscribe to jetbot command topic"""
        self._sub_future, _ = self.mqtt_conn.subscribe(
            topic="robot/jetbot1/command",
            qos=mqtt.QoS.AT_LEAST_ONCE,
            callback=self.on_command_received
        )
        self._sub_result = self._sub_future.result()
        self.get_logger().info(json.dumps(self._sub_result))

    def on_command_received(self, topic, payload, *args, **kwargs):
        """Callback for the jetbot command topic"""
        self.get_logger().info("Received command: {}".format(payload))
        parsed = json.loads(payload)

        twist_msg = Twist()
        enabled = parsed["enabled"]
        direction = parsed["direction"]
        if enabled:
            if direction == "up":
                twist_msg.linear.x += 0.5
            elif direction == "down":
                twist_msg.linear.x -= 0.5
            elif direction == "right":
                twist_msg.angular.z -= 0.5
            elif direction == "left":
                twist_msg.angular.z += 0.5

        self._cmd_vel_pub.publish(twist_msg)
