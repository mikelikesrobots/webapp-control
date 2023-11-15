# webapp_control

ROS2 package containing Python source code for Jetbot control using MQTT from a webapp.

This repository is intended to be checked out under the workspace provided by [jetbot-joy-teleop](https://github.com/mikelikesrobots/jetbot-joy-teleop). The package can be built using `colcon` as the standard build tool. Note that is requires `awsiotsdk` to be installed as a Python dependency to function correctly.

With thanks to the [AWS IoT Connectivity Samples for ROS2](https://github.com/aws-samples/aws-iot-robot-connectivity-samples-ros2/blob/main/.gitignore), which provided the basis for the IoTListener class in this package.
