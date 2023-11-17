import rclpy
from webapp_control.iot_listener import IoTListener


def main(args=None):
    rclpy.init(args=args)
    iot_listener = IoTListener()
    rclpy.spin(iot_listener)
    iot_listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
