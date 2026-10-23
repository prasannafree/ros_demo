import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class WheelSpeedSubscriber(Node):

    def __init__(self):
        super().__init__('wheel_speed_sub_node')

        self.left_speed = 0.0
        self.right_speed = 0.0

        self.left_sub = self.create_subscription(
            Float64,
            'left_wheel_speed',
            self.left_callback,
            10
        )
        self.right_sub = self.create_subscription(
            Float64,
            'right_wheel_speed',
            self.right_callback,
            10
        )

    def left_callback(self, msg: Float64):
        self.left_speed = msg.data
        self.display_speeds()

    def right_callback(self, msg: Float64):
        self.right_speed = msg.data
        self.display_speeds()

    def display_speeds(self):
        self.get_logger().info(
            f'Wheel Speeds -> Left: {self.left_speed:.2f} rad/s | '
            f'Right: {self.right_speed:.2f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = WheelSpeedSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
