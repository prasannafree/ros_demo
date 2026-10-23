import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class LeftWheelPublisher(Node):

    def __init__(self):
        super().__init__('left_wheel_pub_node')
        self.publisher = self.create_publisher(Float64, 'left_wheel_speed', 10)
        self.timer = self.create_timer(1.0, self.publish_speed)
        self.speed = 10.0

    def publish_speed(self):
        msg = Float64()
        msg.data = self.speed
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing Left Wheel Speed: {msg.data:.2f} rad/s')


def main(args=None):
    rclpy.init(args=args)
    node = LeftWheelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
