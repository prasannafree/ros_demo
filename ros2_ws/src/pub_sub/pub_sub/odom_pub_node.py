from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class OdomPublisher(Node):

    def __init__(self):
        super().__init__('odom_pub_node')
        self.publisher = self.create_publisher(Twist, 'vel_topic', 25)
        self.timer = self.create_timer(1.0, self.publish_velocity)
        self.linear_vel = 0.5
        self.angular_vel = 0.0

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.angular.z = self.angular_vel

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: Linear={msg.linear.x}, Angular={msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    try:
        rclpy.spin(odom_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        odom_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
