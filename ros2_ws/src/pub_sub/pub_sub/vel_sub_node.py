from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class VelSubscriber(Node):

    def __init__(self):
        super().__init__('vel_sub_node')

        # Current estimated position
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # Latest received linear velocity
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # Store the time of the previous position update
        self.last_time = self.get_clock().now()

        # Subscribe to velocity
        self.subscription = self.create_subscription(
            Twist,
            'vel_topic',
            self.velocity_callback,
            25
        )

        # Print current position every 5 seconds
        self.timer = self.create_timer(
            5.0,
            self.timer_callback
        )

    def velocity_callback(self, msg):
        # Update the latest velocity
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vz = msg.linear.z

    def timer_callback(self):
        # Calculate elapsed time in seconds
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # Integrate velocity to estimate position
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt

        self.last_time = current_time

        self.get_logger().info(
            f'Position: x={self.x:.3f} m, '
            f'y={self.y:.3f} m, z={self.z:.3f} m'
        )


def main(args=None):
    rclpy.init(args=args)

    node = VelSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
