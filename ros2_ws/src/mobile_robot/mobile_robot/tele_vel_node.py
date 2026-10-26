
"""Convert keyboard commands into differential-drive wheel velocities."""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray, Float64


class TeleVelNode(Node):

    def __init__(self):
        super().__init__('tele_vel_node')

        # --------------------------------------------------
        # 1. Parameters
        # --------------------------------------------------

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation', 0.30)
        self.declare_parameter('max_linear_speed', 0.30)
        self.declare_parameter('max_angular_speed', 1.0)

        # --------------------------------------------------
        # 2. Publishers: separate topic for each wheel
        # --------------------------------------------------

        self.left_wheel_pub = self.create_publisher(
            Float64,
            '/left_wheel_speed',
            10
        )

        self.right_wheel_pub = self.create_publisher(
            Float64,
            '/right_wheel_speed',
            10
        )

        # --------------------------------------------------
        # 3. Subscriber: keyboard commands
        # --------------------------------------------------

        self.keyboard_sub = self.create_subscription(
            Int32MultiArray,
            '/keyboard_command',
            self.keyboard_callback,
            10
        )

        self.get_logger().info('Tele velocity node started')
        self.get_logger().info(
            'Subscribing to /keyboard_command'
        )
        self.get_logger().info(
            'Publishing /left_wheel_speed and /right_wheel_speed'
        )

    # ------------------------------------------------------
    # 4. Convert keyboard state into body velocities
    # ------------------------------------------------------

    def calculate_body_velocity(
        self, speed, up, down, left, right
    ):
        """Calculate linear and angular velocity."""

        max_linear_speed = self.get_parameter(
            'max_linear_speed'
        ).value

        max_angular_speed = self.get_parameter(
            'max_angular_speed'
        ).value

        # Forward/reverse direction
        if up and not down:
            linear_direction = 1
        elif down and not up:
            linear_direction = -1
        else:
            linear_direction = 0

        # Left/right rotation
        if left and not right:
            angular_direction = 1
        elif right and not left:
            angular_direction = -1
        else:
            angular_direction = 0

        # Speed level ranges from 0 to 9
        speed_fraction = speed / 9.0

        linear_velocity = (
            linear_direction
            * speed_fraction
            * max_linear_speed
        )

        angular_velocity = (
            angular_direction
            * speed_fraction
            * max_angular_speed
        )

        return linear_velocity, angular_velocity

    # ------------------------------------------------------
    # 5. Calculate individual wheel rotational velocities
    # ------------------------------------------------------

    def calculate_wheel_velocity(
        self, linear_velocity, angular_velocity
    ):
        """Return left and right wheel speeds in rad/s."""

        wheel_radius = self.get_parameter(
            'wheel_radius'
        ).value

        wheel_separation = self.get_parameter(
            'wheel_separation'
        ).value

        if wheel_radius <= 0.0:
            self.get_logger().error(
                'wheel_radius must be positive'
            )
            return None

        if wheel_separation <= 0.0:
            self.get_logger().error(
                'wheel_separation must be positive'
            )
            return None

        # Differential-drive inverse kinematics
        left_wheel_velocity = (
            linear_velocity
            - (wheel_separation / 2.0) * angular_velocity
        ) / wheel_radius

        right_wheel_velocity = (
            linear_velocity
            + (wheel_separation / 2.0) * angular_velocity
        ) / wheel_radius

        return left_wheel_velocity, right_wheel_velocity

    # ------------------------------------------------------
    # 6. Subscriber callback
    # ------------------------------------------------------

    def keyboard_callback(self, msg):
        """Process keyboard state and publish wheel commands."""

        if len(msg.data) != 5:
            self.get_logger().warning(
                'Expected [speed, up, down, left, right]'
            )
            return

        speed, up, down, left, right = msg.data

        # Validate incoming command
        if not 0 <= speed <= 9:
            self.get_logger().warning(
                'Speed level must be between 0 and 9'
            )
            return

        if any(
            value not in (0, 1)
            for value in (up, down, left, right)
        ):
            self.get_logger().warning(
                'Movement flags must be 0 or 1'
            )
            return

        # Speed level zero means stop
        if speed == 0:
            linear_velocity = 0.0
            angular_velocity = 0.0
        else:
            linear_velocity, angular_velocity = (
                self.calculate_body_velocity(
                    speed, up, down, left, right
                )
            )

        wheel_velocities = self.calculate_wheel_velocity(
            linear_velocity, angular_velocity
        )

        if wheel_velocities is None:
            return

        left_wheel_velocity, right_wheel_velocity = (
            wheel_velocities
        )

        # Publish left wheel velocity
        left_msg = Float64()
        left_msg.data = left_wheel_velocity

        # Publish right wheel velocity
        right_msg = Float64()
        right_msg.data = right_wheel_velocity

        self.left_wheel_pub.publish(left_msg)
        self.right_wheel_pub.publish(right_msg)

        self.get_logger().info(
            f'Body: v={linear_velocity:.3f} m/s, '
            f'omega={angular_velocity:.3f} rad/s | '
            f'Left={left_wheel_velocity:.3f} rad/s, '
            f'Right={right_wheel_velocity:.3f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)

    node = TeleVelNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
