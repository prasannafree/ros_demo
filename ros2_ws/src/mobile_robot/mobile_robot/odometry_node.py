
"""Estimate differential-drive robot position from wheel velocities."""

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry_node')

        # Robot geometry parameters
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation', 0.30)
        self.declare_parameter('update_rate', 20.0)  # Hz

        # Initial robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Latest wheel velocities (rad/s) cached from callbacks
        self._left_vel = 0.0
        self._right_vel = 0.0

        # Time of the previous odometry update
        self.last_time = self.get_clock().now()

        # ------------------------------------------------------------------
        # Subscribers: one per wheel.
        # tele_vel_node publishes Float64 on /left_wheel_speed and
        # /right_wheel_speed (NOT a combined Float64MultiArray).
        # ------------------------------------------------------------------
        self.create_subscription(
            Float64,
            '/left_wheel_speed',
            self._left_wheel_cb,
            10,
        )
        self.create_subscription(
            Float64,
            '/right_wheel_speed',
            self._right_wheel_cb,
            10,
        )

        # Publisher: estimated robot pose and velocity
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Timer-driven integration at a fixed rate
        update_rate = self.get_parameter('update_rate').value
        self.create_timer(1.0 / update_rate, self._update)

        self.get_logger().info(
            'Odometry node started — '
            'subscribing to /left_wheel_speed and /right_wheel_speed'
        )

    # ------------------------------------------------------------------
    # Wheel velocity callbacks — cache the latest values
    # ------------------------------------------------------------------

    def _left_wheel_cb(self, msg: Float64):
        self._left_vel = msg.data

    def _right_wheel_cb(self, msg: Float64):
        self._right_vel = msg.data

    # ------------------------------------------------------------------
    # Timer callback: integrate pose and publish
    # ------------------------------------------------------------------

    def _update(self):
        wheel_radius = self.get_parameter('wheel_radius').value
        wheel_separation = self.get_parameter('wheel_separation').value

        if wheel_radius <= 0.0 or wheel_separation <= 0.0:
            self.get_logger().error(
                'wheel_radius and wheel_separation must be positive'
            )
            return

        # Elapsed time
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0.0:
            return

        # Convert wheel angular velocities (rad/s) to linear velocities (m/s)
        v_left = wheel_radius * self._left_vel
        v_right = wheel_radius * self._right_vel

        # Robot body velocities
        linear_velocity = (v_right + v_left) / 2.0
        angular_velocity = (v_right - v_left) / wheel_separation

        # Integrate using the midpoint heading (2nd-order Runge-Kutta)
        delta_theta = angular_velocity * dt
        midpoint_theta = self.theta + delta_theta / 2.0

        self.x += linear_velocity * math.cos(midpoint_theta) * dt
        self.y += linear_velocity * math.sin(midpoint_theta) * dt
        self.theta += delta_theta

        # Normalise heading to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self._publish_odometry(linear_velocity, angular_velocity)

    # ------------------------------------------------------------------
    # Publish nav_msgs/Odometry
    # ------------------------------------------------------------------

    def _publish_odometry(self, linear_velocity, angular_velocity):
        msg = Odometry()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # Position
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        # Yaw → quaternion
        msg.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(self.theta / 2.0),
            w=math.cos(self.theta / 2.0),
        )

        # Velocity
        msg.twist.twist.linear.x = linear_velocity
        msg.twist.twist.angular.z = angular_velocity

        self.odom_pub.publish(msg)

        self.get_logger().info(
            f'Position: x={self.x:.3f} m, '
            f'y={self.y:.3f} m, '
            f'theta={math.degrees(self.theta):.1f}°  '
            f'| v={linear_velocity:.3f} m/s, ω={angular_velocity:.3f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()

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
