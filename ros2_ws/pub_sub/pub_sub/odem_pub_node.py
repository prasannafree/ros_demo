import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist


class odemPublisher(Node):
    def __init__(self):
        super().__init__('odem_pub_node')
        self.publisher = self.create_publisher(Twist , 'vel_topic' , 25 )
        self.timer =  self.create_timer(1.0 , self.myfun)
        self.linear_vel = 0.5 
        self.angular_vel = 0.0
    

    def myfun(self):
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.angular.z = self.angular_vel

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: Linear={msg.linear.x}, Angular={msg.angular.z}')


def main(args = None ):
    rclpy.init(args=args)
    odem_publisher = odemPublisher()
    try:
        # Keep the node running so it continues to publish
        rclpy.spin(odem_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shutdown
        odem_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


        