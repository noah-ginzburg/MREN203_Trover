import rclpy
from rclpy.node import Node
from nav2_msgs.msg import Odometry

class Trover_PubSub(Node):

    def __init__(self):
        super.__init__('odom_publisher')
        self.publisher = self.create_publisher(Odometry, 'odom', 10)

        
def main(args=None):
    rclpy.init(args=args)

    odom_publisher = OdomPublisher()

    rclpy.spin(odom_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()