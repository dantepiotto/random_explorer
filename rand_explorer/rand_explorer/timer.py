import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import Empty


class gopoint_timer(Node):

    def __init__(self):
        super().__init__('gopoint_timer')
        self.publisher_ = self.create_publisher(Empty, 'gopoint', 1)
        timer_period = 2  # seconds
        time.sleep(5)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Empty()
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    publisher = gopoint_timer()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
