import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3


class Controller(Node):

    def __init__(self):
        super().__init__("spiral_controller")
        self.publisher_ = self.create_publisher(Twist, "spiral/cmd_vel", 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.theta = 2.0

    def timer_callback(self):
        msg = Twist(linear=Vector3(x=0.5, y=0.5, z=0.0), angular=Vector3(x=0.0, y=0.0, z=self.theta))
        self.publisher_.publish(msg)
        self.theta *= 0.95
        # self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
