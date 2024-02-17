import rclpy
from rclpy.node import Node
from math import *
from geometry_msgs.msg import Twist, Vector3

from collections import namedtuple

Pose = namedtuple("Pose", "x y angle")


class Controller(Node):

    def __init__(self, dt, v, size):
        super().__init__("spiral_controller")
        self.publisher_ = self.create_publisher(Twist, "spiral/cmd_vel", 10)
        self.dt = dt
        self.v = v
        self.size = size
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.pose = Pose(0.0, 0.0, 0.0)
        self.theta = 0.0

    def pose(self, theta):
        r = self.size * theta
        x = r * cos(theta)
        y = r * sin(theta)
        angle = theta + pi / 2
        return Pose(x, y, angle)

    def twist_between(self, p1, p2):
        return Twist(
            linear=Vector3(x=sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2) / self.dt, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=(p2.angle - p1.angle) / self.dt),
        )

    def timer_callback(self):
        r = self.size = self.theta
        max_dtheta = self.v * self.dt * 2 * pi / r
        theta2 = self.theta + max_dtheta
        r2 = self.size * theta2
        x2 = r2 * cos(theta2)
        y2 = r2 * sin(theta2)
        angle2 = theta2 + pi / 2
        pose2 = Pose(x2, y2, angle2)
        msg = self.twist_between(self.pose, pose2)
        self.publisher_.publish(msg)
        self.pose = pose2
        self.theta = theta2
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    controller = Controller(dt=0.5, v=1.0, size=0.15)

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
