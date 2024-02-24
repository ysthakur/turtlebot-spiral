import rclpy
from rclpy.node import Node
from math import *
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose

START_THETA = 2 * pi

FF_THETA = 1

class Controller(Node):

    def __init__(self, v, size):
        super().__init__("spiral_controller")
        self.publisher = self.create_publisher(Twist, "spiral/cmd_vel", 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtlesim/turtle1/pose',
            self.listener_callback,
            10)
        self.size = size
        self.v = v
        self.state = "start"

    def pose_at(self, theta):
        r = self.size * theta
        x = r * cos(theta)
        y = r * sin(theta)
        angle = theta + pi / 2
        return Pose(x=x, y=y, theta=angle)

    def twist_between(self, p1, p2):
        return Twist(
            linear=Vector3(x=sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2), y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=FF_THETA * (p2.theta - p1.theta)),
        )

    def turn(self, angle):
        self.publisher.publish(Twist(
            linear=Vector3(x=0.0, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=angle),
        ))

    def listener_callback(self, pose):
        if self.state == "start":
            self.center = pose
            self.turn(-pose.theta)
            self.state = "straight"
            return

        pose.x -= self.center.x
        pose.y -= self.center.y

        if self.state == "straight":
            goal = self.pose_at(START_THETA)
            if pose.x >= goal.x or abs(pose.x - goal.x) < 0.05:
                self.state = "turn"
            else:
                self.get_logger().info(f"{pose} {goal}")
                dx = goal.x - pose.x
                max_dx = self.v
                actual_dx = min(dx, max_dx)
                self.publisher.publish(Twist(
                    linear=Vector3(x=actual_dx, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=0.0),
                ))
                return
        if self.state == "turn":
            goal = self.pose_at(START_THETA)
            self.turn(goal.theta - pose.theta)
            self.theta = START_THETA
            self.state = "spiral"
            return

        # self.get_logger().info(f"{pose}")
        # return

        # r = sqrt(pose.x ** 2 + pose.y ** 2)
        # theta = r / self.size
        # theta = atan2(pose.x, pose.y)
        theta = self.theta
        r = self.size * theta
        max_dtheta = self.v * 2 * pi / r
        theta2 = theta + max_dtheta
        msg = self.twist_between(pose, self.pose_at(theta2))
        self.publisher.publish(msg)
        self.theta = theta2
        self.get_logger().info(f"theta={theta}, max={max_dtheta}, r={r} x={pose.x} y={pose.y}")
        # self.get_logger().info(f"Publishing {msg}")


def main(args=None):
    rclpy.init(args=args)

    controller = Controller(v=0.5, size=0.2)

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
