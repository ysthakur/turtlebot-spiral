import rclpy
from rclpy.node import Node
from math import *
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose
import time

START_THETA = 2 * pi

FF_THETA = 1


def normalize(angle):
    angle = angle % (2 * pi)
    if angle > pi:
        angle -= 2 * pi
    return angle


def format_pose(pose: Pose):
    return f"Pose(x={pose.x:.3f}, y={pose.y:.3f}, theta={pose.theta:.3f})"


class Controller(Node):

    def __init__(self, v, vangle, size):
        super().__init__("spiral_controller")
        self.publisher = self.create_publisher(Twist, "spiral/cmd_vel", 10)
        self.subscription = self.create_subscription(
            Pose, "/turtlesim/turtle1/pose", self.listener_callback, 10
        )
        self.size = size
        self.v = v
        self.vangle = vangle

        self.actions = [
            self.initial_straight,
            self.turn_after_straight,
            self.actual_spiral,
        ]
        self.action_ind = -1

        self.last_time = time.time()

    def pose_at(self, theta):
        r = self.size * theta
        x = r * cos(theta)
        y = r * sin(theta)
        angle = normalize(theta + pi / 2)
        return Pose(x=x, y=y, theta=angle)

    def twist_between(self, p1, p2):
        return Twist(
            linear=Vector3(
                x=sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2), y=0.0, z=0.0
            ),
            angular=Vector3(x=0.0, y=0.0, z=FF_THETA * (p2.theta - p1.theta)),
        )

    def initial_straight(self, pose):
        # self.publisher.publish(
        #     Twist(
        #         linear=Vector3(x=1.0, y=0.0, z=0.0),
        #         angular=Vector3(x=0.0, y=0.0, z=pi),
        #     )
        # )
        # return True
        goal = self.pose_at(START_THETA)

        if pose.x >= goal.x or abs(pose.x - goal.x) < 0.05:
            return True

        self.get_logger().info(
            f"[straight] Pose: {format_pose(pose)}, goal: {format_pose(goal)}"
        )
        dx = goal.x - pose.x
        max_dx = self.v * self.dt
        actual_dx = min(dx, max_dx)
        self.publisher.publish(
            Twist(
                linear=Vector3(x=actual_dx, y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=0.0),
            )
        )

    def turn_after_straight(self, pose: Pose):
        goal = self.pose_at(START_THETA)
        if pose.theta >= goal.theta or abs(pose.theta - goal.theta) < 0.1:
            self.theta = START_THETA
            return True
        self.get_logger().info(
            f"[turn] Pose: {format_pose(pose)}, goal: {format_pose(goal)}"
        )
        vangle = self.vangle if pose.theta < goal.theta else -self.vangle
        self.publisher.publish(
            Twist(
                linear=Vector3(x=0.0, y=0.0, z=0.0),
                angular=Vector3(
                    x=0.0,
                    y=0.0,
                    z=vangle * self.dt,
                ),
            )
        )

    def actual_spiral(self, pose: Pose):
        # self.get_logger().info(f"{pose}")
        # return

        # r = sqrt(pose.x**2 + pose.y**2)
        # theta = r / self.size
        theta = self.theta
        r = self.size * theta
        max_dtheta = self.v * self.dt * 2 * pi / r
        theta2 = theta + max_dtheta
        goal = self.pose_at(theta2)
        msg = self.twist_between(pose, goal)
        self.publisher.publish(msg)
        self.theta = theta2
        self.get_logger().info(
            f"theta={theta:.4f}, d={max_dtheta:.4f}, dt={self.dt}, r={r:.4f} pose={format_pose(pose)} goal={format_pose(goal)}, msg = {msg}"
        )
        # self.get_logger().info(f"Publishing {msg}")

    def listener_callback(self, pose):
        if self.action_ind == -1:
            self.center = pose
            self.action_ind += 1
            return

        pose.x -= self.center.x
        pose.y -= self.center.y

        # self.get_logger().info(f"[listener start] Pose: {pose}")

        if self.action_ind < len(self.actions):
            now = time.time()
            self.dt = now - self.last_time
            action = self.actions[self.action_ind]
            if action(pose):
                self.action_ind += 1
            self.last_time = now

        # self.get_logger().info(f"[listener end] Pose: {pose}")


def main(args=None):
    rclpy.init(args=args)

    controller = Controller(v=15, vangle=15, size=0.2)

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
