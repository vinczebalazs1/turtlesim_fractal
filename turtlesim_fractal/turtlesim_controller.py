
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtlesimController(Node):

    def __init__(self):
        super().__init__('turtlesim_controller')

        # Publisher: mozgásparancsok küldése
        self.twist_pub = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10
        )

        # Subscriber: teknős pozíciójának fogadása
        self.pose = None

        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        # Mozgás időzítése
        self.counter = 0
        self.finished = False
        self.timer = self.create_timer(0.1, self.move)

    def pose_callback(self, msg):
        # Első pozíció kiírása
        if self.pose is None:
            self.get_logger().info(
                f'Teknős pozíciója: x={msg.x:.2f}, y={msg.y:.2f}'
            )

        # Aktuális pozíció eltárolása
        self.pose = msg

    def move(self):
        msg = Twist()

        if self.counter < 20:
            # Előrehaladás
            msg.linear.x = 2.0
        else:
            # Megállás
            self.finished = True
            self.timer.cancel()

        self.twist_pub.publish(msg)
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    controller = TurtlesimController()

    while rclpy.ok() and not controller.finished:
        rclpy.spin_once(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
