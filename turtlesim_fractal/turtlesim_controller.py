
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtlesimController(Node):

    def __init__(self):
        super().__init__('turtlesim_controller')

        # Publisher: mozgásparancsok
        self.twist_pub = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10
        )

        # Subscriber: pozícióadatok
        self.pose = None
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose',
            self.pose_callback, 10
        )

        # A négyzet célpontjai
        self.targets = [
            (8.0, 5.54),
            (8.0, 8.0),
            (5.54, 8.0),
            (5.54, 5.54)
        ]

        # Aktuális célpont indexe
        self.target_index = 0
        self.finished = False

        # Szabályozás 0.1 másodpercenként
        self.timer = self.create_timer(0.1, self.move)

    def pose_callback(self, msg):
        self.pose = msg

    def move(self):
        if self.pose is None:
            return

        # Aktuális célpont kiválasztása
        target_x, target_y = self.targets[self.target_index]

        # Távolság számítása
        dx = target_x - self.pose.x
        dy = target_y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)

        msg = Twist()

        # Célpont elérése
        if distance < 0.08:
            self.twist_pub.publish(msg)

            self.get_logger().info(
                f'{self.target_index + 1}. célpont elérve!'
            )

            # Következő célpontra lépés
            self.target_index += 1

            # Ha minden célpontot elértünk
            if self.target_index >= len(self.targets):
                self.get_logger().info('Négyzet kész!')
                self.finished = True
                self.timer.cancel()

            return

        # Célpont irányának számítása
        target_angle = math.atan2(dy, dx)

        # Irányhiba normalizálása
        angle_error = target_angle - self.pose.theta
        angle_error = math.atan2(
            math.sin(angle_error),
            math.cos(angle_error)
        )

        # Arányos forgásszabályozás
        msg.angular.z = max(
            -2.0, min(2.0, 4.0 * angle_error)
        )

        # Arányos haladási szabályozás
        if abs(angle_error) < 0.25:
            msg.linear.x = min(2.0, 1.5 * distance)

        self.twist_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    controller = TurtlesimController()

    while rclpy.ok() and not controller.finished:
        rclpy.spin_once(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
