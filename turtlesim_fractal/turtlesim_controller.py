import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, TeleportAbsolute


# A 2. szintű Koch-hópehely 48 szakaszból áll.
FRACTAL_LEVEL = 2
POSITION_TOLERANCE = 0.025


def koch_segment(start, end, level):
    """A Koch-szakasz végpontjait adja vissza (a kezdőpont nélkül)."""
    if level == 0:
        return [end]

    dx = (end[0] - start[0]) / 3.0
    dy = (end[1] - start[1]) / 3.0

    a = (start[0] + dx, start[1] + dy)
    b = (start[0] + 2.0 * dx, start[1] + 2.0 * dy)

    # A középső harmadot 60 fokkal kifelé (jobbra) forgatjuk.
    height = math.sqrt(3.0) / 2.0
    tip = (a[0] + 0.5 * dx + height * dy,
           a[1] - height * dx + 0.5 * dy)

    return (
        koch_segment(start, a, level - 1)
        + koch_segment(a, tip, level - 1)
        + koch_segment(tip, b, level - 1)
        + koch_segment(b, end, level - 1)
    )


def make_snowflake(level):
    """Egy szabályos háromszög három oldalából Koch-hópelyhet épít."""
    side = 5.8
    start = (2.6, 3.1)
    right = (start[0] + side, start[1])
    top = (start[0] + side / 2.0,
           start[1] + side * math.sqrt(3.0) / 2.0)

    points = [start]
    points += koch_segment(start, right, level)
    points += koch_segment(right, top, level)
    points += koch_segment(top, start, level)
    return points


class TurtlesimController(Node):

    def __init__(self):
        super().__init__('turtlesim_controller')

        # ROS kommunikáció: parancs ki, pozíció be.
        self.velocity_pub = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10
        )
        self.pose = None
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10
        )

        # Szolgáltatások: toll, rajztábla törlése, kezdőpozíció.
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.clear_client = self.create_client(Empty, '/clear')
        self.teleport_client = self.create_client(
            TeleportAbsolute, '/turtle1/teleport_absolute'
        )

        self.points = make_snowflake(FRACTAL_LEVEL)
        self.target_index = 1
        self.stage = 'wait_services'
        self.future = None
        self.finished = False

        # 20 Hz-es vezérlési ciklus.
        self.timer = self.create_timer(0.05, self.update)

    def pose_callback(self, msg):
        self.pose = msg

    def update(self):
        if self.stage != 'drawing':
            self.prepare_drawing()
        else:
            self.move_to_next_point()

    def prepare_drawing(self):
        """Toll fel -> törlés -> teleport -> toll le, aszinkron ROS service-ekkel."""
        if self.stage == 'wait_services':
            if not (self.pen_client.service_is_ready()
                    and self.clear_client.service_is_ready()
                    and self.teleport_client.service_is_ready()):
                return

            request = SetPen.Request()
            request.off = 1
            self.future = self.pen_client.call_async(request)
            self.stage = 'pen_off'
            return

        if not self.future.done():
            return

        try:
            self.future.result()
        except Exception as exc:
            self.get_logger().error(f'Service hiba: {exc}')
            self.stop()
            return

        if self.stage == 'pen_off':
            self.future = self.clear_client.call_async(Empty.Request())
            self.stage = 'clearing'

        elif self.stage == 'clearing':
            request = TeleportAbsolute.Request()
            request.x = float(self.points[0][0])
            request.y = float(self.points[0][1])
            request.theta = 0.0  # Az első szakasz jobbra tart.
            self.future = self.teleport_client.call_async(request)
            self.stage = 'teleporting'

        elif self.stage == 'teleporting':
            request = SetPen.Request()
            request.r = 255
            request.g = 230
            request.b = 70
            request.width = 3
            request.off = 0
            self.future = self.pen_client.call_async(request)
            self.stage = 'pen_on'

        elif self.stage == 'pen_on':
            self.pose = None  # Friss pozíciót várunk a teleportálás után.
            self.stage = 'drawing'
            self.get_logger().info(
                f'Koch-hópehely: {len(self.points) - 1} szakasz rajzolása.'
            )

    def move_to_next_point(self):
        """P-szabályozó a következő célponthoz."""
        if self.pose is None:
            return

        target_x, target_y = self.points[self.target_index]
        dx = target_x - self.pose.x
        dy = target_y - self.pose.y
        distance = math.hypot(dx, dy)

        if distance < POSITION_TOLERANCE:
            self.velocity_pub.publish(Twist())
            self.target_index += 1

            if self.target_index >= len(self.points):
                self.get_logger().info('Koch-hópehely elkészült!')
                self.stop()
            return

        target_angle = math.atan2(dy, dx)
        angle_error = math.atan2(
            math.sin(target_angle - self.pose.theta),
            math.cos(target_angle - self.pose.theta)
        )

        command = Twist()
        # P-szabályozó: szögsebesség arányos az irányhibával.
        command.angular.z = max(-3.0, min(3.0, 4.0 * angle_error))

        # Előbb a helyes irányba fordulunk, csak utána haladunk.
        if abs(angle_error) < 0.12:
            command.linear.x = min(2.5, 2.7 * distance)

        self.velocity_pub.publish(command)

    def stop(self):
        self.velocity_pub.publish(Twist())
        self.finished = True
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    controller = TurtlesimController()

    try:
        while rclpy.ok() and not controller.finished:
            rclpy.spin_once(controller, timeout_sec=0.1)
    except KeyboardInterrupt:
        controller.velocity_pub.publish(Twist())
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
