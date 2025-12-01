import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray


class PS4Controller:
    def __init__(self):
        # Joysticks
        self.LStickX = 0.0
        self.LStickY = 0.0
        self.RStickX = 0.0
        self.RStickY = 0.0

        # Triggers
        self.L2 = 0
        self.R2 = 0

        # Botones
        self.Square = 0
        self.Cross = 0
        self.Circle = 0
        self.Triangle = 0
        self.L1 = 0
        self.R1 = 0
        self.Share = 0
        self.Options = 0
        self.LStickPress = 0
        self.RStickPress = 0
        self.PS = 0

        # D-pad
        self.UP = 0
        self.DOWN = 0
        self.LEFT = 0
        self.RIGHT = 0

    def update(self, joy_msg):
        # Ejes
        self.LStickX = joy_msg.axes[0]
        self.LStickY = joy_msg.axes[1]
        self.RStickX = joy_msg.axes[3]
        self.RStickY = joy_msg.axes[4]

        # BOTONES
        self.Square = joy_msg.buttons[0]
        self.Cross = joy_msg.buttons[1]
        self.Circle = joy_msg.buttons[2]
        self.Triangle = joy_msg.buttons[3]
        self.L1 = joy_msg.buttons[4]
        self.R1 = joy_msg.buttons[5]
        self.L2 = joy_msg.buttons[6]
        self.R2 = joy_msg.buttons[7]
        self.Share = joy_msg.buttons[8]
        self.Options = joy_msg.buttons[9]
        self.LStickPress = joy_msg.buttons[10]
        self.RStickPress = joy_msg.buttons[11]
        self.PS = joy_msg.buttons[12]

        # D-pad
        self.UP = 1 if joy_msg.axes[7] == 1 else 0
        self.DOWN = 1 if joy_msg.axes[7] == -1 else 0
        self.LEFT = 1 if joy_msg.axes[6] == -1 else 0
        self.RIGHT = 1 if joy_msg.axes[6] == 1 else 0


class TeleopPS4(Node):
    def __init__(self):
        super().__init__('teleop_ps4_node')

        self.pub = self.create_publisher(Float32MultiArray, '/Data_PS4', 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.controller = PS4Controller()
        self.last_msg = Float32MultiArray()

        # TIMER: publica SIEMPRE a 50 Hz (0.02 s)
        self.timer = self.create_timer(0.02, self.publish_fixed_rate)

        self.get_logger().info("Nodo PS4 listo (publicación constante a 50 Hz)")

    def joy_callback(self, msg):
        self.controller.update(msg)

    def publish_fixed_rate(self):
        out = Float32MultiArray()
        out.data = [
            self.controller.LStickX,
            self.controller.LStickY,
            self.controller.RStickX,
            self.controller.RStickY,

            float(self.controller.L2),
            float(self.controller.R2),

            float(self.controller.Square),
            float(self.controller.Cross),
            float(self.controller.Circle),
            float(self.controller.Triangle),
            float(self.controller.L1),
            float(self.controller.R1),
            float(self.controller.Share),
            float(self.controller.Options),
            float(self.controller.LStickPress),
            float(self.controller.RStickPress),
            float(self.controller.PS),

            float(self.controller.UP),
            float(self.controller.DOWN),
            float(self.controller.LEFT),
            float(self.controller.RIGHT),
        ]

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopPS4()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
