import rclpy
from rclpy.node import Node

class JointStatesConverter(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.data_ready = False

    def main_loop(self):
        while rclpy.ok():
            if self.data_ready:
                # self.calculate()
                self.data_ready = False
            rclpy.spin_once(self)


def main(args=None):
    rclpy.init(args=args)

    node = JointStatesConverter()

    try:
        node.main_loop()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
