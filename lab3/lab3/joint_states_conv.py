import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from math import pi

class JointStatesConverter(Node):
    def __init__(self):
        super().__init__('joint_states_conv')
        self.subscription = self.create_subscription(JointState, '/dobot_joint_states', self.listener_callback, 10)
        # self.subscription = self.create_subscription(JointState, '/dobot_joint_states_simulator', self.listener_callback, 10)
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        self.joint_names = ['J1 - waist', 'J2 - shoulder', 'J3 - elbow', 'J? - wrist', 'J4 - serwo']
        self.offset = [0, 0, 0, 0]
        
        self.data_ready = False

    def listener_callback(self, msg):
        self.theta_read = [msg.position[i] for i in range(len(msg.position))]
        self.data_ready = True

    def calculate_wrist_angle(self):
        theta1 = self.theta_read[0] + self.offset[0]
        theta2 = self.theta_read[1] + self.offset[1]
        theta3 = self.theta_read[2] + self.offset[2]
        theta5 = self.theta_read[3] + self.offset[3]

        theta3=theta3-theta2
        theta4 = -(theta2 + theta3)

        self.publish_joint_states([theta1, theta2, theta3, theta4, -theta5])

    def publish_joint_states(self, joint_positions):
        joint_states = JointState()
        joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_states.name = self.joint_names
        joint_states.position = joint_positions
        self.publisher.publish(joint_states)

    def main_loop(self):
        while rclpy.ok():
            if self.data_ready:
                self.calculate_wrist_angle()
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