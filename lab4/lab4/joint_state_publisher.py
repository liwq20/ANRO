import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatesConverter(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        self.subscription = self.create_subscription(JointState, '/dobot_joint_states', self.listener, 10)
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        self.namesJoints = [
            'base_first_joint', 
            'first_second_joint', 
            'second_third_joint', 
            'third_fourth_joint', 
            'fourth_fifth_joint'
        ]
        
        self.calculatePosition = False


    def listener(self, msg):
        self.fiRead = [msg.position[i] for i in range(len(msg.position))]
        self.calculatePosition = True


    def calculateAngle(self):
        self.publishJoints([
            self.fiRead[0],
            self.fiRead[1],
            self.fiRead[2] - self.fiRead[1],
            -self.fiRead[2],
            -self.fiRead[3]
        ])


    def publishJoints(self, positions):
        myJointState = JointState()
        myJointState.name = self.namesJoints
        myJointState.position = positions
        myJointState.header.stamp = self.get_clock().now().to_msg() 
        self.publisher.publish(myJointState)


    def loopFunction(self):
        while rclpy.ok():
            if self.calculatePosition:
                self.calculateAngle()
                self.calculatePosition = False
            rclpy.spin_once(self)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = JointStatesConverter()
        node.loopFunction()
    except Exception:
        node.destroy_node()


    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()