import rclpy
import math
from math import cos, sin, sqrt
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


class InverseKin(Node):

    def __init__(self):
        super().__init__('inversekin')
        self.subscription = self.create_subscription(PointStamped, '/clicked_point', self.listener_callback, 10)
        self.timer = self.create_timer(0.1, self.publish_new_model)
        self.joints = (0.0, 0.0, 0.0, 0.0, 0.0)
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

    def listener_callback(self, msg):
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
        self.get_logger().info(str(msg.point))

        if x**2 + y**2 < 1.2:
            self.get_logger().info('too short distance')
            return
        
        h = 1.38
        l1 = 1.35
        l2 = 1.47
        l3 = 0.74
        z += l3
        z -= h
        x = sqrt(x**2 + y**2) - 0.6
        d = sqrt(x**2 + z**2)

        gamma = math.pi
        alpha = 0.0
        try:
            gamma = math.acos((l1**2 + l2**2 - d**2)/(2*l1*l2))
            alpha = math.asin(l2*sin(gamma)/d)
        except ValueError:
            pass
        
        theta1 = math.atan2(msg.point.y, msg.point.x)
        theta2 = math.atan2(x, z) - alpha
        theta3 = math.pi / 2 - gamma

        self.joints = (theta1, theta2, theta3, - theta3 - theta2, 0.0)
    
    def publish_new_model(self):
        model_state = JointState()
        model_state.position = self.joints
        model_state.name = ['base_first_joint', 'first_second_joint', 'second_third_joint', 'third_fourth_joint', 'fourth_fifth_joint']
        now = self.get_clock().now()
        model_state.header.stamp = now.to_msg()
        model_state.header.frame_id = "base"
        self.publisher_.publish(model_state)


def main(args=None):
    rclpy.init(args=args)

    inverse_kin = InverseKin()

    rclpy.spin(inverse_kin)
    inverse_kin.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()