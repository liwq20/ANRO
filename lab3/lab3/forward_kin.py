import rclpy
from rclpy.node import Node
import numpy as np
# from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose
from scipy.spatial.transform import Rotation as R


class ForwardKin(Node):
    def __init__(self):
        super().__init__('forward_kin_node')
        self.subscription = self.create_subscription(JointState, '/joint_states', self.listener_callback, 10)
        self.publisher = self.create_publisher(PoseStamped, '/pose_topic', 10)

        self.data_ready = False
        
        pi_2 = np.pi/2

        self.d =     [1.38, 0, 0, 0, 0.74]
        self.a =     [0,0, 1.35, 1.47, 0.6]
        self.theta = [0,-pi_2, pi_2, 0, 0]
        self.alpha = [0,-pi_2, 0, 0, -pi_2]

    def listener_callback(self, msg):
        self.theta_read = [msg.position[i] for i in range(len(msg.position))]
        self.data_ready = True
        # self.pose_calculate()
        # self.get_logger().info(str(self.theta_read))
        
    def matrix(self, i):
        theta = self.theta[i]+self.theta_read[i]
        alpha = self.alpha[i]
        a = self.a[i]
        d = self.d[i]

        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        cos_alpha = np.cos(alpha)
        sin_alpha = np.sin(alpha)
        
        matrix = np.array([[cos_theta,              -sin_theta,             0,              a           ],
                           [sin_theta*cos_alpha,    cos_theta*cos_alpha,    -sin_alpha,     -d*sin_alpha],
                           [sin_theta*sin_alpha,    cos_theta*sin_alpha,    cos_alpha,      d*cos_alpha ],
                           [0,                      0,                      0,              1           ]])
        
        return matrix

    def pose_calculate(self):
        matrices = [self.matrix(i) for i in range(5)]
        result_matrix = matrices[-1]
        for matrix in reversed(matrices[:-1]):
            result_matrix = np.dot(matrix, result_matrix)
            # self.get_logger().info(str(result_matrix))

        self.publish_pose_stamped(result_matrix)

    def publish_pose_stamped(self, result_matrix):
        position = result_matrix[:3,3]

        orientation = result_matrix[:3, :3]
        quaternion = R.from_matrix(orientation).as_quat()

        pose_msg = Pose()
        pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = position        
        pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w = quaternion
        
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.frame_id = 'base_link'
        pose_stamped_msg.pose = pose_msg

        # self.get_logger().info(str(pose_stamped_msg.pose))
        self.publisher.publish(pose_stamped_msg)

    def main_loop(self):
        while rclpy.ok():
            if self.data_ready:
                self.pose_calculate()
                self.data_ready = False
            rclpy.spin_once(self)


def main(args=None):
    rclpy.init(args=args)

    node = ForwardKin()

    try:
        node.main_loop()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()