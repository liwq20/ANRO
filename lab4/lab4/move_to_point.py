import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped
from dobot_msgs.action import PointToPoint

class MoveToPoint(Node):

    def __init__(self):
        super().__init__("move_to_point")
        self.dobot_client = ActionClient(self, PointToPoint, '/PTP_action')
        self.subscriber = self.create_subscription(PointStamped, "/clicked_point", self.callback, 10)

    def callback(self, msg):
        goal = PointToPoint.Goal()
        self.get_logger().info(str(msg.point.x/msg.point.y))
        if (85*msg.point.z-37)<5:
            return 
        # goal.target_pose = np.array([100*msg.point.x, 100*msg.point.y, 100*msg.point.z-1.13, 0])
        goal.target_pose = np.array([69*msg.point.x, 69*msg.point.y, 85*msg.point.z-37, 0])
        goal.motion_type = 1
        goal.acceleration_ratio = 0.4
        goal.velocity_ratio = 0.5
        while not self.dobot_client.wait_for_server(1.0):
            self._logger.info('calling dobot')
        self.dobot_future = self.dobot_client.send_goal_async(goal)
        self.dobot_future.add_done_callback(self.dobotResponse_callback)

    def dobotResponse_callback(self, future):
        result = future.result()
        if not result.accepted:
            return
        self.rotateResult_future = result.get_result_async()
        self.rotateResult_future.add_done_callback(self.dobotResult_callback)

    def dobotResult_callback(self, future):
        self.get_logger().info(f'MoveToPoint result: {future.result().result}')


def main(args=None):
    rclpy.init(args=args)
    move_to_point = MoveToPoint()
    rclpy.spin(move_to_point)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
