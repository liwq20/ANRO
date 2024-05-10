#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from turtlesim.action import RotateAbsolute
from turtlesim.srv import Spawn
from functools import partial

class TurtleControllerNode(Node):

    def __init__(self, turtle2_name, turtle3_name):
        super().__init__("turtle_controller")
        self.state = 1
        self.declare_parameter('turtle2_name', 'default_turtle2')
        self.declare_parameter('turtle3_name', 'default_turtle3')
        self.turtle2_name = self.get_parameter('turtle2_name').get_parameter_value().string_value
        self.turtle3_name = self.get_parameter('turtle3_name').get_parameter_value().string_value
        self.get_logger().info("Turtle has been started")
        self.rotate_absolute_client = ActionClient(
            self, RotateAbsolute ,"/turtle1/rotate_absolute")

    # Wywolanie akcji obracania
    def send_goal_rotate(self, theta):
        # Tworzenie zapytania
        goal = RotateAbsolute.Goal()
        goal.theta = theta
        
        # Wysylanie zapytania
        self.get_logger().info("Sending goal")
        self.rotate_absolute_client.send_goal_async(goal). \
            add_done_callback(self.goal_response_callback)

    # Odpowiedz po skonczeniu obracania
    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
    
    # Obracanie zakonczone
    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Rotate absolute done")
        if self.state == 1:
            self.call_spawn_service(self.turtle2_name, x=1.0, y=1.0)
        if self.state == 2:
            self.call_spawn_service(self.turtle3_name, x=2.5, y=2.5)
            self.state == 3
    
    # Wywolanie uslugi spawn
    def call_spawn_service(self, name, x, y):
        self.get_logger().info("Sending spawn")
        client = self.create_client(Spawn, "/spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().info("Waiting for service..")
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_spawn))

    # Odpowidz po wykonaniu uslugi spawn
    def callback_spawn(self, future):
        try:
            response = future.result
            if self.state == 1:
                self.state = 2
                self.send_goal_rotate(-4.71239)
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode("turtle2", "turtle3")
    node.send_goal_rotate(3.14159)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()