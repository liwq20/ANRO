from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction
import launch_ros.descriptions

def generate_launch_description():
    pick_node = Node(
        package="lab2",
        executable="pick_and_place",
        name="lab2"
    )
    ld = LaunchDescription()
    ld.add_action(pick_node)
    return ld
