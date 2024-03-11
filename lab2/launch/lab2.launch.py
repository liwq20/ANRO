from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction
import launch_ros.descriptions

def generate_launch_description():

    # Declare launch arguments
    turtle2_name_arg = DeclareLaunchArgument(
        'turtle2_name',
        default_value='turtle2'
    )

    turtle3_name_arg = DeclareLaunchArgument(
        'turtle3_name',
        default_value='turtle3'
    )

    turtle2_name = LaunchConfiguration('turtle2_name')
    turtle3_name = LaunchConfiguration('turtle3_name')

    
    # Node definitions
    turtle_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="sim"
    )
    turtle_controller = Node(
        package="lab2",
        executable="turtle_controller",
        name="lab2",
        parameters=[
            {"turtle2_name": turtle2_name},
            {"turtle3_name": turtle3_name}
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add actions to the launch description
    ld.add_action(turtle2_name_arg)
    ld.add_action(turtle3_name_arg)
    #actions = TimerAction(period=5.0, actions= [turtle_node, turtle_controller])
    #ld.add_action(actions)
    ld.add_action(turtle_node)
    ld.add_action(TimerAction(period=5.0, actions=[turtle_controller]))

    return ld