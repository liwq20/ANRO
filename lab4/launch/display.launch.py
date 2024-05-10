from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_path = get_package_share_path('lab4')
    default_model_path = package_path / 'urdf/robot.xacro'
    default_rviz_config_path = package_path / 'rviz/conf.rviz'

    
    model = DeclareLaunchArgument(name='model', default_value=str(default_model_path))
    
    rviz = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path))
    
    rqt = DeclareLaunchArgument(name='rqt_plot', default_value='true', choices=['true', 'false'])
    jsp = DeclareLaunchArgument(name='jsp_arg', default_value='true', choices=['true', 'false'])
    fkn = DeclareLaunchArgument(name='fkn_arg', default_value='true', choices=['true', 'false'])
    mtp = DeclareLaunchArgument(name='mtp_arg', default_value='true', choices=['true', 'false'])

    

    robotStatePublisherNode = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    rviz2Node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )


    joint_state_publisherKin = Node(
        package='lab4',
        executable='joint_state_publisher',
        condition=IfCondition(LaunchConfiguration('jsp_arg'))
    )

    moveToPoint = Node(
        package='lab4',
        executable='move_to_point',
        condition=IfCondition(LaunchConfiguration('mtp_arg'))
    )

    forwardKin = Node(
        package='lab4',
        executable='forward_kin',
        condition=IfCondition(LaunchConfiguration('fkn_arg'))
    )

    return LaunchDescription([
        model,
        rviz,
        rqt,
        jsp,
        fkn,
        mtp,
        robotStatePublisherNode,
        rviz2Node,
        joint_state_publisherKin,
        moveToPoint,
        forwardKin
        ])
