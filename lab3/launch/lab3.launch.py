from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node

def generate_launch_description():
    package_path = get_package_share_path('lab3')
    default_model_path = package_path / 'urdf/robot.xacro'
    default_rviz_config_path = package_path / 'rviz/urdf_config.rviz'
    topic_list = [f"/joint_states/position[{i}]" for i in range(5)]


    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'], 
                                    description='Flag to enable joint_state_publisher_gui.')
    
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file.')
    
    rqt_arg = DeclareLaunchArgument(name='rqt_plot', default_value='false', choices=['true', 'false'], 
                                    description='Flag to enable rqt_plot')
    
    fkn_arg = DeclareLaunchArgument(name='fkn_arg', default_value='true', choices=['true', 'false'], 
                                    description='Flag to enable forward_kinematic_node')
    

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
        # parameters=[{'robot_description': open(default_model_path).read()}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    rqt_plot_node = Node(
        package='rqt_plot',
        executable='rqt_plot',
        output='screen',
        arguments=topic_list)

    forward_kin_node = Node(
        package='lab3',
        executable='forward_kin',
        condition=IfCondition(LaunchConfiguration('fkn_arg'))
    )

    joint_states_converter_node = Node(
        package='lab3',
        executable='joint_states_conv',
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        rqt_arg,
        fkn_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        rqt_plot_node,
        forward_kin_node,
        joint_states_converter_node
    ])
