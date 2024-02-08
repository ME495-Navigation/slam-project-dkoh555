from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, Shutdown
from launch.substitutions import Command, LaunchConfiguration, \
                                 TextSubstitution, PathJoinSubstitution, EqualsSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os.path


def generate_launch_description():

    return LaunchDescription([
    
        # Launch file arguments
        DeclareLaunchArgument(name='cmd_src',
                              default_value='circle',
                              description='',
                              choices=['circle', 'teleop', 'none']),
    
        DeclareLaunchArgument(name='robot',
                              default_value='nusim',
                              description='',
                              choices=['nusim', 'localhost', 'none']),

        DeclareLaunchArgument(name='use_rviz',
                              default_value='true',
                              description=''),

        # If selected, launch the circle node
        Node(
            package="nuturtle_control",
            executable="circle",
            name="circle",
            condition=IfCondition(
                EqualsSubstitution(LaunchConfiguration("cmd_src"), "circle")
            ),
        ),
    
        # # Launch the robot state publisher
        # Node(
        #         package="robot_state_publisher",
        #         executable="robot_state_publisher",
        #         name="robot_state_publisher",
        #         parameters=[
        #             {
        #                 "robot_description": ParameterValue(
        #                     Command(
        #                         [
        #                             ExecutableInPackage("xacro", "xacro"),
        #                             " ",
        #                             PathJoinSubstitution(
        #                                 [
        #                                     FindPackageShare("nuturtle_description"),
        #                                     "urdf",
        #                                     "turtlebot3_burger.urdf.xacro",
        #                                 ]
        #                             ),
        #                             TextSubstitution(text=" color:="),
        #                             LaunchConfiguration('color'),
        #                         ]
        #                     )
        #                 ),
        #                 "frame_prefix": ParameterValue(
        #                     PythonExpression(["'", LaunchConfiguration('namespace'), "/'"])
        #                 )
        #             }
        #         ],
        #     ),

        # # Initialize full rviz config variables
        # SetLaunchConfiguration(name='config_file',
        #                        value=['basic_', LaunchConfiguration('color'), '.rviz']),

        # SetLaunchConfiguration(name='fixed_frame',
        #                        value=[LaunchConfiguration('namespace'), '/base_footprint']),
        
        # # Launch rviz with config file
        # Node(
        #         package="rviz2",
        #         executable="rviz2",
        #         name="rviz2",
        #         condition=IfCondition(
        #             EqualsSubstitution(LaunchConfiguration("use_rviz"), "true")
        #         ),
        #         arguments=['-d', [os.path.join(get_package_share_directory('nuturtle_description'), 'config/'), LaunchConfiguration('config_file')], '-f', [LaunchConfiguration('fixed_frame')]],
        #         on_exit=Shutdown()
        #     ),
        
        # # Launch the joint_state_publisher that then publishes default states
        # Node(
        #     package="joint_state_publisher",
        #     executable="joint_state_publisher",
        #     name="joint_state_publisher",
        #     condition=IfCondition(
        #         EqualsSubstitution(LaunchConfiguration("use_jsp"), "true")
        #     ),
        # ),
    ])