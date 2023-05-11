from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():

    yaml_path_arg = DeclareLaunchArgument('yaml_path',
                                          description='Path of the yaml file with state publisher node parameters')

    remappings = [("odri_cmd", "/odri/robot_command")]

    otp_smi = Node(package="offline_trajectory_publisher_state_machine_interface",
                   executable="talker",
                   name="talker",
                   output="screen",
                   emulate_tty=True,
                   # prefix=['xterm -e gdb -ex run --args'],
                   parameters=[LaunchConfiguration('yaml_path')],
                   remappings=remappings,
                   namespace='odri')

    ld = LaunchDescription()

    ld.add_action(yaml_path_arg)
    ld.add_action(otp_smi)

    return ld
