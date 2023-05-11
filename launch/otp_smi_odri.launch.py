import sys

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # BRINGUP CONFIGURATION
    platform_name = 'solo12'
    robot_name = platform_name

    yaml_path = PathJoinSubstitution([FindPackageShare('offline_trajectory_publisher_state_machine_interface'), 'config', 'bringup.yaml'])

    bringup_args = {}

    ld = LaunchDescription()

    # OFFLINE TRAJECTORY PUBLISHER
    bringup_args['otp'] = {'yaml_path': yaml_path}
    otp_path = PathJoinSubstitution([FindPackageShare('offline_trajectory_publisher_state_machine_interface'), 'launch', '_offline_trajectory_publisher_state_machine_interface.launch.py'])
    otp_src = PythonLaunchDescriptionSource([otp_path])
    otp_launch = IncludeLaunchDescription(otp_src, launch_arguments=bringup_args['otp'].items())
    ld.add_action(otp_launch)

    # ODRI
    bringup_args['odri'] = {'yaml_path': yaml_path}
    odri_path = PathJoinSubstitution([FindPackageShare('odri_ros2_hardware'), 'launch', '_robot_interface.launch.py'])
    odri_src = PythonLaunchDescriptionSource([odri_path])
    odri_launch = IncludeLaunchDescription(odri_src, launch_arguments=bringup_args['odri'].items())
    ld.add_action(odri_launch)

    return ld


if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())