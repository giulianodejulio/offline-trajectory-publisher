from launch_ros.actions import Node

import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():

    publish_period_arg = launch.actions.DeclareLaunchArgument(
        'publish_period',
        default_value="50",
        description="publishing period of the offline trajectory publisher"
    )

    xs_dot_txt_arg = launch.actions.DeclareLaunchArgument(
        'xs_dot_txt',
        default_value='/home/hidro/solo12_ws/src/from_nparray_to_MatrixXd/src/xs.txt',
        description="OCP state trajectory"
    )

    us_dot_txt_arg = launch.actions.DeclareLaunchArgument(
        'us_dot_txt',
        default_value='/home/hidro/solo12_ws/src/from_nparray_to_MatrixXd/src/us.txt',
        description="OCP control trajectory"
    )

    kp_arg = launch.actions.DeclareLaunchArgument(
        'kp',
        default_value="1.",
        description="kp value"
    )

    kd_arg = launch.actions.DeclareLaunchArgument(
        'kd',
        default_value="1.",
        description="kd value"
    )

    i_sat_arg = launch.actions.DeclareLaunchArgument(
        'i_sat',
        default_value="1.",
        description="i_sat value"
    )

    remappings = [("odri_cmd", "/odri/RobotCommand")]

    otp_smi = Node(package="offline_trajectory_publisher_state_machine_interface",
                    executable="talker",
                    name="talker",
                    output="screen",
                    emulate_tty=True,
                    # prefix=['xterm -e gdb -ex run --args'],
                    parameters=[{
                        "publish_period": LaunchConfiguration('publish_period')
                    }, {
                        "xs_dot_txt": LaunchConfiguration('xs_dot_txt')
                    }, {
                        "us_dot_txt": LaunchConfiguration('us_dot_txt')
                    }, {
                        "kp": LaunchConfiguration('kp')
                    }, {
                        "kd": LaunchConfiguration('kd')
                    }, {
                        "i_sat": LaunchConfiguration('i_sat')
                    }],
                    remappings=remappings)

    ld = LaunchDescription()

    ld.add_action(publish_period_arg)
    ld.add_action(xs_dot_txt_arg)
    ld.add_action(us_dot_txt_arg)
    ld.add_action(kp_arg)
    ld.add_action(kd_arg)
    ld.add_action(i_sat_arg)
    ld.add_action(otp_smi)

    return ld
