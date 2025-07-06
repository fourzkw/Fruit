from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    loop_rate_arg = DeclareLaunchArgument(
        'loop_rate',
        default_value='10.0',
        description='Rate at which the state machine loop runs (Hz)'
    )

    # Create the arm control node
    arm_control_node = Node(
        package='arm_control',
        executable='arm_control_node',
        name='arm_control_node',
        output='screen',
        parameters=[{
            'loop_rate': LaunchConfiguration('loop_rate')
        }],
        emulate_tty=True
    )

    # Return the launch description
    return LaunchDescription([
        loop_rate_arg,
        arm_control_node
    ]) 