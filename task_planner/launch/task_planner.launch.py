from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true, real time if false'
    )

    # Create the task planner node
    task_planner_node = Node(
        package='task_planner',
        executable='task_planner_node',
        name='task_planner_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        emulate_tty=True
    )

    # Return the launch description
    return LaunchDescription([
        use_sim_time_arg,
        task_planner_node
    ]) 