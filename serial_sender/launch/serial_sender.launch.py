from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyUSB0',
        description='Serial port name'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial port baud rate'
    )

    # Create the serial_node
    serial_node = Node(
        package='portable_serial_sender',
        executable='serial_node',
        name='serial_node',
        output='screen',
        parameters=[{
            'port_name': LaunchConfiguration('port_name'),
            'baud_rate': LaunchConfiguration('baud_rate')
        }],
        emulate_tty=True
    )

    # Return the launch description
    return LaunchDescription([
        port_name_arg,
        baud_rate_arg,
        serial_node
    ]) 