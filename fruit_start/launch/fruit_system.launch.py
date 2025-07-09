from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directories
    usb_camera_pkg_dir = get_package_share_directory('usb_camera')
    fruit_detector_pkg_dir = get_package_share_directory('fruit_detector')
    arm_moveit_pkg_dir = get_package_share_directory('arm_moveit')
    arm_control_pkg_dir = get_package_share_directory('arm_control')
    serial_sender_pkg_dir = get_package_share_directory('portable_serial_sender')
    
    # Include USB camera launch file
    usb_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(usb_camera_pkg_dir, 'launch', 'usb_camera.launch.py')
        )
    )
    
    # Include fruit detector launch file with appropriate topic remapping
    fruit_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fruit_detector_pkg_dir, 'launch', 'fruit_detector.launch.py')
        ),
        launch_arguments={
            'image_topic': '/image_raw',
            'camera_info_topic': '/camera_info'
        }.items()
    )
    
    # Include MoveIt demo launch
    moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_moveit_pkg_dir, 'launch', 'demo.launch.py')
        )
    )
    
    # Include arm_control launch file with delay
    arm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_control_pkg_dir, 'launch', 'arm_control.launch.py')
        )
    )
    
    # Include serial sender launch file
    serial_sender_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(serial_sender_pkg_dir, 'launch', 'serial_sender.launch.py')
        ),
        launch_arguments={
            'port_name': '/dev/ttyUSB0',
            'baud_rate': '115200'
        }.items()
    )
    
    # Delay arm_control node launch to ensure MoveIt is fully started
    delayed_arm_control = TimerAction(
        period=5.0,  # 5 second delay
        actions=[arm_control_launch]
    )
    
    return LaunchDescription([
        usb_camera_launch,
        fruit_detector_launch,
        moveit_demo_launch,
        delayed_arm_control,
        serial_sender_launch
    ]) 