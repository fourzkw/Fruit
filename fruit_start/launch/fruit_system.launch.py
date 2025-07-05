from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directories
    usb_camera_pkg_dir = get_package_share_directory('usb_camera')
    fruit_detector_pkg_dir = get_package_share_directory('fruit_detector')
    
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
    
    return LaunchDescription([
        usb_camera_launch,
        fruit_detector_launch
    ]) 