from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('usb_camera')
    
    # Launch arguments
    camera_name = LaunchConfiguration('camera_name', default='head_camera')
    camera_device = LaunchConfiguration('camera_device', default='/dev/video0')
    camera_frame_id = LaunchConfiguration('camera_frame_id', default='camera_optical_frame')
    
    # Exposure and image quality settings
    auto_exposure = LaunchConfiguration('auto_exposure', default=3)  # 1=manual, 3=auto
    exposure_time_absolute = LaunchConfiguration('exposure_time_absolute', default=100)
    brightness = LaunchConfiguration('brightness', default=0)
    contrast = LaunchConfiguration('contrast', default=10)
    gain = LaunchConfiguration('gain', default=0)
    white_balance_automatic = LaunchConfiguration('white_balance_automatic', default=True)
    white_balance_temperature = LaunchConfiguration('white_balance_temperature', default=4500)
    
    # Prepare camera info URL
    camera_info_path = os.path.join(pkg_dir, 'config', 'head_camera.yaml')
    camera_info_url = LaunchConfiguration(
        'camera_info_url', 
        default=f'file://{camera_info_path}'
    )
    
    # Define v4l2_camera node
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[{
            'camera_name': camera_name,
            'video_device': camera_device,
            'frame_id': camera_frame_id,
            'pixel_format': 'YUYV',
            'output_encoding': 'rgb8',
            'image_size': [1280, 720],
            'camera_info_url': camera_info_url,
            'publish_rate': 30.0,
            # Exposure settings
            'auto_exposure': 0,
            'exposure_time_absolute': 100,
        }],
        output='screen',
    )
    
    # Define camera parameters publisher node
    camera_params_node = Node(
        package='usb_camera',
        executable='camera_params_publisher',
        name='camera_params_publisher',
        parameters=[{
            'camera_name': camera_name,
            'camera_info_url': camera_info_url,
            'frame_id': camera_frame_id,
            'publish_rate': 10.0,
        }],
        output='screen',
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'camera_name',
            default_value='head_camera',
            description='Camera name'
        ),
        DeclareLaunchArgument(
            'camera_device',
            default_value='/dev/video0',
            description='Camera device'
        ),
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='camera_optical_frame',
            description='Camera frame ID'
        ),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value=f'file://{camera_info_path}',
            description='Camera calibration file URL'
        ),
        # Exposure settings arguments
        DeclareLaunchArgument(
            'auto_exposure',
            default_value='3',
            description='Auto exposure mode (1=manual, 3=auto)'
        ),
        DeclareLaunchArgument(
            'exposure_time_absolute',
            default_value='100',
            description='Absolute exposure time (when in manual mode)'
        ),
        DeclareLaunchArgument(
            'brightness',
            default_value='0',
            description='Image brightness (-64 to 64)'
        ),
        DeclareLaunchArgument(
            'contrast',
            default_value='10',
            description='Image contrast (0 to 64)'
        ),
        DeclareLaunchArgument(
            'gain',
            default_value='0',
            description='Image gain (0 to 100)'
        ),
        DeclareLaunchArgument(
            'white_balance_automatic',
            default_value='true',
            description='Auto white balance'
        ),
        DeclareLaunchArgument(
            'white_balance_temperature',
            default_value='4500',
            description='White balance temperature (when auto white balance is off)'
        ),
        
        # Nodes to launch
        v4l2_camera_node,
        camera_params_node,
    ]) 