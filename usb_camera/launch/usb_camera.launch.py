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
            'image_size': [640, 480],
            'camera_info_url': camera_info_url,
            'publish_rate': 30.0,
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
        
        # Nodes to launch
        v4l2_camera_node,
        camera_params_node,
    ]) 