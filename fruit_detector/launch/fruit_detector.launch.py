from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    package_dir = get_package_share_directory('fruit_detector')
    model_path = os.path.join(package_dir, 'model', 'fruit.onnx')
    
    # Launch arguments
    image_topic = LaunchConfiguration('image_topic', default='/image_raw')
    camera_info_topic = LaunchConfiguration('camera_info_topic', default='/camera_info')
    conf_threshold = LaunchConfiguration('conf_threshold', default='0.5')
    iou_threshold = LaunchConfiguration('iou_threshold', default='0.45')
    
    # Declare launch arguments
    declare_image_topic = DeclareLaunchArgument(
        'image_topic',
        default_value='/image_raw',
        description='Topic name for the camera image'
    )
    
    declare_camera_info_topic = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera_info',
        description='Topic name for the camera info'
    )
    
    declare_conf_threshold = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.5',
        description='Confidence threshold for object detection'
    )
    
    declare_iou_threshold = DeclareLaunchArgument(
        'iou_threshold',
        default_value='0.45',
        description='IoU threshold for non-maximum suppression'
    )
    
    # Create node
    fruit_detector_node = Node(
        package='fruit_detector',
        executable='fruit_detector_node',
        name='fruit_detector',
        parameters=[{
            'image_topic': image_topic,
            'camera_info_topic': camera_info_topic,
            'model_path': model_path,
            'conf_threshold': conf_threshold,
            'iou_threshold': iou_threshold
        }],
        output='screen'
    )
    
    return LaunchDescription([
        declare_image_topic,
        declare_camera_info_topic,
        declare_conf_threshold,
        declare_iou_threshold,
        fruit_detector_node
    ]) 