from launch import LaunchDescription
from launch_ros.actions import Node

def static_tf(name, x,y,z, r,p,yaw, parent, child):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', str(x), '--y', str(y), '--z', str(z),
            '--roll', str(r), '--pitch', str(p), '--yaw', str(yaw),
            '--frame-id', parent, '--child-frame-id', child
        ]
    )

def generate_launch_description():
    tf_map_to_odom = static_tf('tf_map_to_odom', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'map', 'odom')
    tf_odom_to_base = static_tf('tf_odom_to_base', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'odom', 'base_link')
    tf_base_to_camera = static_tf('tf_base_to_camera', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'base_link', 'camera')
    return LaunchDescription([
        tf_odom_to_base,
        tf_map_to_odom,
        tf_base_to_camera,
    ])
