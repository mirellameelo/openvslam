from launch import LaunchDescription
from launch_ros.actions import Node


# ok

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='image_transport',
        #     node_executable='republish',
        #     output="screen",
        #     arguments=["raw"],
        #     remappings=[
        #         ('in', '/video/image_raw'),
        #         ('out', '/camera/image_raw'),
        #         ('in_transport', 'raw'),
        #     ]
        # ),        
        Node(
            package='openvslam',
            node_executable='run_slam',
            arguments=[
                "-v", "/home/mirellameelo/openvslam_fodeu/build/orb_vocab/orb_vocab.dbow2", 
                "-c", "/home/mirellameelo/openvslam_fodeu/ros2/configs/config.yaml",
                "--eval-log",
                "--map-db", "/home/mirellameelo/openvslam_fodeu/ros2/maps/map.msg"
            ]
        ),
        Node(
            package='publisher',
            node_executable='video',
            arguments=[
                "-m", "/home/mirellameelo/openvslam_fodeu/ros2/maps/house_map/video.mp4",
            ]
        )
    ])
