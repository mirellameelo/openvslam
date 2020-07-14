from launch import LaunchDescription
from launch_ros.actions import Node

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
            node_executable='run_localization',
            arguments=[
                "-v", "/home/mirellameelo/openvslam/build/orb_vocab/orb_vocab.dbow2", 
                "-c", "/home/mirellameelo/openvslam/ros2/configs/config4.yaml",
                "-p", "/home/mirellameelo/openvslam/ros2/maps/map.msg"
                #"--mapping"
            ]
        ),
        Node(
            package='publisher',
            node_executable='video',
            arguments=["-m", "/home/mirellameelo/openvslam/ros2/maps/house_map/video.mp4"]
        )
    ])
