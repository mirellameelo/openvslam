from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='image_tools',
        #     node_executable='cam2image',  
        #     arguments=["-t", "camera"]
        # ),
        # Node(
        #     package='image_transport',
        #     node_executable='republish',
        #     arguments=["raw"],
        #     remappings=[
        #         ('in', '/camera'),
        #         ('out', '/camera/image_raw'),
        #     ]
        # ),
        Node(
            package='openvslam',
            node_executable='run_localization',
            arguments=[
                "-v", "/home/mirellameelo/openvslam/build/orb_vocab/orb_vocab.dbow2", 
                "-c", "/home/mirellameelo/openvslam/build/aist_living_lab_1/config.yaml",
                "-p", "/home/mirellameelo/openvslam/ros2/maps/map.msg"
            ]
        )
    ])


        # Node(
        #     package='openvslam',
        #     node_executable='run_localization',
        #     arguments=[
        #         '-v', '/home/mirellameelo/openvslam/build/orb_vocab/orb_vocab.dbow2',
        #          the config file is cagado
        #         '-c', '/home/mirellameelo/openvslam/config.yalm',
        #         '--map-db', '/home/mirellameelo/openvslam/ros2/maps/map.msg'
        #     ]
        # )