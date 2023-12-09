from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    # this node just starts up the webots sim (not an actual ROS2 node)
    webots_start = Node(
        package='3pv_drone',
        # namespace='sim',
        executable='webots_sim_node',
        name='motion_control'
    )

    # this node listens to pose msgs from aruco detector
    # and sends motion commands to drone controller
    motion_control_node = Node(
        package='3pv_drone',
        # namespace='sim',
        executable='crazyflie_motion_control',
        name='motion_control'
    )

    # this node listens to camera images, detects aruco tags,
    # and publishes their detection data
    aruco_node = Node(
        package='3pv_drone',
        # namespace='common',
        executable='aruco_tag_node',
        name='aruco'
    )

    ld = LaunchDescription()
    ld.add_action(webots_start)
    ld.add_action(motion_control_node)
    ld.add_action(aruco_node)
    return ld
