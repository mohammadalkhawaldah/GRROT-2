"""Headless Fortress + installed Kinova/Robotiq models, without hardware drivers."""
from pathlib import Path
import sys
import os
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from prepare_robot import prepare


def generate_launch_description():
    urdf = prepare()
    step=float(os.environ.get('CUP_PHYSICS_STEP_S','0.001'))
    if step not in (0.001,0.002,0.004,0.005):
        raise ValueError('CUP_PHYSICS_STEP_S must be 0.001, 0.002, 0.004 or 0.005')
    world=ROOT/'worlds/cup_drag.sdf'
    if step!=0.001:
        tree=ET.parse(world)
        tree.find('world/physics/max_step_size').text=str(step)
        world=ROOT/'outputs/cup_drag.runtime.sdf'
        tree.write(world,encoding='unicode',xml_declaration=True)
    gazebo = ExecuteProcess(cmd=['ign', 'gazebo', '-s', '-r', '-v', '3',
                                  str(world)], output='screen',
                            additional_env={'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': '/opt/ros/humble/lib'})
    publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                     parameters=[{'robot_description': urdf.read_text(), 'use_sim_time': True}],
                     output='screen')
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-world', 'cup_drag', '-file', str(urdf), '-name', 'gen3'],
                 output='screen')
    spawners = [Node(package='controller_manager', executable='spawner',
                     arguments=[name, '--controller-manager-timeout', '90'], output='screen')
                for name in ['joint_state_broadcaster', 'joint_trajectory_controller', 'gripper_controller']]
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/world/cup_drag/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        '/world/cup_drag/model/cup/link/cup_link/sensor/cup_contact/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
    ], remappings=[('/world/cup_drag/dynamic_pose/info', '/scene_poses'),
                   ('/world/cup_drag/model/cup/link/cup_link/sensor/cup_contact/contact', '/cup_contacts')],
        output='screen')
    return LaunchDescription([gazebo, publisher, bridge,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=spawners)), spawn])
