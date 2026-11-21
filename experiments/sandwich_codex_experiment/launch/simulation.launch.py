from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT.parents[1]/'scripts'))
sys.path.insert(0,str(ROOT/'scripts'))

def generate_launch_description():
    urdf=ROOT/'outputs/robot.urdf'
    gazebo=ExecuteProcess(cmd=['ign','gazebo','-s','-r','-v','2',str(ROOT/'worlds/sandwich.sdf')],
        additional_env={'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':'/opt/ros/humble/lib'},output='screen')
    pub=Node(package='robot_state_publisher',executable='robot_state_publisher',
        parameters=[{'robot_description':urdf.read_text(),'use_sim_time':True}])
    spawn=Node(package='ros_gz_sim',executable='create',arguments=['-world','sandwich','-file',str(urdf),'-name','gen3'])
    spawners=[Node(package='controller_manager',executable='spawner',arguments=[n,'--controller-manager-timeout','90'])
        for n in ['joint_state_broadcaster','joint_trajectory_controller','gripper_controller']]
    args=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/world/sandwich/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V']
    remaps=[('/world/sandwich/dynamic_pose/info','/scene_poses')]
    for n in ['bread_bottom','tomato','cheese','bread_top','table','source_plate','destination_plate','tomato_rest','cheese_rest']:
        topic=f'/world/sandwich/model/{n}/link/body/sensor/contacts/contact'
        args.append(topic+'@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts');remaps.append((topic,'/contacts/'+n))
    bridge=Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=args,remappings=remaps)
    return LaunchDescription([gazebo,pub,bridge,RegisterEventHandler(OnProcessExit(target_action=spawn,on_exit=spawners)),spawn])
