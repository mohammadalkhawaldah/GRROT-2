"""Native MoveIt planning services, using the exact generated simulation model."""
from pathlib import Path
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[1]


def generate_launch_description():
    config = Path(get_package_share_directory('kinova_gen3_7dof_robotiq_2f_85_moveit_config')) / 'config'
    params = {
        'robot_description': (ROOT / 'outputs/robot.urdf').read_text(),
        'robot_description_semantic': (config / 'gen3.srdf').read_text(),
        'robot_description_kinematics': yaml.safe_load((config / 'kinematics.yaml').read_text()),
        'robot_description_planning': yaml.safe_load((config / 'joint_limits.yaml').read_text()),
        'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
        'ompl': yaml.safe_load((config / 'ompl_planning.yaml').read_text()),
        'allow_trajectory_execution': False, 'use_sim_time': True,
        'publish_robot_description_semantic': True,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': {
            'controller_names': ['joint_trajectory_controller', 'gripper_controller'],
            'joint_trajectory_controller': {'type': 'FollowJointTrajectory',
                'joints': [f'joint_{i}' for i in range(1,8)], 'action_ns': 'follow_joint_trajectory', 'default': True},
            'gripper_controller': {'type': 'FollowJointTrajectory',
                'joints': ['robotiq_85_left_knuckle_joint'], 'action_ns': 'follow_joint_trajectory', 'default': True},
        },
    }
    params['robot_description_kinematics']['manipulator']['kinematics_solver_timeout'] = 0.1
    if os.environ.get('CUP_IK_SOLVER') == 'pick_ik':
        params['robot_description_kinematics']['manipulator'].update({
            'kinematics_solver': 'pick_ik/PickIkPlugin',
            'kinematics_solver_timeout': 0.2, 'mode': 'local',
            'position_threshold': 0.001, 'orientation_threshold': 0.01,
            'minimal_displacement_weight': 0.01, 'cost_threshold': 1.0,
            'memetic_num_threads': 2,
        })
    return LaunchDescription([Node(package='moveit_ros_move_group', executable='move_group',
        output='screen', parameters=[params], arguments=['--ros-args', '--log-level', 'warn'])])
