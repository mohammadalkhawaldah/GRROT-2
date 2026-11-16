#!/usr/bin/env python3
"""Measure control in Gazebo; this is not a grasp or human-trajectory replay."""
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.msg import Contacts
from trajectory_msgs.msg import JointTrajectoryPoint

ROOT = Path(__file__).resolve().parents[1]


class SmokeTest(Node):
    def __init__(self):
        super().__init__('cup_drag_smoke_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.positions = {}
        self.clock_samples = []
        self.cup_poses = []
        self.contact_samples = 0
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/scene_poses', self.on_cup, qos_profile_sensor_data)
        self.create_subscription(Contacts, '/cup_contacts', self.on_contact, qos_profile_sensor_data)

    def on_joints(self, msg):
        # gz_ros2_control exposes measured mimic state with a documented suffix.
        self.positions.update((name.removesuffix('_mimic'), position)
                              for name, position in zip(msg.name, msg.position))

    def on_clock(self, msg):
        self.clock_samples.append(msg.clock.sec + msg.clock.nanosec * 1e-9)

    def on_cup(self, msg):
        for t in msg.transforms:
            if t.child_frame_id != 'cup':
                continue
            self.cup_poses.append({'frame': t.header.frame_id, 'child': t.child_frame_id,
                                   'x': t.transform.translation.x, 'y': t.transform.translation.y,
                                   'z': t.transform.translation.z})
        self.cup_poses = self.cup_poses[-100:]

    def on_contact(self, msg):
        self.contact_samples += bool(msg.contacts)

    def until(self, predicate, timeout=60):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError('Timed out waiting for ROS state/action')
            rclpy.spin_once(self, timeout_sec=0.1)

    def future(self, future, timeout=60):
        self.until(future.done, timeout)
        return future.result()

    def action(self, client, goal):
        if not client.wait_for_server(timeout_sec=30):
            raise RuntimeError(f'Action unavailable: {client.action_name}')
        handle = self.future(client.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('Action goal rejected')
        result = self.future(handle.get_result_async(), 90)
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'Action failed with status {result.status}: {result.result}')
        # Allow the state subscription to receive the final position.
        finish = time.monotonic() + 1
        self.until(lambda: time.monotonic() >= finish, 3)
        return str(result.result)

    def run(self, report):
        names = [f'joint_{i}' for i in range(1, 8)]
        self.until(lambda: all(n in self.positions for n in names) and len(self.clock_samples) > 2, 90)
        report['clock_advances'] = self.clock_samples[-1] > self.clock_samples[0]
        list_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        if not list_client.wait_for_service(timeout_sec=30):
            raise RuntimeError('controller_manager is unavailable')
        expected = ['joint_state_broadcaster', 'joint_trajectory_controller', 'gripper_controller']
        # Spawners activate independently. Joint states may arrive before the
        # arm/gripper controllers are ready; wait for the actual stage gate.
        deadline = time.monotonic() + 60
        while True:
            controllers = self.future(list_client.call_async(ListControllers.Request()), 10)
            report['controllers'] = {c.name: c.state for c in controllers.controller}
            report['controllers_active'] = all(report['controllers'].get(n) == 'active' for n in expected)
            if report['controllers_active']:
                break
            if time.monotonic() > deadline:
                raise RuntimeError('Required controllers did not become active')
            poll_after = time.monotonic() + 0.2
            self.until(lambda: time.monotonic() >= poll_after, 1)

        arm = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        start = [self.positions[n] for n in names]
        target = start.copy()
        target[0] += 0.15
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        point = JointTrajectoryPoint(positions=target)
        point.time_from_start.sec = 3
        goal.trajectory.points = [point]
        report['arm_action'] = self.action(arm, goal)
        achieved = [self.positions[n] for n in names]
        report['arm_start_rad'] = start
        report['arm_target_rad'] = target
        report['arm_achieved_rad'] = achieved
        report['arm_max_error_rad'] = max(abs(math.atan2(math.sin(a-b), math.cos(a-b)))
                                           for a, b in zip(achieved, target))
        report['arm_motion_pass'] = report['arm_max_error_rad'] < 0.03

        manifest = json.loads((ROOT / 'outputs/robot_manifest.json').read_text())
        primary = manifest['gripper_primary']
        gripper = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        report['gripper_primary_joint'] = primary
        report['gripper_before_rad'] = {k: v for k, v in self.positions.items() if k not in names}
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [primary]
        point = JointTrajectoryPoint(positions=[0.4])
        point.time_from_start.sec = 3
        goal.trajectory.points = [point]
        report['gripper_close_action'] = self.action(gripper, goal)
        closed = {k: v for k, v in self.positions.items() if k not in names}
        report['gripper_closed_rad'] = closed
        report['gripper_primary_close_pass'] = abs(closed[primary] - 0.4) < 0.03
        right = [k for k in closed if 'right' in k and 'knuckle' in k]
        report['opposite_finger_motion_pass'] = bool(right) and any(
            abs(closed[k] - report['gripper_before_rad'].get(k, closed[k])) > 0.1 for k in right)
        import xml.etree.ElementTree as ET
        robot = ET.parse(ROOT / 'outputs/robot.urdf').getroot()
        mimic_errors = {}
        for control in robot.findall('ros2_control'):
            for joint in control.findall('joint'):
                mimic = joint.find("param[@name='mimic']")
                if mimic is None:
                    continue
                multiplier = float(joint.findtext("param[@name='multiplier']", '1'))
                name = joint.get('name')
                mimic_errors[name] = (abs(closed[name] - multiplier * closed[mimic.text])
                                      if name in closed else None)
        report['mimic_joint_errors_rad'] = mimic_errors
        report['all_finger_feedback_pass'] = len(mimic_errors) == 5 and all(
            v is not None and v < 0.05 for v in mimic_errors.values())
        point.positions = [0.1]
        report['gripper_open_action'] = self.action(gripper, goal)
        report['gripper_reopened_rad'] = {k: v for k, v in self.positions.items() if k not in names}
        report['gripper_primary_open_pass'] = abs(self.positions[primary] - 0.1) < 0.03
        report['cup_pose_samples'] = self.cup_poses[-10:]
        report['cup_contact_message_count'] = self.contact_samples
        report['cup_pose_available'] = bool(self.cup_poses)
        report['cup_table_contact_available'] = self.contact_samples > 0
        report['all_passed'] = all(report[k] for k in ['clock_advances', 'controllers_active',
            'arm_motion_pass', 'gripper_primary_close_pass', 'opposite_finger_motion_pass',
            'all_finger_feedback_pass', 'gripper_primary_open_pass', 'cup_pose_available',
            'cup_table_contact_available'])


def main():
    rclpy.init()
    node = SmokeTest()
    report = {'test': 'headless_arm_gripper_control', 'all_passed': False,
              'cup_grasp_tested': False, 'human_replay_tested': False, 'learning_tested': False}
    try:
        node.run(report)
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        path = ROOT / 'outputs/smoke_test.json'
        path.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if report['all_passed'] else 1)


if __name__ == '__main__':
    main()
