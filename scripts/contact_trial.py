#!/usr/bin/env python3
"""Measured contact baseline using native MoveIt plans and Gazebo dynamics.

This is a programmed feasibility trial, not a learned skill or expert dataset.
Run in a fresh headless world with planning.launch.py active.
"""
import csv
import json
import math
import os
import subprocess
from pathlib import Path
import time
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import AllowedCollisionEntry, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene, GetPositionFK
from trajectory_msgs.msg import JointTrajectoryPoint
from rosidl_runtime_py.convert import message_to_ordereddict
from rcl_interfaces.srv import GetParameters
from plan_approach import Planner

ROOT=Path(__file__).resolve().parents[1]


class Trial(Planner):
    def __init__(self):
        self.stage='initializing';self.trace=[];self.last_trace_time=-1.0
        self.contact_pairs=set();self.bilateral_events=0;self.last_bilateral_time=-1.0
        self.current_finger_contact=False;self.commands=[]
        super().__init__()

    def on_joints(self,msg):
        super().on_joints(msg)
        t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if t-self.last_trace_time<.05:return
        self.last_trace_time=t
        cup=self.cup_poses[-1] if self.cup_poses else {}
        self.trace.append(dict(time_s=t,stage=self.stage,
            **{f'joint_{i}':self.positions.get(f'joint_{i}') for i in range(1,8)},
            gripper_rad=self.positions.get('robotiq_85_left_knuckle_joint'),
            cup_x_m=cup.get('x'),cup_y_m=cup.get('y'),cup_z_m=cup.get('z')))

    def on_contact(self,msg):
        super().on_contact(msg)
        names=[]
        for c in msg.contacts:
            self.contact_pairs.add(tuple(sorted([c.collision1.name,c.collision2.name])))
            names.extend([c.collision1.name,c.collision2.name])
        self.current_finger_contact=any('finger' in name for name in names)
        if any('left_finger' in n for n in names) and any('right_finger' in n for n in names):
            self.bilateral_events+=1;self.last_bilateral_time=self.get_clock().now().nanoseconds*1e-9

    def execute(self,trajectory,min_seconds=0.0):
        traj=trajectory.joint_trajectory
        if not traj.points:
            raise RuntimeError('Planner returned no joint trajectory points')
        total=traj.points[-1].time_from_start.sec+traj.points[-1].time_from_start.nanosec*1e-9
        if total <= 0:
            error=max(abs(self.positions[n]-v) for n,v in zip(traj.joint_names,traj.points[-1].positions))
            if error<.01:
                return dict(action='Already at planned joint target; no command sent',
                            max_joint_error_rad=error,duration_s=0.0)
            raise RuntimeError('Planner returned zero duration for a nonzero movement')
        scale=max(1.0,min_seconds/total)
        for p in traj.points:
            t=(p.time_from_start.sec+p.time_from_start.nanosec*1e-9)*scale
            p.time_from_start.sec=int(t);p.time_from_start.nanosec=int((t-int(t))*1e9)
            p.velocities=[v/scale for v in p.velocities]
            p.accelerations=[a/scale**2 for a in p.accelerations]
        goal=FollowJointTrajectory.Goal();goal.trajectory=traj
        self.commands.append(dict(stage=self.stage,client_time_s=self.get_clock().now().nanoseconds*1e-9,
                                  trajectory=message_to_ordereddict(traj)))
        client=ActionClient(self,FollowJointTrajectory,'/joint_trajectory_controller/follow_joint_trajectory')
        result=self.action(client,goal)
        errors=[abs(math.atan2(math.sin(self.positions[n]-v),math.cos(self.positions[n]-v)))
                for n,v in zip(traj.joint_names,traj.points[-1].positions)]
        if max(errors)>.04:raise RuntimeError(f'Measured arm tracking error: {max(errors)} rad')
        return dict(action=result,max_joint_error_rad=max(errors),duration_s=total*scale)

    def cartesian(self,xyz):
        req=GetCartesianPath.Request();req.header.frame_id='world'
        req.group_name='manipulator';req.link_name='end_effector_link';req.max_step=.004
        req.jump_threshold=0.0;req.revolute_jump_threshold=.3;req.prismatic_jump_threshold=.01
        req.avoid_collisions=True
        req.max_velocity_scaling_factor=.15;req.max_acceleration_scaling_factor=.15
        req.start_state.joint_state.name=list(self.positions)
        req.start_state.joint_state.position=list(self.positions.values())
        pose=Pose();pose.position.x,pose.position.y,pose.position.z=map(float,xyz)
        pose.orientation=Quaternion(x=1.0,y=0.0,z=0.0,w=0.0)
        req.waypoints=[pose]
        out=self.service(GetCartesianPath,'/compute_cartesian_path',req)
        if out.error_code.val!=1 or out.fraction<.999:
            raise RuntimeError(f'Cartesian path incomplete: fraction={out.fraction}, code={out.error_code.val}')
        return out.solution

    def verify_grasp_pose(self):
        req=GetPositionFK.Request();req.header.frame_id='world';req.fk_link_names=['end_effector_link']
        req.robot_state.joint_state.name=list(self.positions)
        req.robot_state.joint_state.position=list(self.positions.values())
        result=self.service(GetPositionFK,'/compute_fk',req)
        if result.error_code.val!=1:raise RuntimeError('Grasp FK query failed')
        pose=result.pose_stamped[0].pose
        error=math.sqrt((pose.position.x-.55)**2+pose.position.y**2+(pose.position.z-.615)**2)
        angle=2*math.acos(min(1.0,abs(pose.orientation.x)))
        if error>.005 or angle>.03:
            raise RuntimeError(f'Grasp pose error: position={error} m, orientation={angle} rad')
        return dict(position_error_m=error,orientation_error_rad=angle)

    def allow_gripper_cup_contact(self):
        req=GetPlanningScene.Request();req.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        scene=self.service(GetPlanningScene,'/get_planning_scene',req).scene;scene.is_diff=True
        acm=scene.allowed_collision_matrix
        import xml.etree.ElementTree as ET
        names=['cup']+[link.get('name') for link in ET.parse(ROOT/'outputs/robot.urdf').getroot().findall('link')
                       if link.get('name').startswith('robotiq_85_')]
        for name in names:
            if name not in acm.entry_names:
                acm.entry_names.append(name)
                for row in acm.entry_values:row.enabled.append(False)
                acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
        cup=acm.entry_names.index('cup')
        for i,name in enumerate(acm.entry_names):
            if name.startswith('robotiq_85_'):
                acm.entry_values[cup].enabled[i]=True;acm.entry_values[i].enabled[cup]=True
        if not self.service(ApplyPlanningScene,'/apply_planning_scene',ApplyPlanningScene.Request(scene=scene)).success:
            raise RuntimeError('Contact collision allowance failed')

    def gripper(self,position):
        goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=['robotiq_85_left_knuckle_joint']
        point=JointTrajectoryPoint(positions=[position]);point.time_from_start.sec=3
        goal.trajectory.points=[point]
        if position>.1:
            # Object contact should stop closure before the empty-space angle.
            # Only this closure goal gets the contact tolerance; bilateral
            # contact is independently required before any drag command.
            goal.goal_tolerance=[JointTolerance(name='robotiq_85_left_knuckle_joint',position=.08)]
        self.commands.append(dict(stage=self.stage,client_time_s=self.get_clock().now().nanoseconds*1e-9,
                                  trajectory=message_to_ordereddict(goal.trajectory),
                                  goal_tolerance_rad=.08 if position>.1 else .03))
        client=ActionClient(self,FollowJointTrajectory,'/gripper_controller/follow_joint_trajectory')
        result=self.action(client,goal)
        return dict(action=result,target_rad=position,
                    achieved_rad=self.positions['robotiq_85_left_knuckle_joint'])

    def run_trial(self,report):
        self.until(lambda:len(self.cup_poses)>0 and all(f'joint_{i}' in self.positions for i in range(1,8)),90)
        model=self.service(GetParameters,'/move_group/get_parameters',
                           GetParameters.Request(names=['robot_description'])).values[0].string_value
        if model != (ROOT/'outputs/robot.urdf').read_text():
            raise RuntimeError('MoveIt model differs from the generated Gazebo model; restart planner')
        report['planning_model_matches_simulation']=True
        self.collision_scene();report['cup_initial']=self.cup_poses[-1]
        self.stage='initial_open';report['initial_open_action']=self.gripper(.1)
        runtime_step=os.environ.get('CUP_RUNTIME_STEP_S')
        if runtime_step:
            step=float(runtime_step)
            if step not in (.002,.0025,.004,.005):raise ValueError('Unsupported runtime physics step')
            changed=subprocess.run(['ign','service','-s','/world/cup_drag/set_physics',
                '--reqtype','ignition.msgs.Physics','--reptype','ignition.msgs.Boolean',
                '--timeout','3000','--req',f'max_step_size: {step} real_time_factor: 1.0'],
                capture_output=True,text=True,timeout=5)
            if changed.returncode or 'data: true' not in changed.stdout:
                raise RuntimeError(f'Physics step update failed: {changed.stdout} {changed.stderr}')
            report['runtime_physics_step_s']=step
        self.stage='pregrasp';plan=self.plan_pregrasp()
        if plan.error_code.val!=1:raise RuntimeError(f'Pregrasp plan failed: {plan.error_code.val}')
        report['pregrasp']=self.execute(plan.trajectory,8)
        self.stage='approach';report['approach']=self.execute(self.cartesian([.55,0.,.615]),5)
        report['grasp_pose_check']=self.verify_grasp_pose()
        report['cup_before_close']=self.cup_poses[-1]
        self.allow_gripper_cup_contact()
        self.stage='close';report['close_action']=self.gripper(.4)
        report['bilateral_contact_events_before_drag']=self.bilateral_events
        if not self.bilateral_events or self.get_clock().now().nanoseconds*1e-9-self.last_bilateral_time>.2:
            raise RuntimeError('No recent measured bilateral finger contact; refusing drag')
        self.stage='drag';report['drag']=self.execute(self.cartesian([.55,.30,.615]),10)
        report['cup_after_drag']=self.cup_poses[-1]
        self.stage='release';report['open_action']=self.gripper(.1)
        self.stage='retract';report['retract']=self.execute(self.cartesian([.55,.30,.75]),5)
        report['cup_final']=self.cup_poses[-1]
        delta=[report['cup_final'][a]-report['cup_initial'][a] for a in ['x','y','z']]
        report['cup_displacement_m']=delta
        drag_heights=[row['cup_z_m'] for row in self.trace if row['stage']=='drag' and row['cup_z_m'] is not None]
        report['max_cup_vertical_deviation_during_drag_m']=max(
            (abs(z-report['cup_initial']['z']) for z in drag_heights),default=float('inf'))
        report['released_finger_contacts_clear']=not self.current_finger_contact
        report['success']=(abs(delta[1]-.30)<.03 and abs(delta[0])<.03 and abs(delta[2])<.01
            and report['max_cup_vertical_deviation_during_drag_m']<.01
            and report['released_finger_contacts_clear'])


def main():
    rclpy.init();node=Trial()
    report=dict(test='programmed_physics_contact_baseline',success=False,learned=False,
        human_timing_reproduced=False,artificial_attachment=False,robot_commands_not_human_6dof=True,
        physics_step_s=float(os.environ.get('CUP_PHYSICS_STEP_S','0.001')))
    try:node.run_trial(report)
    except Exception as exc:report['error']=f'{type(exc).__name__}: {exc}'
    finally:
        report['bilateral_contact_events']=node.bilateral_events
        report['observed_contact_pairs']=sorted(node.contact_pairs)
        (ROOT/'outputs/contact_commands.json').write_text(json.dumps(node.commands,indent=2)+'\n')
        (ROOT/'outputs/contact_trial.json').write_text(json.dumps(report,indent=2)+'\n')
        if node.trace:
            with (ROOT/'outputs/contact_trace.csv').open('w',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=list(node.trace[0]));writer.writeheader();writer.writerows(node.trace)
        print(json.dumps(report,indent=2));node.destroy_node();rclpy.shutdown()
    raise SystemExit(0 if report['success'] else 1)


if __name__=='__main__':main()
