#!/usr/bin/env python3
"""Collision-check a pregrasp target with native MoveIt; never execute it."""
import json
import yaml
from pathlib import Path
import rclpy
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import (AllowedCollisionEntry, CollisionObject, Constraints, JointConstraint,
    OrientationConstraint, PlanningScene, PlanningSceneComponents, PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from rosidl_runtime_py.convert import message_to_ordereddict
from smoke_test import SmokeTest

ROOT=Path(__file__).resolve().parents[1]


class Planner(SmokeTest):
    def service(self, kind, name, request):
        client=self.create_client(kind,name)
        if not client.wait_for_service(timeout_sec=30):
            raise RuntimeError(f'Service unavailable: {name}')
        return self.future(client.call_async(request),60)

    def collision_scene(self):
        request=GetPlanningScene.Request()
        request.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        scene=self.service(GetPlanningScene,'/get_planning_scene',request).scene
        scene.is_diff=True
        acm=scene.allowed_collision_matrix
        for name in ['table','cup','base_link']:
            if name not in acm.entry_names:
                acm.entry_names.append(name)
                for row in acm.entry_values:row.enabled.append(False)
                acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
        a,b=acm.entry_names.index('table'),acm.entry_names.index('base_link')
        acm.entry_values[a].enabled[b]=True
        acm.entry_values[b].enabled[a]=True
        cup=acm.entry_names.index('cup')
        for i in range(len(acm.entry_names)):
            acm.entry_values[cup].enabled[i]=False
            acm.entry_values[i].enabled[cup]=False
        objects=[]
        for name,kind,dimensions,xyz in [
                ('table',SolidPrimitive.BOX,[1.6,1.2,.4],[.4,0,.2]),
                ('cup',SolidPrimitive.CYLINDER,[.1,.025],[.55,0,.45])]:
            obj=CollisionObject(id=name,operation=CollisionObject.ADD)
            obj.header.frame_id='world'
            obj.primitives=[SolidPrimitive(type=kind,dimensions=dimensions)]
            p=Pose();p.position.x,p.position.y,p.position.z=map(float,xyz);p.orientation.w=1.0
            obj.primitive_poses=[p];objects.append(obj)
        scene.world.collision_objects=objects
        request=ApplyPlanningScene.Request(scene=scene)
        if not self.service(ApplyPlanningScene,'/apply_planning_scene',request).success:
            raise RuntimeError('Planning scene update failed')

    def plan(self,xyz):
        request=GetMotionPlan.Request();req=request.motion_plan_request
        req.group_name='manipulator';req.num_planning_attempts=5;req.allowed_planning_time=10.0
        req.max_velocity_scaling_factor=.2;req.max_acceleration_scaling_factor=.2
        req.start_state.joint_state.name=list(self.positions)
        req.start_state.joint_state.position=list(self.positions.values())
        pc=PositionConstraint();pc.header.frame_id='world';pc.link_name='end_effector_link';pc.weight=1.0
        p=Pose();p.position.x,p.position.y,p.position.z=map(float,xyz);p.orientation.w=1.0
        pc.constraint_region.primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE,dimensions=[.003])]
        pc.constraint_region.primitive_poses=[p]
        oc=OrientationConstraint();oc.header.frame_id='world';oc.link_name='end_effector_link'
        oc.orientation=Quaternion(x=1.0,y=0.0,z=0.0,w=0.0)
        oc.absolute_x_axis_tolerance=.03;oc.absolute_y_axis_tolerance=.03;oc.absolute_z_axis_tolerance=.03
        oc.weight=1.0
        req.goal_constraints=[Constraints(position_constraints=[pc],orientation_constraints=[oc])]
        return self.service(GetMotionPlan,'/plan_kinematic_path',request).motion_plan_response

    def plan_pregrasp(self):
        target=yaml.safe_load((ROOT/'config/pregrasp_joint_target.yaml').read_text())['joints']
        request=GetMotionPlan.Request();req=request.motion_plan_request
        req.group_name='manipulator';req.num_planning_attempts=5;req.allowed_planning_time=10.0
        req.max_velocity_scaling_factor=.2;req.max_acceleration_scaling_factor=.2
        req.start_state.joint_state.name=list(self.positions)
        req.start_state.joint_state.position=list(self.positions.values())
        req.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=name,
            position=float(value),tolerance_above=.002,tolerance_below=.002,weight=1.0)
            for name,value in target.items()])]
        return self.service(GetMotionPlan,'/plan_kinematic_path',request).motion_plan_response


def main():
    rclpy.init();node=Planner()
    report={'test':'collision_checked_pregrasp_plan','executed':False,'passed':False}
    try:
        node.until(lambda:all(f'joint_{i}' in node.positions for i in range(1,8)),90)
        node.collision_scene()
        response=node.plan_pregrasp()
        report.update(error_code=response.error_code.val,planning_time_s=response.planning_time,
            trajectory_points=len(response.trajectory.joint_trajectory.points),
            target_world_m=[.55,0.,.75],passed=response.error_code.val==1,
            trajectory=message_to_ordereddict(response.trajectory))
    except Exception as exc:report['error']=f'{type(exc).__name__}: {exc}'
    finally:
        (ROOT/'outputs/approach_plan.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k!='trajectory'},indent=2))
        node.destroy_node();rclpy.shutdown()
    raise SystemExit(0 if report['passed'] else 1)


if __name__=='__main__':main()
