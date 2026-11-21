"""Gated physical transfer, using baseline MoveIt/JTC helpers read-only."""
import csv
import json
import math
import subprocess
import sys
import time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AllowedCollisionEntry, CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPositionFK, GetCartesianPath, GetPositionIK, GetStateValidity
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.srv import GetParameters
from ros_gz_interfaces.msg import Contacts
from shape_msgs.msg import SolidPrimitive
from rosidl_runtime_py.convert import message_to_ordereddict
from action_msgs.msg import GoalStatus
from scene import ROOT,BASE,FOOD,SUPPORTS
sys.path.insert(0,str(BASE/'scripts'))
from contact_trial import Trial
import plan_approach
plan_approach.ROOT=ROOT

class Sandwich(Trial):
    def __init__(self,out):
        self.out=out;self.items={};self.pose_rows=[];self.contact_rows=[]
        self.bilateral={};self.fingers={};self.unwanted=[];self.last_pose_t=-1
        self.active_item=None;self.held=False;self.feedback=[];self.action_results=[];self.contact_logged={};self.envelope_checks=[];self.removed=set();self.rates=[];self.last_rate=-1
        super().__init__()
        for n in [*FOOD,*SUPPORTS]:
            self.create_subscription(Contacts,'/contacts/'+n,lambda m,n=n:self.contacts(n,m),qos_profile_sensor_data)

    def now(self):return self.get_clock().now().nanoseconds*1e-9

    def on_clock(self,msg):
        super().on_clock(msg)
        t=msg.clock.sec+msg.clock.nanosec*1e-9
        if t-self.last_rate>.2:
            self.last_rate=t;self.rates.append(dict(time_s=t,wall_monotonic_s=time.monotonic(),stage=self.stage))

    def service(self,kind,name,req):
        out=super().service(kind,name,req)
        if kind is GetCartesianPath and out.fraction<.999:
            diag=dict(stage=self.stage,fraction=out.fraction,response=message_to_ordereddict(out))
            ik=GetPositionIK.Request();ik.ik_request.group_name='manipulator'
            ik.ik_request.ik_link_name='end_effector_link';ik.ik_request.pose_stamped.header.frame_id='world'
            ik.ik_request.pose_stamped.pose=req.waypoints[-1];ik.ik_request.avoid_collisions=False
            ik.ik_request.robot_state=req.start_state
            if out.solution.joint_trajectory.points:
                ik.ik_request.robot_state.joint_state.name=list(out.solution.joint_trajectory.joint_names)
                ik.ik_request.robot_state.joint_state.position=list(out.solution.joint_trajectory.points[-1].positions)
            ik.ik_request.timeout.sec=1
            found=super().service(GetPositionIK,'/compute_ik',ik);diag['diagnostic_ik']=message_to_ordereddict(found)
            if found.error_code.val==1:
                vr=GetStateValidity.Request();vr.robot_state=found.solution;vr.group_name='manipulator'
                diag['diagnostic_validity']=message_to_ordereddict(super().service(GetStateValidity,'/check_state_validity',vr))
            (self.out/'path_failure.json').write_text(json.dumps(diag,indent=2)+'\n')
        return out

    def on_joints(self,msg):
        before=len(self.trace)
        super().on_joints(msg)
        if len(self.trace)>before:
            for key in ['cup_x_m','cup_y_m','cup_z_m']:self.trace[-1].pop(key,None)
            theta=self.positions.get('robotiq_85_left_knuckle_joint')
            if theta is not None:
                # Inner tip-mesh bounding planes from installed vendor geometry;
                # geometric joint-derived aperture, not a tactile measurement.
                self.trace[-1]['tip_mesh_aperture_estimate_m']=2*(.03060114+.03715750*math.cos(theta)-.04342168*math.sin(theta)-.02525865)

    def action(self,client,goal):
        if not client.wait_for_server(timeout_sec=30):raise RuntimeError('Controller action unavailable')
        stage=self.stage
        def feedback(msg):
            f=msg.feedback
            for i,n in enumerate(f.joint_names):
                self.feedback.append(dict(time_s=self.now(),stage=stage,joint=n,
                    source_time_s=f.header.stamp.sec+f.header.stamp.nanosec*1e-9,
                    desired_rad=f.desired.positions[i],actual_rad=f.actual.positions[i],
                    error_rad=f.error.positions[i]))
        handle=self.future(client.send_goal_async(goal,feedback_callback=feedback))
        record=dict(stage=stage,accepted=handle.accepted,accepted_time_s=self.now())
        self.action_results.append(record)
        if not handle.accepted:raise RuntimeError('Controller goal rejected')
        result=self.future(handle.get_result_async(),180)
        record.update(result_time_s=self.now(),status=result.status,error_code=result.result.error_code,
            error_string=result.result.error_string)
        if result.status!=GoalStatus.STATUS_SUCCEEDED:raise RuntimeError(f'Controller action failed: {record}')
        self.wait_sim(.5)
        return str(result.result)

    def on_cup(self,msg):
        t=self.now()
        for tr in msg.transforms:
            if tr.child_frame_id not in FOOD:continue
            p=tr.transform.translation;q=tr.transform.rotation
            self.items[tr.child_frame_id]=dict(x=p.x,y=p.y,z=p.z,qx=q.x,qy=q.y,qz=q.z,qw=q.w)
        if t-self.last_pose_t>=.05:
            self.last_pose_t=t
            for n,p in self.items.items():self.pose_rows.append(dict(time_s=t,stage=self.stage,item=n,**p))

    def contacts(self,n,msg):
        t=self.now();names=[]
        for c in msg.contacts:
            a,b=c.collision1.name,c.collision2.name;names.extend([a,b])
            self.contact_pairs.add(tuple(sorted([a,b])))
            key=(n,a,b,self.stage)
            if t-self.contact_logged.get(key,-100)>=.05:
                self.contact_logged[key]=t
                self.contact_rows.append(dict(time_s=t,stage=self.stage,sensor=n,collision1=a,collision2=b,
                    contact_json=json.dumps(message_to_ordereddict(c),separators=(',',':'))))
            # Table/base mounting is intentional. Any other robot/support contact fails.
            if n in SUPPORTS and any('gen3' in v and 'base_link' not in v for v in [a,b]):
                self.unwanted.append(dict(time_s=t,stage=self.stage,pair=[a,b]))
        left=any('left_finger_tip' in x for x in names)
        right=any('right_finger_tip' in x for x in names)
        if n in FOOD:
            self.fingers[n]=left or right
            if left and right:self.bilateral[n]=t

    def wait_sim(self,seconds):
        start=self.now();self.until(lambda:self.now()-start>=seconds,seconds*8+10)

    def scene(self,allow=None):
        req=GetPlanningScene.Request();req.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        sc=self.service(GetPlanningScene,'/get_planning_scene',req).scene;sc.is_diff=True
        acm=sc.allowed_collision_matrix
        for n in [*FOOD,*SUPPORTS,'base_link']:
            if n not in acm.entry_names:
                acm.entry_names.append(n)
                for row in acm.entry_values:row.enabled.append(False)
                acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
        for n in FOOD:
            i=acm.entry_names.index(n)
            for j,k in enumerate(acm.entry_names):
                if k.startswith('robotiq_85_'):
                    acm.entry_values[i].enabled[j]=acm.entry_values[j].enabled[i]=(n==allow)
        i,j=acm.entry_names.index('table'),acm.entry_names.index('base_link')
        acm.entry_values[i].enabled[j]=acm.entry_values[j].enabled[i]=True
        objects=[]
        entries=[(n,size,xyz) for n,(size,xyz) in SUPPORTS.items()]
        entries += [(n,d['size'],[self.items[n][a] for a in ['x','y','z']]) for n,d in FOOD.items()]
        for n,size,xyz in entries:
            if self.held and n==self.active_item:
                # A stale world-fixed copy becomes a false obstacle on descent.
                # Remove only that planning proxy. No attached object is used;
                # Gazebo retains its original dynamic body and all collisions.
                if n not in self.removed:
                    objects.append(CollisionObject(id=n,operation=CollisionObject.REMOVE));self.removed.add(n)
                continue
            self.removed.discard(n)
            obj=CollisionObject(id=n,operation=CollisionObject.ADD);obj.header.frame_id='world'
            obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=list(map(float,size)))]
            p=Pose();p.position.x,p.position.y,p.position.z=map(float,xyz);p.orientation.w=1.
            if n in FOOD:
                for a in 'xyzw':setattr(p.orientation,a,self.items[n]['q'+a])
            obj.primitive_poses=[p];objects.append(obj)
        sc.world.collision_objects=objects
        if not self.service(ApplyPlanningScene,'/apply_planning_scene',ApplyPlanningScene.Request(scene=sc)).success:
            raise RuntimeError('Planning scene update failed')

    def move(self,xyz,seconds=4):
        print(self.stage,xyz,flush=True)
        if self.held:self.check_payload_envelope(xyz)
        self.scene(self.active_item)
        result=self.execute(self.cartesian(xyz),seconds)
        req=GetPositionFK.Request();req.header.frame_id='world';req.fk_link_names=['end_effector_link']
        req.robot_state.joint_state.name=list(self.positions);req.robot_state.joint_state.position=list(self.positions.values())
        fk=self.service(GetPositionFK,'/compute_fk',req)
        if fk.error_code.val!=1:raise RuntimeError('FK failed')
        p=fk.pose_stamped[0].pose
        err=math.dist([p.position.x,p.position.y,p.position.z],xyz)
        angle=2*math.acos(min(1.,abs(p.orientation.x)))
        result.update(target_xyz_m=xyz,fk_position_error_m=err,fk_orientation_error_rad=angle)
        if err>.006 or angle>.04:raise RuntimeError(f'Tool pose error {err} m {angle} rad')
        if self.unwanted:raise RuntimeError('Measured robot/table or plate collision')
        return result

    def check_payload_envelope(self,target):
        # Fixed tool orientation makes each Cartesian segment a straight payload
        # translation. Check a conservative rigid-box envelope independently of
        # MoveIt's robot collision test; this never moves any simulator body.
        import numpy as np
        from scipy.spatial.transform import Rotation
        req=GetPositionFK.Request();req.header.frame_id='world';req.fk_link_names=['end_effector_link']
        req.robot_state.joint_state.name=list(self.positions);req.robot_state.joint_state.position=list(self.positions.values())
        fk=self.service(GetPositionFK,'/compute_fk',req)
        if fk.error_code.val!=1:raise RuntimeError('Payload envelope FK unavailable')
        p=fk.pose_stamped[0].pose.position;start=np.array([p.x,p.y,p.z]);end=np.array(target)
        n=self.active_item;item=self.items[n];center=np.array([item[a] for a in ['x','y','z']])
        def half_size(name):
            q=[self.items[name]['q'+a] for a in 'xyzw']
            return np.abs(Rotation.from_quat(q).as_matrix())@np.array(FOOD[name]['size'])/2+.0005
        half=half_size(n)
        obstacles=[(name,np.array(xyz),np.array(size)/2) for name,(size,xyz) in SUPPORTS.items()]
        obstacles += [(name,np.array([self.items[name][a] for a in ['x','y','z']]),half_size(name)) for name in FOOD if name!=n]
        count=max(2,int(np.linalg.norm(end-start)/.004)+1)
        for s in np.linspace(0,1,count):
            c=center+s*(end-start)
            for name,oc,oh in obstacles:
                overlap=half+oh-np.abs(c-oc)
                if np.all(overlap>0) and np.min(overlap)>.002:
                    raise RuntimeError(f'Payload swept envelope intersects {name}: {overlap.tolist()}')
        self.envelope_checks.append(dict(stage=self.stage,samples=count,spacing_m=.004,
            rotation_fixed=True,box_margin_m=.0005,touch_tolerance_m=.002,passed=True))

    def transfer(self,n,height):
        report={'item':n,'passed':False};self.report['transfers'].append(report)
        self.active_item=None;self.stage=n+':pregrasp';self.scene()
        self.stage=n+':open';report['open']=self.gripper(.1)
        src=self.items[n].copy();x,y,z=src['x'],src['y'],src['z']
        report['initial']=src
        self.stage=n+':above_source';report['above_source']=self.move([x,y,.76],5)
        self.active_item=n
        self.stage=n+':approach';report['approach']=self.move([x,y,z-FOOD[n]['size'][2]/2+(.161 if n.startswith('bread') else .154)],5)
        self.stage=n+':close';report['close']=self.gripper(.4)
        if self.now()-self.bilateral.get(n,-100)>.2:raise RuntimeError('No recent bilateral FINGERTIP contact on '+n)
        report['bilateral_before_lift']=True
        self.held=True
        self.stage=n+':lift';report['lift']=self.move([x,y,.76],5)
        lifted=self.items[n].copy();report['lifted']=lifted
        if lifted['z']-src['z']<.07:raise RuntimeError('Item did not lift at least 7 cm')
        if self.now()-self.bilateral.get(n,-100)>.2:raise RuntimeError('Bilateral contact lost during lift')
        cues=json.loads((ROOT/'config/human_task_cues.json').read_text())['ingredients'][n]
        report['human_task_cues']=cues
        self.stage=n+':transport';report['transport']=self.move([.55,.30,.76],cues['programmed_min_transport_s'])
        moved=self.items[n].copy();report['after_transport']=moved
        if math.hypot(moved['x']-.55,moved['y']-.30)>.015:raise RuntimeError('Item slipped during transport')
        if self.now()-self.bilateral.get(n,-100)>.2:raise RuntimeError('Bilateral contact lost during transport')
        # Measured grasp offset + 5mm release gap avoids pressing the
        # position-controlled carried body into a freely sliding layer.
        import numpy as np
        from scipy.spatial.transform import Rotation
        req=GetPositionFK.Request();req.header.frame_id='world';req.fk_link_names=['end_effector_link']
        req.robot_state.joint_state.name=list(self.positions);req.robot_state.joint_state.position=list(self.positions.values())
        fk=self.service(GetPositionFK,'/compute_fk',req)
        if fk.error_code.val!=1:raise RuntimeError('Placement FK unavailable')
        p=fk.pose_stamped[0].pose.position;tool=np.array([p.x,p.y,p.z])
        obj=self.items[n];center=np.array([obj[a] for a in ['x','y','z']]);offset=center-tool
        half_z=float((np.abs(Rotation.from_quat([obj['q'+a] for a in 'xyzw']).as_matrix())@np.array(FOOD[n]['size'])/2)[2])
        release_gap=.005 if n.startswith('bread') else .009
        desired=np.array([.55,.30,height-FOOD[n]['size'][2]/2+half_z+release_gap])
        place=(desired-offset).tolist()
        report['placement_strategy']=dict(measured_grasp_offset_m=offset.tolist(),release_gap_m=release_gap,
            desired_object_center_before_release_m=desired.tolist(),not_an_object_pose_command=True)
        self.stage=n+':place';report['place']=self.move(place,5)
        self.stage=n+':release';report['release']=self.gripper(.1)
        self.held=False
        self.stage=n+':retract';report['retract']=self.move([.55,.30,.76],5)
        self.stage=n+':settle';self.wait_sim(3)
        final=self.items[n].copy();report['final']=final
        settled=[p for p in self.pose_rows if p['item']==n and p['stage']==self.stage]
        report['settle_position_range_m']={a:max(p[a] for p in settled)-min(p[a] for p in settled) for a in ['x','y','z']}
        report['released']=not self.fingers.get(n,True)
        report['xy_error_m']=math.hypot(final['x']-.55,final['y']-.30)
        report['z_error_m']=abs(final['z']-height)
        report['tilt_rad']=2*math.acos(min(1.,math.sqrt(final['qz']**2+final['qw']**2)))
        report['passed']=report['released'] and report['xy_error_m']<.015 and report['z_error_m']<.008 and report['tilt_rad']<.15 and max(report['settle_position_range_m'].values())<.003 and not self.unwanted
        if not report['passed']:raise RuntimeError('Placement/release acceptance failed')
        self.active_item=None

    def run(self,full=False,top_only=False):
        if full and top_only:raise ValueError('Select full or isolated top debug')
        self.until(lambda:len(self.items)==4 and len(self.positions)>=7,100)
        self.until(lambda:all(c.state=='active' for c in self.service(ListControllers,'/controller_manager/list_controllers',ListControllers.Request()).controller) and len(self.service(ListControllers,'/controller_manager/list_controllers',ListControllers.Request()).controller)==3,60)
        model=self.service(GetParameters,'/move_group/get_parameters',GetParameters.Request(names=['robot_description'])).values[0].string_value
        if model!=(ROOT/'outputs/robot.urdf').read_text():raise RuntimeError('Planner/simulator robot mismatch')
        self.report['planning_model_matches_simulation']=True
        self.wait_sim(2);self.scene()
        self.stage='initial_open';self.gripper(.1)
        changed=subprocess.run(['ign','service','-s','/world/sandwich/set_physics',
            '--reqtype','ignition.msgs.Physics','--reptype','ignition.msgs.Boolean',
            '--timeout','3000','--req','max_step_size: 0.004 real_time_factor: 1.0'],
            capture_output=True,text=True,timeout=5)
        if changed.returncode or 'data: true' not in changed.stdout:raise RuntimeError('Runtime physics update failed')
        self.report['runtime_physics_step_s']=.004
        # The validated bounded-joint branch is collision planned afresh.
        self.stage='initial_pregrasp';plan=self.plan_pregrasp()
        if plan.error_code.val!=1:raise RuntimeError(f'Native OMPL pregrasp failed {plan.error_code.val}')
        self.report['initial_pregrasp']=self.execute(plan.trajectory,8)
        level=.406 + (sum(FOOD[k]['size'][2] for k in list(FOOD)[:3]) if top_only else 0)
        self.report['mode']='top_only_initial_stack_debug' if top_only else ('full' if full else 'bottom_bread')
        self.report['initial_ingredient_poses']=dict(self.items)
        for n in (['bread_top'] if top_only else (list(FOOD) if full else ['bread_bottom'])):
            center=level+FOOD[n]['size'][2]/2
            self.transfer(n,center);level+=FOOD[n]['size'][2]
        self.stage='final_settle';self.wait_sim(5)
        self.report['final_ingredient_poses']=dict(self.items)
        self.report['stack_contacts']=[]
        if full:
            level=.406
            for n in FOOD:
                expected=level+FOOD[n]['size'][2]/2;actual=self.items[n]
                if math.hypot(actual['x']-.55,actual['y']-.30)>.015 or abs(actual['z']-expected)>.008:
                    raise RuntimeError('Final stack position/order failed for '+n)
                tilt=2*math.acos(min(1.,math.sqrt(actual['qz']**2+actual['qw']**2)))
                if tilt>.15 or self.fingers.get(n,True):raise RuntimeError('Final stack tilt/release failed for '+n)
                recent=[p for p in self.pose_rows if p['item']==n and p['stage']=='final_settle']
                if max(max(p[a] for p in recent)-min(p[a] for p in recent) for a in ['x','y','z'])>.003:
                    raise RuntimeError('Final stack unstable for '+n)
                level+=FOOD[n]['size'][2]
            for a,b in zip(FOOD,list(FOOD)[1:]):
                matches=[r for r in self.contact_rows if r['stage']=='final_settle' and a+'::' in r['collision1']+r['collision2'] and b+'::' in r['collision1']+r['collision2']]
                if not matches:raise RuntimeError('Missing final adjacent-ingredient contact: '+a+'/'+b)
                self.report['stack_contacts'].append([a,b])
        self.report['success']=True

def main():
    out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=Sandwich(out)
    report=node.report=dict(success=False,transfers=[],artificial_attachment=False,
        human_derived='order and coarse direction only',coordinates_calibrated=False)
    try:node.run('--full' in sys.argv,'--top-only' in sys.argv)
    except Exception as e:report['error']=f'{type(e).__name__}: {e}'
    finally:
        report.update(failed_stage=node.stage if not report['success'] else None,
            unwanted_robot_support_contacts=node.unwanted,contact_pairs=sorted(node.contact_pairs),payload_envelope_checks=node.envelope_checks)
        (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
        (out/'commands.json').write_text(json.dumps(node.commands,indent=2)+'\n')
        (out/'action_results.json').write_text(json.dumps(node.action_results,indent=2)+'\n')
        for name,rows in [('joints',node.trace),('object_poses',node.pose_rows),('contacts',node.contact_rows),('controller_feedback',node.feedback),('runtime',node.rates)]:
            if rows:
                with (out/(name+'.csv')).open('w',newline='') as f:
                    writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        print(json.dumps(report,indent=2),flush=True);node.destroy_node();rclpy.shutdown()
    return 0 if report['success'] else 1

if __name__=='__main__':raise SystemExit(main())
