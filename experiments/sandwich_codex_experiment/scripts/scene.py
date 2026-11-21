"""Explicit assumed rigid scene; no coordinates reconstructed from pixels."""
from pathlib import Path
import json
import csv
import xml.etree.ElementTree as ET
from food_visuals import ensure_assets

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT.parents[1]
FOOD = {
    'bread_bottom': dict(size=[.05,.07,.011], xyz=[.55,0,.4235], mass=.0055, color='0.85 0.55 0.25 1'),
    'tomato': dict(size=[.05,.06,.006], xyz=[.42,-.16,.420], mass=.00257, color='0.85 0.08 0.06 1'),
    'cheese': dict(size=[.05,.06,.005], xyz=[.57,-.16,.4195], mass=.00171, color='0.95 0.8 0.12 1'),
    'bread_top': dict(size=[.05,.07,.011], xyz=[.55,0,.4125], mass=.0055, color='0.85 0.55 0.25 1'),
}
SUPPORTS = {
    'table': ([1.6,1.2,.4],[.4,0,.2]),
    'source_plate': ([.34,.30,.006],[.51,-.08,.403]),
    'tomato_rest': ([.024,.046,.010],[.42,-.16,.411]),
    'cheese_rest': ([.024,.046,.010],[.57,-.16,.411]),
    'destination_plate': ([.18,.18,.006],[.55,.30,.403]),
}

def generate(top_debug=False):
    visuals=ensure_assets()
    events=list(csv.DictReader((ROOT/'data/derived/sandwich_ingredient_events_review.csv').open()))
    first=min(float(r['time_s_approx']) for r in events);last=max(float(r['time_s_approx']) for r in events)
    cues={}
    for name in FOOD:
        e={r['event']:float(r['time_s_approx']) for r in events if r['ingredient']==name}
        cues[name]=dict(events_s=e,normalized_events={k:(v-first)/(last-first) for k,v in e.items()},
            approximate_human_carry_s=e['estimated_release']-e['estimated_pickup'],
            programmed_min_transport_s=max(7.,2*(e['estimated_release']-e['estimated_pickup'])))
    (ROOT/'config/human_task_cues.json').write_text(json.dumps(dict(
        provenance='manual visual review; not contact-verified',robot_timing='slower assumed safety minima; not exact human timing',ingredients=cues),indent=2)+'\n')
    tree=ET.parse(BASE/'worlds/cup_drag.sdf'); w=tree.find('world');w.set('name','sandwich')
    for m in list(w.findall('model')):
        if m.get('name') not in ['ground']:w.remove(m)
    for name,(size,xyz) in SUPPORTS.items():
        add(w,name,size,xyz,None,'0.8 0.8 0.85 1' if name!='table' else '0.55 0.4 0.25 1')
    import copy
    initial=copy.deepcopy(FOOD)
    if top_debug:
        level=.406
        for n in list(FOOD)[:3]:
            initial[n]['xyz']=[.55,.30,level+FOOD[n]['size'][2]/2];level+=FOOD[n]['size'][2]
    for name,spec in initial.items():add(w,name,spec['size'],spec['xyz'],spec['mass'],spec['color'],visuals[name])
    ET.indent(tree);tree.write(ROOT/'worlds/sandwich.sdf',encoding='unicode',xml_declaration=True)
    (ROOT/'config/assumptions.json').write_text(json.dumps(dict(
        human_derived='order and image-right source to image-left destination only; +Y robot mapping assumed',
        dimensions_mass_friction_all_assumed=True,rigid_boxes_not_soft_food=True,
        food_appearance='Original COLLADA food meshes: crust/porous crumb, seeded tomato cut face, perforated cheese. Rendering geometry only.',
        food_collision_shapes='Thin rigid boxes matching rendered thickness; visible holes/rounded outlines do not change contact surfaces.',
        food=initial,debug_initial_stack=top_debug,supports=SUPPORTS,friction=.8,tool_quaternion_xyzw=[1,0,0,0],
        grasp_tool_height_above_item_bottom_m=dict(bread=.161,toppings=.154),transport_tool_height_m=.76,
        release_clearance_above_support_m=dict(bread=.005,toppings=.009),
        anchored_robot_base_height_m=.401,mount_clearance_above_table_m=.001,
        planning_payload='No attached collision object; held world proxy removed to prevent stale obstacle. Physics contacts retained.',
        bread_stack='upper source slice called bread_bottom because it becomes sandwich base',
        trays='flat 6mm shallow plate proxies without lips',
        joint_controller='unchanged validated vendor/controller configuration'),indent=2)+'\n')

def add(w,name,size,xyz,mass,color,visual_mesh=None):
    m=ET.SubElement(w,'model',name=name)
    ET.SubElement(m,'pose').text=' '.join(map(str,xyz))+' 0 0 0'
    if mass is None:ET.SubElement(m,'static').text='true'
    l=ET.SubElement(m,'link',name='body')
    if mass:
        inertial=ET.SubElement(l,'inertial');ET.SubElement(inertial,'mass').text=str(mass)
        inertia=ET.SubElement(inertial,'inertia')
        for axis,a,b in [('ixx',1,2),('iyy',0,2),('izz',0,1)]:ET.SubElement(inertia,axis).text=str(mass*(size[a]**2+size[b]**2)/12)
    for tag in ['collision','visual']:
        element=ET.SubElement(l,tag,name=tag);g=ET.SubElement(element,'geometry')
        if tag=='visual' and visual_mesh:
            mesh=ET.SubElement(g,'mesh')
            ET.SubElement(mesh,'uri').text=visual_mesh.as_uri()
            original_height=.04 if name.startswith('bread') else .035
            ET.SubElement(mesh,'scale').text=f'1 1 {size[2]/original_height:.12g}'
            ET.SubElement(element,'cast_shadows').text='true'
        else:ET.SubElement(ET.SubElement(g,'box'),'size').text=' '.join(map(str,size))
        if tag=='visual' and not visual_mesh:ET.SubElement(ET.SubElement(element,'material'),'diffuse').text=color
        elif tag=='collision':
            ode=ET.SubElement(ET.SubElement(ET.SubElement(element,'surface'),'friction'),'ode')
            for f in ['mu','mu2']:ET.SubElement(ode,f).text='.8'
    sensor=ET.SubElement(l,'sensor',name='contacts',type='contact')
    ET.SubElement(sensor,'always_on').text='true';ET.SubElement(sensor,'update_rate').text='50'
    ET.SubElement(ET.SubElement(sensor,'contact'),'collision').text='collision'

if __name__=='__main__':generate()
