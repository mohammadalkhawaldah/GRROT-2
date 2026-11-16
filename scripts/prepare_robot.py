#!/usr/bin/env python3
"""Generate a simulation-only URDF from installed, unmodified vendor assets."""
from pathlib import Path
import json
import re
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro
import yaml

ROOT = Path(__file__).resolve().parents[1]


def prepare():
    vendor = Path(get_package_share_directory('kortex_description'))
    compat = ROOT / 'outputs/description_compat'
    compat.mkdir(parents=True, exist_ok=True)
    # Kortex 0.2.3 passes two optional arguments missing in Robotiq 0.0.1.
    # Adapt only a local macro copy; preserve the vendor robot geometry.
    gripper_source = (vendor / 'grippers/robotiq_2f_85/urdf/robotiq_2f_85_macro.xacro').read_text()
    gripper_source = re.sub(r'\s+isaac_joint_(?:commands|states)="[^"]+"', '', gripper_source)
    (compat / 'gripper.xacro').write_text(gripper_source)
    macro_source = (vendor / 'robots/kortex_robot.xacro').read_text().replace(
        '$(find kortex_description)/grippers/${gripper}/urdf/${gripper}_macro.xacro',
        str(compat / 'gripper.xacro'))
    (compat / 'kortex_robot.xacro').write_text(macro_source)
    robot_source = (vendor / 'robots/kinova.urdf.xacro').read_text().replace(
        '$(find kortex_description)/robots/kortex_robot.xacro', str(compat / 'kortex_robot.xacro'))
    # The vendor's external-cable option supplies +/-2pi revolute bounds. This
    # prevents continuous-angle normalization from introducing a wrap in the
    # planned Cartesian path / trajectory-controller handoff.
    robot_source = robot_source.replace('parent="world"',
        'parent="world" use_external_cable="true"', 1)
    (compat / 'kinova.urdf.xacro').write_text(robot_source)
    doc = xacro.process_file(str(compat / 'kinova.urdf.xacro'), mappings={
        'name': 'gen3', 'arm': 'gen3', 'dof': '7', 'gripper': 'robotiq_2f_85',
        'sim_ignition': 'true', 'sim_gazebo': 'false', 'vision': 'false',
        'simulation_controllers': str(ROOT / 'config/controllers.yaml'),
    })
    robot = ET.fromstring(doc.toxml())
    # The stock world joint anchors the robot. Place its base on the table.
    robot.find("joint[@name='base_joint']/origin").set('xyz', '0 0 0.4')
    changes = []
    interface_additions = []
    for control in robot.findall('ros2_control'):
        plugin = control.find('hardware/plugin')
        changes.append({'hardware': control.get('name'), 'original_plugin': plugin.text})
        plugin.text = 'gz_ros2_control/GazeboSimSystem'
        for joint in control.findall('joint'):
            if joint.find("param[@name='mimic']") is not None:
                # Packaged Robotiq omits these for old Ignition; current control
                # needs state interfaces for measured finger feedback.
                for tag, name in [('command_interface', 'position'),
                                  ('state_interface', 'position'), ('state_interface', 'velocity')]:
                    if joint.find(f"{tag}[@name='{name}']") is None:
                        ET.SubElement(joint, tag, {'name': name})
                        interface_additions.append(f'{joint.get("name")}/{tag}/{name}')
            if not joint.get('name').startswith('joint_'):
                state = joint.find("state_interface[@name='position']")
                if state is not None:
                    initial = state.find("param[@name='initial_value']")
                    if initial is None:
                        initial = ET.SubElement(state, 'param', {'name': 'initial_value'})
                    multiplier = float(joint.findtext("param[@name='multiplier']", '1'))
                    initial.text = str(0.1 * multiplier)
    # Replace legacy Ignition plugin spelling; omit rendering in the first test.
    for gazebo in list(robot.findall('gazebo')):
        for child in list(gazebo):
            if child.tag in ('plugin', 'sensor'):
                gazebo.remove(child)
    gazebo = ET.SubElement(robot, 'gazebo')
    plugin = ET.SubElement(gazebo, 'plugin', {
        'filename': 'gz_ros2_control-system',
        'name': 'gz_ros2_control::GazeboSimROS2ControlPlugin',
    })
    ET.SubElement(plugin, 'parameters').text = str(ROOT / 'config/controllers.yaml')
    control_joints = [j.get('name') for c in robot.findall('ros2_control') for j in c.findall('joint')]
    gripper_joints = [n for n in control_joints if not n.startswith('joint_')]
    primary = next((j.get('name') for c in robot.findall('ros2_control') for j in c.findall('joint')
                    if not j.get('name').startswith('joint_') and j.find('command_interface') is not None
                    and j.find("param[@name='mimic']") is None), None)
    if primary is None:
        raise RuntimeError('No actuated gripper joint found in vendor description')
    config = yaml.safe_load((ROOT / 'config/controllers.yaml').read_text())
    config['gripper_controller']['ros__parameters']['joints'] = [primary]
    (ROOT / 'outputs/controllers.generated.yaml').write_text(yaml.safe_dump(config))
    plugin.find('parameters').text = str(ROOT / 'outputs/controllers.generated.yaml')
    path = ROOT / 'outputs/robot.urdf'
    ET.indent(robot)
    temporary = path.with_suffix('.urdf.tmp')
    ET.ElementTree(robot).write(temporary, encoding='unicode', xml_declaration=True)
    temporary.replace(path)
    manifest = {'vendor_description': str(vendor), 'changes': changes,
                'compatibility': 'Local macro omits unsupported optional isaac_joint_commands/states arguments',
                'mimic_interface_additions': interface_additions,
                'gripper_initial_position_rad': 0.1,
                'arm_joint_mode': 'Vendor use_external_cable=true; bounded revolute +/-2pi for joints 1,3,5,7',
                'gripper_primary': primary, 'gripper_control_joints': gripper_joints,
                'base_height_m': 0.4, 'sensors_enabled': False,
                'artificial_cup_attachment': False}
    (ROOT / 'outputs/robot_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return path


if __name__ == '__main__':
    print(prepare())
