#!/usr/bin/env python3
"""Create a gripper description and the corrected pick-and-place scene."""
import xml.etree.ElementTree as ET
from gazebo_description import build_description
from pick_place_model import scene


def add_text(parent, tag, value):
    ET.SubElement(parent, tag).text = str(value)


def description(package, control_library=None):
    root = ET.fromstring(build_description(package, control_library))
    # Two primitives with explicitly assumed masses, not CAD measurements.
    for name, radius, length, center, mass in [
        ('gripper_stem', .012, .085, .0425, .12),
        ('suction_pad', .022, .015, .0925, .03),
    ]:
        link = ET.SubElement(root, 'link', name=name)
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'origin', xyz='0 0 0')
        ET.SubElement(inertial, 'mass', value=str(mass))
        lateral, axial = mass*(3*radius**2+length**2)/12, mass*radius**2/2
        ET.SubElement(inertial, 'inertia', ixx=str(lateral), iyy=str(lateral), izz=str(axial), ixy='0', ixz='0', iyz='0')
        for kind in ['visual', 'collision']:
            obj = ET.SubElement(link, kind)
            geometry = ET.SubElement(obj, 'geometry')
            ET.SubElement(geometry, 'cylinder', radius=str(radius), length=str(length))
            if kind == 'visual':
                material = ET.SubElement(obj, 'material', name=name+'_material')
                ET.SubElement(material, 'color', rgba='.2 .2 .2 1')
        joint = ET.SubElement(root, 'joint', name='tool0_to_'+name, type='fixed')
        ET.SubElement(joint, 'parent', link='tool0')
        ET.SubElement(joint, 'child', link=name)
        ET.SubElement(joint, 'origin', xyz=f'0 0 {center}', rpy='0 0 0')
    ET.SubElement(root, 'link', name='grasp_tcp')
    joint = ET.SubElement(root, 'joint', name='tool0_to_grasp_tcp', type='fixed')
    ET.SubElement(joint, 'parent', link='tool0')
    ET.SubElement(joint, 'child', link='grasp_tcp')
    ET.SubElement(joint, 'origin', xyz='0 0 .1', rpy='0 0 0')
    gazebo = ET.SubElement(root, 'gazebo')
    # link_6 remains after Gazebo lumps the fixed flange/tool/gripper links.
    plugin = ET.SubElement(gazebo, 'plugin', filename='ignition-gazebo-detachable-joint-system',
                           name='ignition::gazebo::systems::DetachableJoint')
    for tag, value in dict(parent_link='link_6', child_model='box_b', child_link='body',
                           initial_attach='true', attach_topic='/gp7/gripper/attach',
                           detach_topic='/gp7/gripper/detach', output_topic='/gp7/gripper/state').items():
        add_text(plugin, tag, value)
    ET.indent(root)
    return ET.tostring(root, encoding='unicode')


def world(package):
    root = ET.parse(package / 'worlds/gp7_world.sdf').getroot()
    w = root.find('world')
    cfg = scene()
    for name, xyz, size, static, color in [
        ('box_a', cfg['a'], cfg['a_size'], True, '.15 .55 .24 1'),
        ('box_b', cfg['b'], [.05]*3, False, '.95 .4 .05 1'),
    ]:
        model = ET.SubElement(w, 'model', name=name)
        add_text(model, 'static', str(static).lower())
        add_text(model, 'pose', ' '.join(map(str, xyz))+' 0 0 0')
        link = ET.SubElement(model, 'link', name='body')
        if not static:
            inertial = ET.SubElement(link, 'inertial')
            add_text(inertial, 'mass', .1)
            inertia = ET.SubElement(inertial, 'inertia')
            for tag in ['ixx', 'iyy', 'izz']:
                add_text(inertia, tag, .1*.05**2/6)
            for tag in ['ixy', 'ixz', 'iyz']:
                add_text(inertia, tag, 0)
        for kind in ['visual', 'collision']:
            obj = ET.SubElement(link, kind, name=kind)
            add_text(ET.SubElement(ET.SubElement(obj, 'geometry'), 'box'), 'size', ' '.join(map(str, size)))
            if kind == 'visual':
                material = ET.SubElement(obj, 'material')
                add_text(material, 'ambient', color)
                add_text(material, 'diffuse', color)
            else:
                ode = ET.SubElement(ET.SubElement(ET.SubElement(obj, 'surface'), 'friction'), 'ode')
                add_text(ode, 'mu', .8)
                add_text(ode, 'mu2', .8)
        if not static:
            plugin = ET.SubElement(model, 'plugin', filename='ignition-gazebo-pose-publisher-system',
                                   name='ignition::gazebo::systems::PosePublisher')
            for tag, value in dict(publish_model_pose='true', publish_link_pose='false',
                                  publish_nested_model_pose='true', publish_visual_pose='false',
                                  publish_collision_pose='false', publish_sensor_pose='false',
                                  use_pose_vector_msg='false', update_frequency=30).items():
                add_text(plugin, tag, value)
    ET.indent(root)
    return ET.tostring(root, encoding='unicode')
