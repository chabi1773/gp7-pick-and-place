#!/usr/bin/env python3
"""Build a Fortress description; resolve the control library via the ROS package index."""
from pathlib import Path
import argparse
import json
import xml.etree.ElementTree as ET


def find_control_library():
    from ament_index_python.packages import get_package_prefix
    prefix = Path(get_package_prefix('gz_ros2_control'))
    library = prefix / 'lib/libgz_ros2_control-system.so'
    if not library.is_file():
        raise FileNotFoundError(
            f'Control package found at {prefix}, but its system library is missing: {library}. '
            'Check the package installation; for the Humble apt package, run '
            'sudo apt install --reinstall ros-humble-gz-ros2-control.'
        )
    return library.resolve()


def build_description(package_dir, control_library=None):
    # Fortress does not necessarily search the ROS lib directory. Supplying
    # the absolute installed path avoids reliance on the simulator search path.
    control_library = Path(control_library).resolve() if control_library is not None else find_control_library()
    if not control_library.is_file():
        raise FileNotFoundError(f'Control system library not found: {control_library}')
    package_dir = Path(package_dir).resolve()
    root = ET.parse(package_dir / 'urdf/gp7.urdf').getroot()
    bounds = json.loads((package_dir / 'checks/validation.json').read_text())['mesh_bounds_after_visual_transform']
    for mesh in root.findall('.//mesh'):
        prefix = 'package://gp7_stl_description/'
        uri = mesh.get('filename')
        if uri.startswith(prefix):
            mesh.set('filename', (package_dir / uri[len(prefix):]).as_uri())

    # Coarse external collision envelopes: much cheaper than detailed CAD meshes.
    # Self-collision is disabled; these are not accurate grasp/contact surfaces.
    for link in root.findall('link'):
        name = link.get('name')
        if name not in bounds:
            continue
        for collision in link.findall('collision'):
            link.remove(collision)
        b = bounds[name]
        lo, hi = b['minimum_m'], b['maximum_m']
        collision = ET.SubElement(link, 'collision', name=name + '_envelope')
        ET.SubElement(collision, 'origin', xyz=' '.join(str((x+y)/2) for x,y in zip(lo,hi)), rpy='0 0 0')
        geometry = ET.SubElement(collision, 'geometry')
        ET.SubElement(geometry, 'box', size=' '.join(str(y-x) for x,y in zip(lo,hi)))
        gz = ET.SubElement(root, 'gazebo', reference=name)
        ET.SubElement(gz, 'selfCollide').text = 'false'
        # URDF->SDF material handling varies; set explicit visual colours.
        visual = ET.SubElement(gz, 'visual')
        material = ET.SubElement(visual, 'material')
        rgba = '0.08 0.16 0.48 1' if name != 'link_6' else '0.7 0.7 0.74 1'
        ET.SubElement(material, 'ambient').text = rgba
        ET.SubElement(material, 'diffuse').text = rgba

    ET.SubElement(root, 'link', name='world')
    anchor = ET.SubElement(root, 'joint', name='world_to_base', type='fixed')
    ET.SubElement(anchor, 'parent', link='world')
    ET.SubElement(anchor, 'child', link='base_link')
    ET.SubElement(anchor, 'origin', xyz='0 0 0', rpy='0 0 0')

    control = ET.SubElement(root, 'ros2_control', name='GP7GazeboSystem', type='system')
    hw = ET.SubElement(control, 'hardware')
    ET.SubElement(hw, 'plugin').text = 'gz_ros2_control/GazeboSimSystem'
    for joint in root.findall('joint'):
        if joint.get('type') != 'revolute':
            continue
        entry = ET.SubElement(control, 'joint', name=joint.get('name'))
        command = ET.SubElement(entry, 'command_interface', name='position')
        limits = joint.find('limit')
        ET.SubElement(command, 'param', name='min').text = limits.get('lower')
        ET.SubElement(command, 'param', name='max').text = limits.get('upper')
        state = ET.SubElement(entry, 'state_interface', name='position')
        ET.SubElement(state, 'param', name='initial_value').text = '0.0'
        ET.SubElement(entry, 'state_interface', name='velocity')
        ET.SubElement(entry, 'state_interface', name='effort')

    gazebo = ET.SubElement(root, 'gazebo')
    plugin = ET.SubElement(gazebo, 'plugin', filename=str(control_library), name='gz_ros2_control::GazeboSimROS2ControlPlugin')
    ET.SubElement(plugin, 'robot_param').text = 'robot_description'
    ET.SubElement(plugin, 'robot_param_node').text = 'robot_state_publisher'
    ET.SubElement(plugin, 'parameters').text = str(package_dir / 'config/controllers.yaml')
    ET.indent(root, space='  ')
    return ET.tostring(root, encoding='unicode')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package-dir', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.write_text(build_description(args.package_dir), encoding='utf-8')
    print(args.output)
