#!/usr/bin/env python3
"""Offline checks, including joint-controller cubic interpolation between points."""
from pathlib import Path
import ast
import json
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from pick_place_model import Robot, DOWN, make_plan
from pick_place_scene import world


def check(package):
    robot = Robot(package)
    plan = make_plan(package)
    max_rotation, max_speed, max_acceleration, checked = 0., 0., 0., 0
    min_floor = float('inf')
    for segment in plan['segments']:
        positions, velocities = np.array(segment['q']), np.array(segment['velocity'])
        dt = segment['duration']/(len(positions)-1)
        assert np.max(np.abs(velocities[[0, -1]])) == 0
        for i in range(len(positions)-1):
            p0, p1, v0, v1 = positions[i], positions[i+1], velocities[i], velocities[i+1]
            for t in np.linspace(0, 1, 5):
                q = (2*t**3-3*t*t+1)*p0 + (t**3-2*t*t+t)*dt*v0 + (-2*t**3+3*t*t)*p1 + (t**3-t*t)*dt*v1
                v = ((6*t*t-6*t)*p0 + (3*t*t-4*t+1)*dt*v0 + (-6*t*t+6*t)*p1 + (3*t*t-2*t)*dt*v1)/dt
                acceleration = ((12*t-6)*p0 + (6*t-4)*dt*v0 + (-12*t+6)*p1 + (6*t-2)*dt*v1)/dt**2
                assert (q >= robot.lower-1e-8).all() and (q <= robot.upper+1e-8).all()
                assert (np.abs(v) < robot.speed).all()
                max_speed = max(max_speed, np.max(np.abs(v)))
                max_acceleration = max(max_acceleration, np.max(np.abs(acceleration)))
                min_floor = min(min_floor, robot.collision_check(q, plan['scene'], segment['carrying'], segment['name'] in ('retreat', 'return_home')))
                if segment['name'] not in ('approach', 'return_home'):
                    angle = Rotation.from_matrix(robot.tcp(q)[:3, :3] @ DOWN.T).magnitude()
                    max_rotation = max(max_rotation, angle)
                checked += 1
    assert max_rotation < np.deg2rad(.01)
    assert plan['segments'][-1]['name'] == 'return_home'
    assert not plan['segments'][-1]['carrying']
    assert np.allclose(plan['segments'][-1]['q'][-1], np.zeros(6), atol=1e-12)
    for previous, following in zip(plan['segments'], plan['segments'][1:]):
        assert np.allclose(previous['q'][-1], following['q'][0], atol=1e-6)
    # Scene XML is generated from the same authoritative configuration as IK.
    installed_world = ET.parse(package/'worlds/pick_place.sdf').getroot()
    generated_world = ET.fromstring(world(package))
    assert ET.tostring(installed_world) == ET.tostring(generated_world)
    assert plan['scene']['a_bottom_center'] == [.5, .5, 0]
    for source in [*(package/'scripts').glob('*.py'), *(package/'launch').glob('*.py')]:
        ast.parse(source.read_text())
    for path in [package/'package.xml', package/'urdf/gp7.urdf', *(package/'worlds').glob('*.sdf')]:
        ET.parse(path)
    result = dict(status='offline checks passed; Gazebo/ROS execution not tested here',
                  box_a_bottom_center_m=plan['scene']['a_bottom_center'],
                  box_b_initial_center_m=plan['scene']['b'],
                  checked_interpolated_configurations=checked,
                  max_planned_tool_orientation_error_degrees=float(np.rad2deg(max_rotation)),
                  max_joint_speed_rad_s=float(max_speed),
                  max_joint_acceleration_rad_s2=float(max_acceleration),
                  min_moving_robot_floor_clearance_m=float(min_floor),
                  total_motion_seconds=sum(s['duration'] for s in plan['segments']),
                  final_joint_positions_rad=plan['segments'][-1]['q'][-1],
                  limitations=['Self-collision disabled, not checked.',
                               'Coarse link bounding boxes, not precise contact geometry.',
                               'Numerical IK with quintic Cartesian timing; not an analytical IK derivation.',
                               'Ideal attachment joint, not a physical suction/friction model.',
                               'Trajectory checks do not prove dynamic tracking accuracy.'])
    (package/'checks/pick_place_validation.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return result


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package-dir', type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    check(args.package_dir)
