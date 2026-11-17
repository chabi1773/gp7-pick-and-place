#!/usr/bin/env python3
"""Offline FK, numerical IK and smooth Cartesian demo planner (metres/radians).

This is a software reference, not the assignment's analytical IK derivation.
"""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

JOINT_NAMES = ['joint_' + str(i) for i in range(1, 7)]
DOWN = np.diag([1., -1., -1.])
TOOL_LENGTH = .100


def scene():
    # User correction: Box A bottom centre (0.5, 0.5, 0); SDF origin is centre.
    # Reproducible random reachable Box B sample. Rebuild the world after changes.
    rng = np.random.default_rng(7)
    b = [float(rng.uniform(.38, .46)), float(rng.uniform(-.32, -.22)), .025]
    return dict(a=[.50, .50, .1], b=b, a_size=[.30, .30, .20], b_size=.05,
                b_sample_seed=7, a_bottom_center=[.5, .5, 0])


def transform(xyz=(0, 0, 0), rotation=None):
    t = np.eye(4)
    t[:3, 3] = xyz
    if rotation is not None:
        t[:3, :3] = rotation
    return t


class Robot:
    def __init__(self, package):
        self.package = Path(package)
        root = ET.parse(self.package / 'urdf/gp7.urdf').getroot()
        self.joints = []
        self.lower, self.upper, self.speed = [], [], []
        for j in root.findall('joint'):
            o = j.find('origin')
            xyz = np.fromstring(o.get('xyz', '0 0 0'), sep=' ') if o is not None else np.zeros(3)
            rpy = np.fromstring(o.get('rpy', '0 0 0'), sep=' ') if o is not None else np.zeros(3)
            axis = np.fromstring(j.find('axis').get('xyz'), sep=' ') if j.get('type') == 'revolute' else None
            name = j.get('name')
            index = JOINT_NAMES.index(name) if axis is not None else None
            self.joints.append((j.find('parent').get('link'), j.find('child').get('link'),
                                transform(xyz, Rotation.from_euler('xyz', rpy).as_matrix()), axis, index))
            if axis is not None:
                limit = j.find('limit')
                self.lower.append(float(limit.get('lower')))
                self.upper.append(float(limit.get('upper')))
                self.speed.append(float(limit.get('velocity')))
        self.lower, self.upper, self.speed = map(np.array, (self.lower, self.upper, self.speed))
        bounds = json.loads((self.package / 'checks/validation.json').read_text())['mesh_bounds_after_visual_transform']
        self.boxes = {}
        for link, box in bounds.items():
            lo, hi = np.array(box['minimum_m']), np.array(box['maximum_m'])
            self.boxes[link] = ((lo + hi) / 2, (hi - lo) / 2)
        self.boxes['tool0'] = (np.array([0, 0, .05]), np.array([.022, .022, .05]))

    def frames(self, q):
        frames = {'base_link': np.eye(4)}
        for parent, child, origin, axis, index in self.joints:
            if parent not in frames:
                raise ValueError('URDF joints must be in parent-before-child order')
            t = origin if axis is None else origin @ transform(rotation=Rotation.from_rotvec(axis * q[index]).as_matrix())
            frames[child] = frames[parent] @ t
        frames['grasp_tcp'] = frames['tool0'] @ transform([0, 0, TOOL_LENGTH])
        return frames

    def tcp(self, q):
        return self.frames(q)['grasp_tcp']

    def ik(self, xyz, seed, rotation=DOWN, retry=False):
        def residual(q):
            t = self.tcp(q)
            return np.r_[t[:3, 3] - xyz, .3 * Rotation.from_matrix(rotation @ t[:3, :3].T).as_rotvec()]
        seeds = [np.asarray(seed)]
        if retry:
            rng = np.random.default_rng(7)
            seeds.extend(rng.uniform(self.lower + .01, self.upper - .01, size=(18, 6)))
        for s in seeds:
            result = least_squares(residual, np.clip(s, self.lower + 1e-6, self.upper - 1e-6),
                                   bounds=(self.lower, self.upper), xtol=1e-10, ftol=1e-10, gtol=1e-10, max_nfev=140)
            if np.linalg.norm(residual(result.x)) < 1e-6:
                return result.x
        raise ValueError('No IK solution found at ' + str(np.round(xyz, 4).tolist()))

    def collision_check(self, q, config, carrying=False, placed=False):
        """Conservative robot envelopes against floor and Box A, not self-collision."""
        frames = self.frames(q)
        minimum = float('inf')
        for link, (center, half) in self.boxes.items():
            t = frames[link]
            c, r = t[:3, :3] @ center + t[:3, 3], t[:3, :3]
            bottom = c[2] - np.abs(r[2]) @ half
            if link != 'base_link':
                minimum = min(minimum, bottom)
                if bottom < -.0001:
                    raise ValueError(f'{link} envelope intersects ground ({bottom:.4f} m)')
            if obb_overlap(c, r, half, np.array(config['a']), np.eye(3), np.array(config['a_size']) / 2):
                raise ValueError(f'{link} envelope intersects Box A')
            if placed:
                box_center = np.array([config['a'][0], config['a'][1], .225])
                if obb_overlap(c, r, half, box_center, np.eye(3), np.full(3, .025)):
                    raise ValueError(f'{link} envelope intersects placed Box B')
        if carrying:
            # The box top is 1 mm below the TCP at attachment.
            t = frames['grasp_tcp']
            c = t[:3, 3] + t[:3, :3] @ np.array([0, 0, .026])
            if c[2] - .025 < -.0001:
                raise ValueError('Carried Box B intersects ground')
            if obb_overlap(c, t[:3, :3], np.full(3, .025), np.array(config['a']), np.eye(3), np.array(config['a_size']) / 2):
                raise ValueError('Carried Box B intersects Box A')
        return minimum


def obb_overlap(c1, r1, h1, c2, r2, h2):
    # Separating-axis theorem, including all nine cross-product directions.
    axes = [*r1.T, *r2.T, *(np.cross(a, b) for a in r1.T for b in r2.T)]
    for axis in axes:
        if np.linalg.norm(axis) < 1e-10:
            continue
        extent = np.abs(axis @ r1) @ h1 + np.abs(axis @ r2) @ h2
        if abs(axis @ (c2 - c1)) > extent + 1e-8:
            return False
    return True


def smooth(u):
    return 10*u**3 - 15*u**4 + 6*u**5


def make_plan(package, hz=20):
    robot, config = Robot(package), scene()
    a, b = config['a'], config['b']
    target = np.array([a[0], a[1], .253])
    # Triangle-inequality upper bound from joint 1 to the tool contact point.
    bound = .040 + .445 + np.hypot(.440, .040) + .080 + TOOL_LENGTH
    distance = np.linalg.norm(target - [0, 0, .330])
    if distance > bound:
        raise ValueError(f'TASK TARGET UNREACHABLE: required distance {distance:.3f} m exceeds '
                         f'conservative reach bound {bound:.3f} m. Robot base and Box A were not moved. '
                         'Choose a reachable target before executing.')
    pick = np.array([b[0], b[1], .051])
    above_pick, above_place = pick.copy(), target.copy()
    above_pick[2] = above_place[2] = .40
    q = robot.ik(above_pick, np.zeros(6), retry=True)
    segments = []
    def append(name, positions, duration, carrying=False):
        samples = np.asarray(positions)
        dt = duration / (len(samples) - 1)
        velocity = np.gradient(samples, dt, axis=0)
        velocity[[0, -1]] = 0
        if (np.abs(velocity) > np.minimum(robot.speed, .65) + .001).any():
            raise ValueError(f'{name}: too fast; increase segment duration')
        placed = name in ('retreat', 'return_home')
        minimum = min(robot.collision_check(s, config, carrying, placed) for s in samples)
        segments.append(dict(name=name, duration=duration, carrying=carrying,
                             q=samples.tolist(), velocity=velocity.tolist(), min_floor_clearance_m=minimum))
    count = int(10 * hz) + 1
    append('approach', [smooth(u)*q for u in np.linspace(0, 1, count)], 10)
    for name, start, end, seconds, carrying in [
        ('descend_to_grasp', above_pick, pick, 6, False),
        ('lift', pick, above_pick, 6, True),
        ('transfer', above_pick, above_place, 8, True),
        ('lower', above_place, target, 5, True),
        ('retreat', target, above_place, 5, False),
    ]:
        positions = []
        for u in np.linspace(0, 1, int(seconds*hz) + 1):
            q = robot.ik(start + smooth(u)*(end-start), q)
            positions.append(q.copy())
        append(name, positions, seconds, carrying)
    # Release and vertical retreat happen first. Return smoothly to the
    # original zero-joint pose with the empty gripper.
    home_start = q.copy()
    append('return_home', [(1-smooth(u))*home_start for u in np.linspace(0, 1, int(10*hz)+1)], 10)
    return dict(scene=config, joint_names=JOINT_NAMES, hz=hz,
                interpolation='cubic Hermite joint positions/velocities; quintic Cartesian timing',
                gripper='ideal detachable fixed joint; not friction/contact grasping', segments=segments)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package-dir', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    plan = make_plan(args.package_dir)
    if args.output:
        args.output.write_text(json.dumps(plan, indent=2) + '\n')
    print('Plan passed:', [(s['name'], s['duration']) for s in plan['segments']])
