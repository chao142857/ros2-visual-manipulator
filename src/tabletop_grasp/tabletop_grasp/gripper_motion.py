"""Offline candidate gripper kinematics; no ROS or hardware dependencies.

Native CAD geometry is in mm. A single approximate configuration is the source
for the CLI, static browser preview and detached ROS markers. Its estimates do
not grant any existing planning or hardware verification claim.
"""

import argparse
import csv
import itertools
import json
import math
import os
from pathlib import Path
import sys


GROUP_NAMES = ('fixed', 'left_drive', 'right_drive', 'left_inner',
               'right_inner', 'left_finger', 'right_finger')
STATUS = 'approximate_unverified'


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a finite number')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return value


def data_path(filename):
    """Find source or installed package data without importing ROS."""
    candidates = [Path(__file__).resolve().parents[1] / 'config' / filename]
    prefixes = [sys.prefix] + os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
    candidates.extend(Path(p) / 'share/tabletop_grasp/config' / filename
                      for p in prefixes if p)
    for path in candidates:
        if path.is_file():
            return path
    raise FileNotFoundError(f'Cannot find {filename}; supply an explicit path')


def interpolate(value, rows):
    for (x0, y0), (x1, y1) in zip(rows, rows[1:]):
        if value <= x1:
            return y0 + (y1 - y0) * (value - x0) / (x1 - x0)
    return rows[-1][1]


class MotionModel:
    """Ideal symmetric four-bar candidate, limited to the supplied pulse table."""

    @classmethod
    def from_json(cls, path):
        with open(path, encoding='utf-8') as source:
            return cls(json.load(source))

    def __init__(self, configuration):
        # Round-trip copy also isolates later caller changes from the live model.
        if not isinstance(configuration, dict):
            raise ValueError('Motion configuration must be an object')
        self.configuration = json.loads(json.dumps(configuration, allow_nan=False))
        cfg = self.configuration
        if (cfg.get('schema_version') != 1 or cfg.get('status') != STATUS
                or cfg.get('hardware_verified') is not False
                or cfg.get('units') != 'mm' or cfg.get('frame_id') != 'gripper_cad_estimate'):
            raise ValueError('Requires an explicitly unverified native-CAD estimate in mm')
        digest = cfg.get('source_sha256', '')
        if len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest):
            raise ValueError('source_sha256 must identify the source STEP')
        limits = cfg['pulse_range_us']
        if len(limits) != 2:
            raise ValueError('pulse_range_us must have two endpoints')
        self.pulse_min_us, self.pulse_max_us = [finite(x, 'pulse endpoint') for x in limits]
        if not 0 < self.pulse_min_us < self.pulse_max_us:
            raise ValueError('invalid pulse interval')
        self.closed_pulse_us = finite(cfg['closed_pulse_us'], 'closed_pulse_us')
        if self.closed_pulse_us != self.pulse_max_us:
            raise ValueError('This candidate uses the high-pulse endpoint as closed')
        self.arm_mm = finite(cfg['arm_length_mm'], 'arm_length_mm')
        self.side_mm = finite(cfg['side_length_mm'], 'side_length_mm')
        self.reference_dx_mm = finite(cfg['reference_dx_mm'], 'reference_dx_mm')
        self.reference_dy_mm = finite(cfg['reference_dy_mm'], 'reference_dy_mm')
        self.reference_gap_mm = finite(cfg['reference_inner_gap_mm'], 'reference_inner_gap_mm')
        if min(self.arm_mm, self.side_mm, self.reference_dx_mm,
               self.reference_dy_mm, self.reference_gap_mm) <= 0:
            raise ValueError('Candidate lengths and reference projections must be positive')
        if not math.isclose(math.hypot(self.reference_dx_mm, self.reference_dy_mm),
                            self.arm_mm, abs_tol=0.001, rel_tol=0):
            raise ValueError('Reference projections must match the measured arm length')
        self.reference_theta_rad = math.atan2(self.reference_dx_mm, self.reference_dy_mm)
        self.gap_offset_mm = self.reference_gap_mm - 2 * self.reference_dx_mm
        closed_gap = finite(cfg['assumed_closed_gap_mm'], 'assumed_closed_gap_mm')
        close_x = (closed_gap - self.gap_offset_mm) / 2
        if closed_gap < 0 or not 0 <= close_x <= self.arm_mm:
            raise ValueError('Closed-gap assumption is incompatible with the linkage')
        self.close_y_mm = math.sqrt(self.arm_mm ** 2 - close_x ** 2)
        samples = cfg['tcp_extension_samples']
        if len(samples) < 2 or any(len(row) != 2 for row in samples):
            raise ValueError('At least two pulse / TCP samples are required')
        self.tcp_samples = [[finite(p, 'sample pulse'), finite(d, 'sample TCP')]
                            for p, d in samples]
        if (self.tcp_samples[0][0] != self.pulse_min_us
                or self.tcp_samples[-1][0] != self.pulse_max_us):
            raise ValueError('TCP table must cover the pulse interval exactly')
        if any(p1 <= p0 or d1 <= d0 for (p0, d0), (p1, d1)
               in zip(self.tcp_samples, self.tcp_samples[1:])):
            raise ValueError('Pulse and TCP samples must each be strictly increasing')
        if self.tcp_samples[0][1] <= 0:
            raise ValueError('TCP extension must be positive')
        self.closed_tcp_mm = self.tcp_samples[-1][1]
        self.pivots = {}
        for name in GROUP_NAMES[1:5]:
            point = cfg['pivots_mm'][name]
            if len(point) != 3:
                raise ValueError(f'{name} pivot must be a 3-vector')
            self.pivots[name] = [finite(x, f'{name} pivot') for x in point]
        for side in ('left', 'right'):
            spacing = math.dist(self.pivots[side + '_drive'], self.pivots[side + '_inner'])
            if not math.isclose(spacing, self.side_mm, abs_tol=0.001, rel_tol=0):
                raise ValueError('Pivot spacing must match the measured side length')
        left_center = sum(self.pivots[n][0] for n in ('left_drive', 'right_drive')) / 2
        inner_center = sum(self.pivots[n][0] for n in ('left_inner', 'right_inner')) / 2
        if not math.isclose(left_center, inner_center, abs_tol=0.001, rel_tol=0):
            raise ValueError('Drive and inner pivots must share a symmetry plane')
        for kind in ('drive', 'inner'):
            if any(not math.isclose(self.pivots['left_' + kind][i],
                                    self.pivots['right_' + kind][i], abs_tol=0.001)
                   for i in (1, 2)):
                raise ValueError('Left and right pivot axes must be symmetric')
        # Validate every interpolation interval; y is monotonic between endpoints.
        for pulse, _ in self.tcp_samples:
            self.state(pulse)

    def _transforms(self, dx, dy, theta):
        result = {'fixed': {'translation_mm': [0., 0., 0.], 'rotation_z_rad': 0.}}
        for name in GROUP_NAMES[1:5]:
            delta = (theta - self.reference_theta_rad) * (-1 if name.startswith('left') else 1)
            c, s = math.cos(delta), math.sin(delta)
            x, y, _ = self.pivots[name]
            result[name] = {'translation_mm': [x - c*x + s*y, y - s*x - c*y, 0.],
                            'rotation_z_rad': delta}
        for side, sign in (('left', -1), ('right', 1)):
            result[side + '_finger'] = {
                'translation_mm': [sign * (dx - self.reference_dx_mm),
                                   self.reference_dy_mm - dy, 0.],
                'rotation_z_rad': 0.}
        return result

    def state(self, pulse_us):
        pulse = finite(pulse_us, 'pulse_us')
        if not self.pulse_min_us <= pulse <= self.pulse_max_us:
            raise ValueError(
                f'pulse_us must be within {self.pulse_min_us:g}..{self.pulse_max_us:g}')
        tcp = interpolate(pulse, self.tcp_samples)
        dy = self.close_y_mm + tcp - self.closed_tcp_mm
        square = self.arm_mm ** 2 - dy ** 2
        if square < -1e-8:
            raise ValueError('TCP table is incompatible with this linkage length')
        dx = math.sqrt(max(0., square))
        gap = 2 * dx + self.gap_offset_mm
        if gap < -1e-7:
            raise ValueError('Candidate geometry gives overlapping projected pads')
        theta = math.atan2(dx, dy)
        return {'schema_version': 1, 'status': STATUS, 'hardware_verified': False,
                'frame_id': 'gripper_cad_estimate', 'pulse_us': pulse,
                'opening_mm': max(0., gap), 'tcp_extension_mm': tcp,
                'theta_rad': theta, 'theta_deg': math.degrees(theta),
                'transforms': self._transforms(dx, dy, theta)}

    def reference_state(self):
        """Exact original CAD pose, independent of approximate pulse calibration."""
        return {'status': STATUS, 'hardware_verified': False, 'pulse_us': None,
                'frame_id': 'gripper_cad_estimate', 'opening_mm': self.reference_gap_mm,
                'theta_rad': self.reference_theta_rad,
                'transforms': self._transforms(self.reference_dx_mm, self.reference_dy_mm,
                                               self.reference_theta_rad)}

    def _pulse_for_tcp(self, tcp):
        if not self.tcp_samples[0][1] - 1e-8 <= tcp <= self.closed_tcp_mm + 1e-8:
            return None
        return min(self.pulse_max_us, max(self.pulse_min_us,
                   interpolate(tcp, [(d, p) for p, d in self.tcp_samples])))

    def opening_to_pulses(self, opening_mm):
        """Return every valid candidate; never choose an ambiguous branch silently."""
        width = finite(opening_mm, 'opening_mm')
        if width < 0:
            raise ValueError('opening_mm must be nonnegative')
        dx = (width - self.gap_offset_mm) / 2
        if not 0 <= dx <= self.arm_mm + 1e-10:
            return []
        dy_abs = math.sqrt(max(0., self.arm_mm ** 2 - dx ** 2))
        result = []
        for dy in (-dy_abs, dy_abs):
            tcp = self.closed_tcp_mm + dy - self.close_y_mm
            pulse = self._pulse_for_tcp(tcp)
            if pulse is not None and not any(abs(pulse - x) < 1e-6 for x in result):
                result.append(pulse)
        return sorted(result)

    def range_summary(self):
        candidates = [self.state(self.pulse_min_us), self.state(self.pulse_max_us)]
        peak = self._pulse_for_tcp(self.closed_tcp_mm - self.close_y_mm)
        if peak is not None:
            candidates.append(self.state(peak))
        minimum = min(candidates, key=lambda state: state['opening_mm'])
        maximum = max(candidates, key=lambda state: state['opening_mm'])
        return {'status': STATUS, 'hardware_verified': False,
                'min_opening_mm': minimum['opening_mm'], 'min_at_pulse_us': minimum['pulse_us'],
                'max_opening_mm': maximum['opening_mm'], 'max_at_pulse_us': maximum['pulse_us'],
                'has_turning_point': (peak is not None
                                      and self.pulse_min_us < peak < self.pulse_max_us),
                'closed_gap_is_assumed': True,
                'note': 'Geometric estimate under the configured assumptions; not measured stops.'}


def transform_point(point, transform):
    angle = transform['rotation_z_rad']
    c, s = math.cos(angle), math.sin(angle)
    x, y, z = point
    tx, ty, tz = transform['translation_mm']
    return [c*x - s*y + tx, s*x + c*y + ty, z + tz]


def load_parts(path, model):
    parts = json.loads(Path(path).read_text(encoding='utf-8'))
    if parts.get('source_sha256') != model.configuration['source_sha256']:
        raise ValueError('CAD parts and motion model have different source hashes')
    groups = parts['groups']
    if sorted(group['name'] for group in groups) != sorted(GROUP_NAMES):
        raise ValueError('Expected each of the seven rigid groups exactly once')
    for group in groups:
        b = group['bounds_mm']
        if len(b) != 6 or any(finite(b[i], 'bound') >= finite(b[i+3], 'bound') for i in range(3)):
            raise ValueError('Invalid group bounds in mm')
    return parts


def corners(bounds):
    return itertools.product(*zip(bounds[:3], bounds[3:]))


def point_bounds(points):
    rows = list(points)
    return [min(p[i] for p in rows) for i in range(3)] + [max(p[i] for p in rows) for i in range(3)]


def posed_bounds(model, parts, pulse_us):
    state = model.state(pulse_us)
    groups = [{'name': group['name'], 'bounds_mm': point_bounds(
        transform_point(p, state['transforms'][group['name']])
        for p in corners(group['bounds_mm']))}
              for group in parts['groups']]
    return {'status': STATUS, 'pulse_us': pulse_us, 'groups': groups,
            'overall_bounds_mm': point_bounds(
                p for group in groups for p in corners(group['bounds_mm']))}


def swept_bounds(model, parts, margin_mm=0.5):
    """Analytic extrema of transformed source boxes over the candidate motion.

    Each coordinate is A*cos(theta)+B*sin(theta)+C. Endpoints and stationary
    points cover the continuous interval, unlike a sampled union. This bounds
    the approximate linkage, not unmodelled motion, flex, or real collisions.
    """
    margin = finite(margin_mm, 'margin_mm')
    if margin < 0:
        raise ValueError('margin_mm must be nonnegative')
    low, high = sorted(model.state(p)['theta_rad']
                       for p in (model.pulse_min_us, model.pulse_max_us))

    def at_theta(theta, name, point):
        t = model._transforms(model.arm_mm * math.sin(theta),
                              model.arm_mm * math.cos(theta), theta)
        return transform_point(point, t[name])
    groups = []
    for group in parts['groups']:
        name = group['name']
        points = []
        for corner in corners(group['bounds_mm']):
            f0, f90, f180 = [at_theta(theta, name, corner) for theta in (0., math.pi/2, math.pi)]
            angles = {low, high}
            for axis in (0, 1):
                constant = (f0[axis] + f180[axis]) / 2
                a, b = (f0[axis] - f180[axis]) / 2, f90[axis] - constant
                base = math.atan2(b, a)
                for k in range(-2, 4):
                    angle = base + k * math.pi
                    if low <= angle <= high:
                        angles.add(angle)
            points.extend(at_theta(theta, name, corner) for theta in angles)
        bounds = point_bounds(points)
        bounds = [v + (-margin if i < 3 else margin) for i, v in enumerate(bounds)]
        groups.append({'name': name, 'bounds_mm': bounds})
    return {'schema_version': 1, 'status': STATUS, 'hardware_verified': False,
            'units': 'mm', 'frame_id': 'gripper_cad_estimate', 'margin_mm': margin,
            'method': ('Analytic coordinate extrema of transformed source AABB corners '
                       'over the continuous candidate angle interval.'),
            'scope': ('Conservative for these source boxes and this ideal motion only; '
                      'not physical collision acceptance or robot attachment.'),
            'groups': groups,
            'overall_bounds_mm': point_bounds(
                p for group in groups for p in corners(group['bounds_mm']))}


def export_estimates(model, directory, parts=None):
    output = Path(directory)
    output.mkdir(parents=True, exist_ok=True)
    pulses = range(math.ceil(model.pulse_min_us), math.floor(model.pulse_max_us)+1, 10)
    states = [model.state(p) for p in pulses]
    if states[-1]['pulse_us'] != model.pulse_max_us:
        states.append(model.state(model.pulse_max_us))
    report = {'model': model.configuration, 'range': model.range_summary(), 'states': states,
              'reference_state': model.reference_state()}
    if parts is not None:
        report['swept_bounds'] = swept_bounds(model, parts)
    (output / 'motion_estimates.json').write_text(
        json.dumps(report, indent=2, allow_nan=False) + '\n')
    with (output / 'predicted_opening.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['pulse_us', 'estimated_opening_mm', 'historical_tcp_extension_mm',
                         'estimated_theta_deg', 'status'])
        writer.writerows([s['pulse_us'], f"{s['opening_mm']:.6f}", f"{s['tcp_extension_mm']:.6f}",
                          f"{s['theta_deg']:.6f}", STATUS] for s in states)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path)
    parser.add_argument('--parts', type=Path)
    actions = parser.add_mutually_exclusive_group()
    actions.add_argument('--pulse-us', type=float)
    actions.add_argument('--opening-mm', type=float)
    parser.add_argument('--export-directory', type=Path)
    args = parser.parse_args(argv)
    try:
        model = MotionModel.from_json(args.model or data_path('gripper_motion_estimate.json'))
        if args.export_directory:
            parts = load_parts(args.parts or data_path('gripper_motion_parts.json'), model)
            export_estimates(model, args.export_directory, parts)
        if args.pulse_us is not None:
            result = model.state(args.pulse_us)
        elif args.opening_mm is not None:
            pulses = model.opening_to_pulses(args.opening_mm)
            result = {'status': STATUS, 'hardware_verified': False, 'opening_mm': args.opening_mm,
                      'candidate_pulses_us': pulses, 'ambiguous': len(pulses) > 1,
                      'reachable_in_candidate': bool(pulses), 'command_sent': False}
        else:
            result = model.range_summary()
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.error(str(error))
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
