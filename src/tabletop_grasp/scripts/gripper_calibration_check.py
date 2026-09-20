#!/usr/bin/env python3
"""Compare later manual measurements against the offline gripper estimate.

Uses only the standard library and never publishes ROS or hardware commands.
Residuals are measured minus predicted. No pass/fail tolerance or fitting is
performed: a comparison cannot certify the gripper's physical travel.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = PACKAGE_ROOT / 'config' / 'gripper_motion_estimate.json'
COLUMNS = (
    'pulse_us', 'direction', 'measured_opening_mm',
    'measured_tcp_extension_mm', 'notes',
)


class MeasurementInputError(ValueError):
    """An invalid CSV, with useful source line numbers."""


def _number(value, field, line, optional=False):
    value = value.strip()
    if not value:
        if optional:
            return None
        raise MeasurementInputError(f'line {line}: {field} is required')
    try:
        result = float(value)
    except ValueError as exc:
        raise MeasurementInputError(
            f'line {line}: {field} must be numeric, got {value!r}'
        ) from exc
    if not math.isfinite(result):
        raise MeasurementInputError(f'line {line}: {field} must be finite')
    if result < 0:
        raise MeasurementInputError(f'line {line}: {field} must be nonnegative')
    return result


def _residual_summary(values):
    if not values:
        return {'count': 0, 'mean_mm': None, 'max_abs_mm': None, 'rmse_mm': None}
    # Scaling avoids overflow when squaring large, but finite, input values.
    scale = max(abs(value) for value in values)
    mean = math.fsum(value / len(values) for value in values)
    rmse = (scale * math.sqrt(math.fsum(
        (value / scale) ** 2 for value in values
    ) / len(values))) if scale else 0.0
    return {
        'count': len(values), 'mean_mm': mean,
        'max_abs_mm': scale, 'rmse_mm': rmse,
    }


def compare_measurements(path, model):
    """Validate every CSV row and return an unverified comparison report.

    ``model.state(pulse_us)`` validates the model's pulse range and returns
    opening_mm, tcp_extension_mm, and status. Measured values deliberately
    have no model-derived upper limit: discrepancies must remain visible.
    """
    rows = []
    seen = set()
    with Path(path).open(encoding='utf-8-sig', newline='') as stream:
        reader = csv.DictReader(stream, strict=True)
        try:
            headers = reader.fieldnames
        except csv.Error as exc:
            raise MeasurementInputError(f'malformed CSV header: {exc}') from exc
        if (headers is None or len(headers) != len(COLUMNS)
                or set(headers) != set(COLUMNS)):
            raise MeasurementInputError(
                'CSV header must contain each column exactly once: '
                + ','.join(COLUMNS)
            )
        try:
            for raw in reader:
                line = reader.line_num
                if None in raw or any(value is None for value in raw.values()):
                    raise MeasurementInputError(
                        f'line {line}: expected exactly {len(COLUMNS)} columns'
                    )
                pulse = _number(raw['pulse_us'], 'pulse_us', line)
                direction = raw['direction'].strip()
                if direction not in ('opening', 'closing'):
                    raise MeasurementInputError(
                        f'line {line}: direction must be opening or closing'
                    )
                key = (pulse, direction)
                if key in seen:
                    raise MeasurementInputError(
                        f'line {line}: duplicate pulse/direction: {pulse:g}/{direction}; '
                        'put repeated measurement runs in separate CSV files'
                    )
                seen.add(key)
                measured = {
                    'opening_mm': _number(
                        raw['measured_opening_mm'], 'measured_opening_mm', line, True
                    ),
                    'tcp_extension_mm': _number(
                        raw['measured_tcp_extension_mm'],
                        'measured_tcp_extension_mm', line, True
                    ),
                }
                try:
                    state = model.state(pulse)
                except ValueError as exc:
                    raise MeasurementInputError(f'line {line}: {exc}') from exc
                predicted = {
                    field: state[field]
                    for field in ('opening_mm', 'tcp_extension_mm')
                }
                residual = {
                    field: None if value is None else value - predicted[field]
                    for field, value in measured.items()
                }
                if any(value is not None and not math.isfinite(value)
                       for value in residual.values()):
                    raise MeasurementInputError(
                        f'line {line}: measurement difference is outside numeric range'
                    )
                rows.append({
                    'source_line': line, 'pulse_us': pulse, 'direction': direction,
                    'status': ('measured' if any(value is not None
                               for value in measured.values()) else 'pending_measurement'),
                    'model_status': state['status'], 'predicted': predicted,
                    'measured': measured, 'residual_measured_minus_predicted_mm': residual,
                    'notes': raw['notes'].strip(),
                })
        except csv.Error as exc:
            raise MeasurementInputError(
                f'line {reader.line_num}: malformed CSV: {exc}'
            ) from exc

    fields = ('opening_mm', 'tcp_extension_mm')
    residuals = {
        field: _residual_summary([
            row['residual_measured_minus_predicted_mm'][field] for row in rows
            if row['measured'][field] is not None
        ]) for field in fields
    }
    by_pulse = {}
    for row in rows:
        by_pulse.setdefault(row['pulse_us'], {})[row['direction']] = row
    hysteresis = []
    for pulse, directions in sorted(by_pulse.items()):
        differences = {}
        for field in fields:
            opening_row = directions.get('opening')
            closing_row = directions.get('closing')
            opening = opening_row['measured'][field] if opening_row else None
            closing = closing_row['measured'][field] if closing_row else None
            differences[field] = (
                None if opening is None or closing is None else closing - opening
            )
        if any(value is not None for value in differences.values()):
            hysteresis.append({
                'pulse_us': pulse,
                'closing_minus_opening_mm': differences,
                'absolute_difference_mm': {
                    field: None if value is None else abs(value)
                    for field, value in differences.items()
                },
            })
    measured_rows = sum(row['status'] == 'measured' for row in rows)
    return {
        'status': ('comparison_only_unverified' if measured_rows else 'pending_measurements'),
        'measurements_file': str(Path(path).resolve()),
        'physical_travel_verified': False,
        'message': ('Comparison only; manual review and physical validation are still required.'
                    if measured_rows else 'No measured values; awaiting manual measurements.'),
        'residual_definition': 'measured_minus_predicted',
        'hysteresis_definition': 'measured_closing_minus_measured_opening_at_same_pulse',
        'counts': {
            'rows_total': len(rows), 'rows_with_measurements': measured_rows,
            'rows_pending': len(rows) - measured_rows,
            'opening_measurements': residuals['opening_mm']['count'],
            'opening_missing': len(rows) - residuals['opening_mm']['count'],
            'tcp_extension_measurements': residuals['tcp_extension_mm']['count'],
            'tcp_extension_missing': len(rows) - residuals['tcp_extension_mm']['count'],
            'paired_opening_pulses': sum(
                item['closing_minus_opening_mm']['opening_mm'] is not None
                for item in hysteresis
            ),
            'paired_tcp_extension_pulses': sum(
                item['closing_minus_opening_mm']['tcp_extension_mm'] is not None
                for item in hysteresis
            ),
        },
        'residual_summary': residuals, 'rows': rows,
        'hysteresis_at_matching_pulses': hysteresis,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--measurements', required=True, type=Path,
                        help='Manual measurement CSV; blank measurements are allowed')
    parser.add_argument('--model', type=Path, default=DEFAULT_MODEL,
                        help='Offline model JSON; defaults relative to this script')
    parser.add_argument('--output', type=Path, help='Write JSON report in addition to stdout')
    args = parser.parse_args(argv)
    exit_code = 0
    try:
        if (args.output is not None and args.output.resolve() in
                (args.measurements.resolve(), args.model.resolve())):
            raise MeasurementInputError('output must not overwrite the measurements or model')
        # Direct execution from the source tree; installed ROS Python packages
        # remain importable normally if this directory has no Python package.
        if (PACKAGE_ROOT / 'tabletop_grasp').is_dir():
            sys.path.insert(0, str(PACKAGE_ROOT))
        from tabletop_grasp.gripper_motion import MotionModel
        model = MotionModel.from_json(args.model)
        report = compare_measurements(args.measurements, model)
        report['model_file'] = str(args.model.resolve())
    except (OSError, ValueError, KeyError, TypeError, ImportError) as exc:
        report = {'status': 'invalid_input', 'physical_travel_verified': False,
                  'error': str(exc)}
        exit_code = 2
    rendered = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    if args.output is not None:
        # Never write to either input, including on an invalid-input path.
        if args.output.resolve() not in (args.measurements.resolve(), args.model.resolve()):
            try:
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(rendered, encoding='utf-8')
            except OSError as exc:
                print(f'Cannot write report: {exc}', file=sys.stderr)
                exit_code = 2
    print(rendered, end='')
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
