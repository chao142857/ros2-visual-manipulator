"""Measurement parsing and numerical comparison, independent of CAD geometry."""

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'gripper_calibration_check', ROOT / 'scripts' / 'gripper_calibration_check.py'
)
checker = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(checker)
HEADER = ','.join(checker.COLUMNS) + '\n'


class ReferenceReadings:
    """A known constant reference isolates measurement arithmetic from fitting."""

    def state(self, pulse):
        if not 600 <= pulse <= 1700:
            raise ValueError('pulse outside reference domain [600, 1700]')
        return {
            'opening_mm': 30.0, 'tcp_extension_mm': 120.0,
            'status': 'approximate_unverified',
        }


def report_for(tmp_path, rows, header=HEADER):
    path = tmp_path / 'measurements.csv'
    path.write_text(header + rows, encoding='utf-8')
    return checker.compare_measurements(path, ReferenceReadings())


def test_signed_residual_statistics_and_measured_hysteresis(tmp_path):
    report = report_for(
        tmp_path, '1300,opening,31,122,first\n1300,closing,29,118,second\n'
    )
    assert report['status'] == 'comparison_only_unverified'
    assert report['physical_travel_verified'] is False
    assert report['rows'][0]['residual_measured_minus_predicted_mm'] == {
        'opening_mm': 1.0, 'tcp_extension_mm': 2.0,
    }
    assert report['residual_summary']['opening_mm'] == {
        'count': 2, 'mean_mm': 0.0, 'max_abs_mm': 1.0, 'rmse_mm': 1.0,
    }
    assert report['residual_summary']['tcp_extension_mm']['rmse_mm'] == 2.0
    assert report['hysteresis_at_matching_pulses'] == [{
        'pulse_us': 1300.0,
        'closing_minus_opening_mm': {'opening_mm': -2.0, 'tcp_extension_mm': -4.0},
        'absolute_difference_mm': {'opening_mm': 2.0, 'tcp_extension_mm': 4.0},
    }]
    assert report['counts']['paired_opening_pulses'] == 1


def test_missing_tcp_stays_unknown_and_different_pulses_are_not_paired(tmp_path):
    report = report_for(
        tmp_path, '1300,opening,34,,same physical pad plane\n'
        '1200,closing,36,,\n1300,closing,,121,\n1400,closing,,,\n'
    )
    assert report['counts']['rows_with_measurements'] == 3
    assert report['counts']['rows_pending'] == 1
    assert report['counts']['opening_missing'] == 2
    assert report['counts']['tcp_extension_missing'] == 3
    assert report['hysteresis_at_matching_pulses'] == []
    assert report['rows'][0]['measured']['tcp_extension_mm'] is None
    assert report['residual_summary']['opening_mm']['mean_mm'] == 5.0
    assert report['residual_summary']['opening_mm']['rmse_mm'] == pytest.approx(26 ** 0.5)


def test_blank_shipped_template_is_pending_and_contains_no_measurements():
    report = checker.compare_measurements(
        ROOT / 'config' / 'gripper_measurements_template.csv', ReferenceReadings()
    )
    assert report['status'] == 'pending_measurements'
    assert report['physical_travel_verified'] is False
    assert report['counts']['rows_total'] == 24
    assert report['counts']['rows_pending'] == 24
    assert report['counts']['opening_measurements'] == 0
    assert report['counts']['tcp_extension_measurements'] == 0
    assert report['residual_summary']['opening_mm']['rmse_mm'] is None
    assert report['hysteresis_at_matching_pulses'] == []


@pytest.mark.parametrize('rows,fragment', [
    ('1300,forward,34,,\n', 'direction'),
    ('1300,opening,nan,,\n', 'finite'),
    ('1300,opening,,inf,\n', 'finite'),
    ('nan,opening,34,,\n', 'finite'),
    ('1300,opening,-1,,\n', 'nonnegative'),
    ('1300,opening,,negative,\n', 'numeric'),
    (',opening,34,,\n', 'required'),
    ('599,opening,,,\n', 'outside reference domain'),
    ('1701,closing,,,\n', 'outside reference domain'),
    ('1300,opening,34,,\n1300.0,opening,,,\n', 'duplicate'),
    ('1300,opening,34,,unquoted,comma\n', 'exactly 5 columns'),
    ('1300,opening,34\n', 'exactly 5 columns'),
    ('1300,opening,34,,"unterminated\n', 'malformed CSV'),
])
def test_invalid_rows_are_rejected_with_source_line(tmp_path, rows, fragment):
    with pytest.raises(checker.MeasurementInputError, match=fragment) as error:
        report_for(tmp_path, rows)
    assert 'line ' in str(error.value)


@pytest.mark.parametrize('header', [
    '', 'pulse_us,direction\n',
    'pulse_us,direction,measured_opening_mm,measured_tcp_extension_mm,notes,notes\n',
    'pulse_us,direction,measured_opening_mm,measured_tcp_extension_mm,other\n',
])
def test_csv_requires_unique_complete_header(tmp_path, header):
    with pytest.raises(checker.MeasurementInputError, match='header'):
        report_for(tmp_path, '', header)


def test_opening_larger_than_model_is_reported_instead_of_clipped(tmp_path):
    report = report_for(tmp_path, '600,opening,75,,\n')
    assert report['rows'][0]['residual_measured_minus_predicted_mm']['opening_mm'] == 45


def test_cli_refuses_to_overwrite_measurement_input(tmp_path, capsys):
    path = tmp_path / 'measurements.csv'
    original = HEADER + '1300,opening,34,,\n'
    path.write_text(original, encoding='utf-8')
    assert checker.main(['--measurements', str(path), '--output', str(path)]) == 2
    assert path.read_text(encoding='utf-8') == original
    assert json.loads(capsys.readouterr().out)['status'] == 'invalid_input'
