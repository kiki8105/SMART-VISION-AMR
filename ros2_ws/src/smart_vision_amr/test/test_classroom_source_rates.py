"""Source timestamps, ROS age and wall receipt cadence are separate evidence."""
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[4]
SPEC = importlib.util.spec_from_file_location(
    'classroom_source_rates_test', ROOT/'scripts/capture_classroom_source_rates.py')
rates = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(rates)
NS = 1_000_000_000


def test_source_hz_is_not_wall_receipt_hz_and_never_comes_from_configuration():
    cadence = rates.SourceCadence()
    for index in range(11):
        source = NS+index*100_000_000  # actual source stamps: 10 Hz
        receipt = 50*NS+index*200_000_000  # observed wall receipts: 5 Hz
        cadence.observe(source, receipt, source+30_000_000)
    report = cadence.summary()
    assert report['receipt_sample_count'] == 11
    assert report['source_stamp_hz'] == pytest.approx(10.)
    assert report['receipt_wall_hz'] == pytest.approx(5.)
    assert report['source_stamp_intervals']['p95_seconds'] == pytest.approx(.1)
    assert report['receipt_monotonic_intervals']['p95_seconds'] == pytest.approx(.2)
    assert report['source_age_ros_seconds']['minimum'] == pytest.approx(.03)
    assert report['source_age_ros_seconds']['maximum'] == pytest.approx(.03)


def test_missing_zero_duplicate_and_backward_stamps_do_not_create_source_rate():
    cadence = rates.SourceCadence()
    stamps = [None, 0, NS, NS, 900_000_000, 1_100_000_000]
    for index, stamp in enumerate(stamps):
        cadence.observe(stamp, (10+index)*NS, 2*NS)
    report = cadence.summary()
    assert report['receipt_sample_count'] == 6
    assert report['receipt_wall_hz'] == 1.
    assert report['source_missing_or_invalid_samples'] == 2
    assert report['source_duplicate_samples'] == 1
    assert report['source_backward_samples'] == 1
    assert report['source_forward_samples'] == 2
    assert report['source_stamp_hz'] == pytest.approx(10.)
    assert report['source_first_accepted_stamp_ns'] == NS
    assert report['source_last_accepted_stamp_ns'] == 1_100_000_000
    # A backward sample is neither restamped nor used as a new rate baseline.
    assert report['witness_last_samples'][4]['source_stamp_ns'] == 900_000_000
    assert report['source_age_ros_seconds']['sample_count'] == 4


def test_paused_source_keeps_receipt_count_and_reveals_increasing_age():
    cadence = rates.SourceCadence()
    for index in range(6):
        cadence.observe(NS, (10+index)*NS, (1+index)*NS)
    report = cadence.summary()
    assert report['source_stamp_hz'] is None
    assert report['source_stamp_intervals']['sample_count'] == 0
    assert report['receipt_wall_hz'] == 1.
    assert report['source_duplicate_samples'] == 5
    assert report['source_age_ros_seconds']['minimum'] == 0.
    assert report['source_age_ros_seconds']['maximum'] == 5.


def test_future_age_is_not_clamped_and_missing_ros_clock_is_not_wall_substituted():
    cadence = rates.SourceCadence()
    cadence.observe(2*NS, 10*NS, NS)
    cadence.observe(3*NS, 11*NS, 0)
    report = cadence.summary()
    assert report['source_age_ros_seconds'] == {
        'sample_count': 1, 'minimum': -1., 'maximum': -1.,
        'future_stamp_samples': 1, 'clock_unavailable_samples': 1}


def test_clock_reset_does_not_silently_relabel_new_epoch_as_regular_source_data():
    cadence = rates.SourceCadence()
    for i, stamp in enumerate([100*NS, 101*NS, NS, 2*NS, 102*NS]):
        cadence.observe(stamp, (10+i)*NS, stamp)
    report = cadence.summary()
    assert report['source_backward_samples'] == 2
    assert report['source_forward_samples'] == 3
    assert report['source_stamp_hz'] == 1.
    assert report['source_stamp_intervals']['maximum_seconds'] == 1.


def test_nonincreasing_receipt_clock_is_excluded_and_observation_is_read_only():
    cadence = rates.SourceCadence()
    for source, receipt in [(NS, 5*NS), (2*NS, 5*NS), (3*NS, 4*NS), (4*NS, 6*NS)]:
        cadence.observe(source, receipt, source)
    report = cadence.summary()
    assert report['receipt_sample_count'] == 4
    assert report['receipt_valid_clock_samples'] == 2
    assert report['receipt_nonincreasing_clock_samples'] == 2
    assert report['receipt_wall_hz'] == 1.
    assert report['source_stamp_hz'] == 1.
    assert cadence.summary() == report  # writing reports cannot advance the measurement


def test_empty_and_single_sample_never_invent_configured_hz_or_zero_interval():
    cadence = rates.SourceCadence()
    for report in [cadence.summary(), None]:
        if report is None:
            cadence.observe(NS, 5*NS, NS)
            report = cadence.summary()
        assert report['source_stamp_hz'] is None
        assert report['receipt_wall_hz'] is None
        assert report['source_stamp_intervals']['maximum_seconds'] is None


def test_nearest_rank_p95_includes_actual_long_interval():
    report = rates.interval_metrics([.05]*18+[.20, .70])
    assert report['p95_seconds'] == .20
    assert report['maximum_seconds'] == .70


@pytest.mark.parametrize('domain,environment', [
    (16, {'ROS_DOMAIN_ID': '16', 'ROS_LOCALHOST_ONLY': '1'}),
    (176, {'ROS_DOMAIN_ID': '176', 'ROS_LOCALHOST_ONLY': '0'}),
    (176, {'ROS_DOMAIN_ID': '176'}),
    (176, {'ROS_DOMAIN_ID': '16', 'ROS_LOCALHOST_ONLY': '1'}),
    (176, {'ROS_DOMAIN_ID': '177', 'ROS_LOCALHOST_ONLY': '1'}),
    (233, {'ROS_DOMAIN_ID': '233', 'ROS_LOCALHOST_ONLY': '1'}),
])
def test_physical_nonlocal_or_mismatched_domain_is_refused(domain, environment):
    with pytest.raises(ValueError):
        rates.require_isolated_domain(domain, environment)


def test_isolated_topics_select_actual_ekf_producer_only_when_opted_in():
    rates.require_isolated_domain(176, {'ROS_DOMAIN_ID': '176', 'ROS_LOCALHOST_ONLY': '1'})
    nominal = rates.selected_topics()
    delayed = rates.selected_topics(True)
    assert nominal == {'raw_scan': '/classroom_fault/raw_scan', 'raw_imu': '/imu',
                       'wheel_joint_states': '/joint_states', 'ekf_output': '/odom'}
    assert delayed['ekf_output'] == '/classroom_fault/raw_odom'
    assert {key: val for key, val in delayed.items() if key != 'ekf_output'} == {
        key: val for key, val in nominal.items() if key != 'ekf_output'}


@pytest.mark.parametrize('sec,nanosec,expected', [
    (0, 0, None), (1, 20, NS+20), (-1, 0, None), (1, NS, None),
    (True, 0, None), (1, -.1, None),
])
def test_header_stamp_validates_original_without_restamping(sec, nanosec, expected):
    message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)))
    assert rates.header_stamp_ns(message) == expected
    assert (message.header.stamp.sec, message.header.stamp.nanosec) == (sec, nanosec)


def test_malformed_stamp_witness_remains_strict_json_serializable():
    cadence = rates.SourceCadence()
    cadence.observe(float('nan'), float('inf'), None)
    report = cadence.summary()
    assert report['source_missing_or_invalid_samples'] == 1
    assert report['receipt_invalid_clock_samples'] == 1
    json.dumps(report, allow_nan=False)
