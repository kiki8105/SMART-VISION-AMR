"""Confidence lifecycle, data outages and reset invariants without hardware."""

import math
from types import SimpleNamespace

import pytest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

from smart_vision_amr.ekf_sensor_guard import odometry_issues
from smart_vision_amr.localization_health_math import LocalizationHealthCore
from smart_vision_amr.localization_health import LocalizationHealth
from smart_vision_amr.wheel_joint_odometry import WheelIntegrator, load_stock_wheel_geometry
from smart_vision_amr.sensor_stamp_guard import SensorStampGuard


def healthy(**changes):
    return dict({'map_available': True, 'map_tf_age_seconds': 0.05,
                 'odom_tf_valid': True, 'scan_age_seconds': 0.1,
                 'odom_age_seconds': 0.1, 'scan_header_age_seconds': 0.1,
                 'odom_header_age_seconds': 0.1, 'scan_valid': True, 'odom_valid': True,
                 'position_std_m': 0.1, 'yaw_std_rad': 0.1,
                 'moving': False, 'amcl_age_seconds': 100.0, 'amcl_update_epoch': 0.0,
                 'scan_map_agreement': 0.85, 'agreement_age_seconds': 0.1,
                 'pose_jump': False}, **changes)


def armed():
    core = LocalizationHealthCore(stable_seconds=1.0)
    assert core.evaluate(0.0, healthy())['level'] == 'warning'
    assert core.evaluate(1.1, healthy())['level'] == 'ok'
    return core


def test_initial_unknown_pose_allows_deliberate_localization_but_not_autonomous_motion():
    core = LocalizationHealthCore()
    metrics = healthy(map_tf_age_seconds=None, position_std_m=None,
                      yaw_std_rad=None, scan_map_agreement=None)
    for now in (0.0, 10.0, 50.0):
        status = core.evaluate(now, metrics)
        assert status['level'] == 'warning'
        assert status['allow_localization']
        assert not status['autonomous_allowed']
        assert not status['stop_latched']


def test_stream_loss_stops_after_debounce_and_never_resumes_without_reset():
    core = armed()
    lost = healthy(odom_age_seconds=1.5)
    assert core.evaluate(2.0, lost)['level'] == 'warning'
    assert not core.last_result['autonomous_allowed']
    status = core.evaluate(2.6, lost)
    assert status['level'] == 'critical' and not status['allow_localization']
    assert 'odom_unavailable' in status['reasons']
    assert not core.operator_reset(2.6)
    core.evaluate(3.0, healthy())
    assert core.evaluate(4.1, healthy())['level'] == 'critical'
    assert core.operator_reset(4.1)
    assert core.evaluate(4.2, healthy())['autonomous_allowed']


def test_receive_age_half_second_blocks_auto_before_fault_latches():
    core = armed()
    assert core.sensor_timeout == .5
    assert core.evaluate(2.0, healthy(scan_age_seconds=.49))['autonomous_allowed']
    delayed = healthy(scan_age_seconds=.51)
    warning = core.evaluate(2.1, delayed)
    assert warning['level'] == 'warning'
    assert not warning['autonomous_allowed']
    assert not warning['stop_latched']
    assert core.evaluate(2.61, delayed)['stop_latched']


def test_short_dynamic_obstruction_does_not_latch_but_sustained_disagreement_does():
    core = armed()
    blocked = healthy(scan_map_agreement=0.1)
    core.evaluate(2.0, blocked)
    assert core.evaluate(3.0, blocked)['level'] == 'warning'
    assert core.evaluate(3.1, healthy())['level'] == 'ok'
    core.evaluate(4.0, blocked)
    assert core.evaluate(7.1, blocked)['level'] == 'critical'


def test_old_stationary_amcl_pose_allows_first_motion_then_expires_after_moving_time():
    core = armed()
    assert core.evaluate(2.0, healthy(amcl_age_seconds=1000))['level'] == 'ok'
    moving = healthy(moving=True, amcl_age_seconds=1000.)
    assert core.evaluate(2.25, moving)['level'] == 'ok'
    assert core.last_result['autonomous_allowed']
    for now in (2.5, 3., 3.5, 4., 4.5, 5., 5.5, 6., 6.5):
        core.evaluate(now, moving)
    assert 'amcl_not_updating_while_moving' in core.last_result['reasons']
    assert core.evaluate(7.1, moving)['level'] == 'critical'


def test_stop_go_pulses_accumulate_and_only_new_amcl_sample_resets_motion_budget():
    core = armed()
    for index in range(20):
        core.evaluate(2.+index*.5, healthy(moving=index%2 == 0, amcl_age_seconds=1000.))
    assert core.amcl_motion_without_update_seconds >= 4.
    assert core.stop_latched
    prior = core.amcl_motion_without_update_seconds
    core.evaluate(12., healthy(moving=False, amcl_age_seconds=1010.))
    assert core.amcl_motion_without_update_seconds >= prior
    core.evaluate(12.1, healthy(moving=False, amcl_age_seconds=.0, amcl_update_epoch=12.1))
    assert core.amcl_motion_without_update_seconds == 0.
    assert core.stop_latched  # New pose does not automatically release safety stop.


def test_motion_clock_step_is_bounded_and_reinitialization_starts_new_acquisition():
    core = armed()
    core.evaluate(2., healthy(moving=True))
    before = core.amcl_motion_without_update_seconds
    core.evaluate(100., healthy(moving=True))
    assert core.amcl_motion_without_update_seconds-before <= core.motion_dt_limit
    assert core.motion_dt_clamped > 0
    core.reinitialize()
    assert core.amcl_motion_without_update_seconds == 0.
    assert core.last_amcl_update_epoch is None
    assert not core.acquired


def test_plain_operator_reset_does_not_clear_the_motion_update_budget():
    core = armed()
    core.evaluate(2., healthy(moving=True))
    core.evaluate(2.5, healthy(moving=True))
    before = core.amcl_motion_without_update_seconds
    assert before > 0.
    assert core.operator_reset(2.5)
    assert core.amcl_motion_without_update_seconds == before


@pytest.mark.parametrize('limits', [{'motion_dt_limit': 0.}, {'motion_dt_limit': math.nan},
                                    {'amcl_moving_timeout': -1.}])
def test_motion_integration_limits_must_be_finite_and_positive(limits):
    with pytest.raises(ValueError):
        LocalizationHealthCore(**limits)


def test_initialpose_reacquires_without_clearing_critical_stop():
    core = armed()
    core.evaluate(2.0, healthy(pose_jump=True))
    assert core.evaluate(2.6, healthy(pose_jump=True))['stop_latched']
    core.reinitialize()
    assert core.stop_latched and not core.acquired
    core.evaluate(3.0, healthy())
    assert core.evaluate(4.1, healthy())['stop_latched']
    assert core.operator_reset(4.1)


def test_explicit_initialpose_then_reset_can_enable_sweep_without_autonomous_motion():
    core = armed()
    core.evaluate(2.0, healthy(map_tf_age_seconds=None))
    assert core.evaluate(2.6, healthy(map_tf_age_seconds=None))['stop_latched']
    assert not core.operator_reset(2.6)  # Reset alone cannot bypass localization failure.
    core.reinitialize()
    acquiring = healthy(map_tf_age_seconds=None, scan_map_agreement=0.1,
                         position_std_m=0.6)
    core.evaluate(3.0, acquiring)
    assert not core.operator_reset(3.1)
    assert core.evaluate(4.1, acquiring)['stop_latched']
    assert core.operator_reset(4.1)
    status = core.evaluate(4.2, acquiring)
    assert status['level'] == 'warning'
    assert status['allow_localization'] and not status['autonomous_allowed']


@pytest.mark.parametrize('change,reason', [
    ({'position_std_m': 0.9}, 'position_std_m_high'),
    ({'yaw_std_rad': math.nan}, 'amcl_covariance_unavailable'),
    ({'map_tf_age_seconds': 2.0}, 'map_tf_unavailable'),
    ({'odom_tf_valid': False}, 'odom_tf_unavailable'),
    ({'scan_header_age_seconds': -0.5}, 'scan_unavailable'),
])
def test_uncertainty_numeric_errors_and_timestamp_errors_trip(change, reason):
    core = armed()
    core.evaluate(2.0, healthy(**change))
    status = core.evaluate(2.6, healthy(**change))
    assert status['level'] == 'critical'
    assert reason in status['reasons']


def test_warning_covariance_reduces_confidence_without_latching_stop():
    core = armed()
    status = core.evaluate(2.0, healthy(position_std_m=0.4))
    assert status['level'] == 'warning' and status['autonomous_allowed']
    assert not status['stop_latched']


def odom():
    message = Odometry()
    message.header.frame_id = 'odom'
    message.child_frame_id = 'base_footprint'
    message.pose.pose.orientation.w = 1.0
    return message


def test_filter_output_validation_catches_publisher_without_valid_estimate():
    message = odom()
    assert odometry_issues(message) == []
    message.twist.twist.linear.x = math.nan
    assert odometry_issues(message) == ['nonfinite odometry value']
    message = odom()
    message.pose.covariance[0] = 30.0
    assert 'position covariance' in odometry_issues(message, max_position_variance=25.0)[0]
    message = odom()
    message.pose.pose.orientation.w = 0.0
    assert 'quaternion' in odometry_issues(message)[0]
    message = odom()
    message.child_frame_id = 'wrong_base'
    assert 'frames' in odometry_issues(message)[0]


def test_local_absolute_ekf_covariance_growth_is_not_mistaken_for_bad_global_pose():
    message = odom()
    message.pose.covariance[0] = message.pose.covariance[7] = 100.0
    message.pose.covariance[35] = 25.0
    assert odometry_issues(message) == []
    message.twist.covariance[0] = 1.0
    assert 'velocity covariance' in odometry_issues(message)[0]


@pytest.mark.parametrize('change', [{'radius': 0.0}, {'separation': -0.2},
                                    {'max_wheel_speed': math.nan}])
def test_invalid_physical_geometry_is_rejected(change):
    with pytest.raises(ValueError):
        WheelIntegrator(**change)


def test_stock_geometry_preserves_repeated_vendor_wildcard_blocks(tmp_path):
    path = tmp_path/'waffle.yaml'
    text = '''/**:
  turtlebot3_node:
    ros__parameters:
      wheels: {radius: 0.034, separation: 0.29}
/**:
  diff_drive_controller:
    ros__parameters:
      odometry: {use_imu: true}
'''
    path.write_text(text)
    assert load_stock_wheel_geometry(path) == {'wheel_radius': 0.034, 'wheel_separation': 0.29}
    path.write_text(text.replace('radius: 0.034', 'radius: 0.034, radius: 0.1'))
    with pytest.raises(ValueError, match='Ambiguous'):
        load_stock_wheel_geometry(path)


def test_amcl_uses_correlated_principal_uncertainty_and_rejects_nonphysical_covariance():
    node = SimpleNamespace(odom=None, previous_amcl=None, position_std=None,
                           yaw_std=None, amcl_at=None, map_frame='map', last_amcl_stamp_ns=None)
    message = PoseWithCovarianceStamped()
    message.header.frame_id = 'map'
    message.header.stamp.sec = 1
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[0] = message.pose.covariance[7] = 0.09
    message.pose.covariance[1] = message.pose.covariance[6] = 0.08
    message.pose.covariance[35] = 0.01
    LocalizationHealth.on_amcl(node, message)
    assert node.position_std == pytest.approx(math.sqrt(0.17))
    message.pose.covariance[1] = message.pose.covariance[6] = 0.2
    message.header.stamp.sec = 2
    LocalizationHealth.on_amcl(node, message)
    assert node.position_std is None and node.yaw_std is None


def test_duplicate_amcl_header_does_not_refresh_update_epoch():
    node = SimpleNamespace(odom=None, previous_amcl=None, position_std=.1,
                           yaw_std=.1, amcl_at=5., map_frame='map', last_amcl_stamp_ns=1_000_000_000)
    message = PoseWithCovarianceStamped()
    message.header.frame_id = 'map'
    message.header.stamp.sec = 1
    message.pose.pose.orientation.w = 1.
    LocalizationHealth.on_amcl(node, message)
    assert node.amcl_at == 5.
    assert node.last_amcl_stamp_ns == 1_000_000_000


def test_missing_amcl_covariance_requests_bounded_filter_updates_without_motion():
    requests = []
    node = SimpleNamespace(
        no_motion_future=None, position_std=None, scan=object(), map_distance=object(),
        scan_at=10.0, core=SimpleNamespace(sensor_timeout=1.0), last_no_motion_at=None,
        no_motion_client=SimpleNamespace(
            service_is_ready=lambda: True,
            call_async=lambda request: requests.append(request) or SimpleNamespace(done=lambda: True)))
    LocalizationHealth.request_initial_covariance(node, 10.1)
    assert len(requests) == 1
    LocalizationHealth.request_initial_covariance(node, 10.5)
    assert len(requests) == 1
    node.scan_at = 12.2
    LocalizationHealth.request_initial_covariance(node, 12.2)
    assert len(requests) == 2
    node.position_std = 0.1
    node.scan_at = 14.5
    LocalizationHealth.request_initial_covariance(node, 14.5)
    assert len(requests) == 2


def sample_node(monkeypatch):
    clock = {'ros': 10_000_000_000, 'steady': 12.0}
    monkeypatch.setattr('smart_vision_amr.localization_health.time.monotonic', lambda: clock['steady'])
    node = SimpleNamespace(
        base_frame='base_footprint', scan_valid=False, odom_valid=False,
        scan=None, odom=None, scan_at=None, odom_at=None,
        last_received_at={'scan': None, 'odom': None},
        last_received_stamp={'scan': None, 'odom': None},
        rejected_count={'scan': 0, 'odom': 0}, rejection_reason={'scan': '', 'odom': ''},
        last_rejection_reason={'scan': '', 'odom': ''},
        stamps={'scan': SensorStampGuard(), 'odom': SensorStampGuard()},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clock['ros'])))
    node.reject = lambda name, reason: LocalizationHealth.reject(node, name, reason)
    return node, clock


def test_duplicate_odom_keeps_last_good_valid_without_refreshing_its_age(monkeypatch):
    node, clock = sample_node(monkeypatch)
    message = odom()
    message.header.stamp.sec = 10
    LocalizationHealth.on_odom(node, message)
    assert node.odom_valid and node.odom_at == 12.
    clock.update(steady=12.1, ros=10_100_000_000)
    LocalizationHealth.on_odom(node, message)
    assert node.odom_valid and node.odom_at == 12.
    assert node.last_received_at['odom'] == 12.1
    assert node.rejected_count['odom'] == 1
    assert node.rejection_reason['odom'] == 'duplicate_or_reordered_header'
    clock.update(steady=14., ros=12_000_000_000)
    LocalizationHealth.on_odom(node, message)
    core = armed()
    metrics = healthy(odom_age_seconds=2., odom_header_age_seconds=2., odom_valid=node.odom_valid)
    core.evaluate(14., metrics)
    assert core.evaluate(14.6, metrics)['level'] == 'critical'
    assert node.odom_at == 12.  # Replays cannot keep the stream alive.
    message.twist.twist.linear.x = math.nan
    LocalizationHealth.on_odom(node, message)
    assert not node.odom_valid
    assert node.rejection_reason['odom'] == 'nonfinite odometry value'


def test_scan_timestamp_replay_preserves_fresh_good_scan_but_bad_values_invalidate(monkeypatch):
    node, clock = sample_node(monkeypatch)
    message = LaserScan()
    message.header.stamp.sec = 10
    message.range_min, message.range_max = .1, 5.
    message.angle_increment = .01
    message.ranges = [1.]*30
    LocalizationHealth.on_scan(node, message)
    assert node.scan_valid and node.scan_at == 12.
    clock.update(steady=12.1, ros=10_100_000_000)
    LocalizationHealth.on_scan(node, message)
    assert node.scan_valid and node.scan_at == 12.
    assert node.rejection_reason['scan'] == 'duplicate_or_reordered_header'
    message.ranges = [math.nan]*30
    LocalizationHealth.on_scan(node, message)
    assert not node.scan_valid
    assert node.rejection_reason['scan'] == 'invalid_scan_values'
