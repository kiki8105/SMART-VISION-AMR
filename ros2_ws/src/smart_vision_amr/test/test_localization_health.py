"""Confidence lifecycle, data outages and reset invariants without hardware."""

import json
import math
import os
import time
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest
import rclpy
import yaml
from rclpy.context import Context
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from smart_vision_amr.ekf_sensor_guard import odometry_issues
from smart_vision_amr.localization_health_math import LocalizationHealthCore
from smart_vision_amr.localization_health import LocalizationHealth
from smart_vision_amr.motion_mux import MotionMuxCore
from smart_vision_amr.wheel_joint_odometry import WheelIntegrator, load_stock_wheel_geometry
from smart_vision_amr.sensor_stamp_guard import SensorStampGuard


def test_localization_reliable_observer_retains_reset_before_queued_holds():
    # Reset is published once by the reliable mux service. A later hold must
    # not replace it while the health executor is busy processing a scan.
    context = Context()
    with patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}):
        rclpy.init(args=[], context=context, domain_id=232,
                   signal_handler_options=SignalHandlerOptions.NO)
    health = LocalizationHealth(context=context)
    probe = Node('localization_control_qos_probe', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(health)
    executor.add_node(probe)
    service_control = probe.create_publisher(String, '/smart_vision_amr/manual_control', 10)
    for timer in health.timers:
        timer.cancel()
    try:
        subscriptions = [item for item in health.subscriptions
                         if item.topic_name == '/smart_vision_amr/manual_control']
        assert len(subscriptions) == 1
        qos = subscriptions[0].qos_profile
        assert qos.reliability == ReliabilityPolicy.RELIABLE
        assert qos.durability == DurabilityPolicy.VOLATILE
        assert qos.depth == 10
        deadline = time.monotonic()+3.
        while service_control.get_subscription_count() == 0 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert service_control.get_subscription_count() > 0
        now = time.monotonic()
        health.core.evaluate(now-3., healthy())
        health.core.evaluate(now-.1, healthy())
        health.core.stop_latched = True
        assert health.core.evaluate(now, healthy())['reasons'] == ['operator_reset_required']
        source = 'mux-service:qos-test'
        for seq in range(1, 10):
            service_control.publish(String(data=json.dumps({
                'source_id': source, 'seq': seq, 'mode': 'reset' if seq == 1 else 'hold',
                'sent_at_ns': probe.get_clock().now().nanoseconds})))
        time.sleep(.05)  # DDS receives the burst before the observer takes it.
        deadline = time.monotonic()+.7
        while health.last_reset_seq.get(source) != 1 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert health.last_reset_seq.get(source) == 1
        assert not health.core.stop_latched
        assert health.core.evaluate(time.monotonic(), healthy())['level'] == 'ok'
    finally:
        executor.shutdown()
        probe.destroy_node()
        health.destroy_node()
        rclpy.shutdown(context=context)


def test_localization_reset_observer_keeps_source_stamp_and_sequence_guards():
    class FakeCore:
        stop_latched = True
        reset_calls = 0

        def operator_reset(self, _now):
            self.reset_calls += 1
            self.stop_latched = False
            return True

    node = SimpleNamespace(core=FakeCore(), last_reset_seq={},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=20_000_000_000)),
        publish_estop=lambda _now: pytest.fail('Valid reset unexpectedly rejected'))

    def packet(source, seq, stamp):
        return String(data=json.dumps({'source_id': source, 'seq': seq, 'mode': 'reset',
                                       'sent_at_ns': stamp}))

    queue = [packet('localization_safety', 1, 20_000_000_000),
             packet('mux-service:queued', 1, 18_900_000_000),
             packet('mux-service:queued', 1, 20_000_000_000)]
    for message in queue:
        LocalizationHealth.on_control(node, message)
    assert node.core.reset_calls == 1
    node.core.stop_latched = True
    LocalizationHealth.on_control(node, queue[-1])
    assert node.core.stop_latched and node.core.reset_calls == 1  # Replay cannot clear a later latch.
    LocalizationHealth.on_control(node, packet('mux-service:queued', 2, 21_100_000_000))
    assert node.core.stop_latched and node.core.reset_calls == 1  # Future packet is also rejected.


def healthy(**changes):
    return dict({'map_available': True, 'map_tf_age_seconds': 0.05,
                 'odom_tf_valid': True, 'scan_age_seconds': 0.1,
                 'odom_age_seconds': 0.1, 'scan_header_age_seconds': 0.1,
                 'odom_header_age_seconds': 0.1, 'scan_valid': True, 'odom_valid': True,
                 'position_std_m': 0.1, 'yaw_std_rad': 0.1,
                 'moving': False, 'amcl_age_seconds': 0.1,
                 'amcl_header_age_seconds': 0.1, 'amcl_update_epoch': 0.0,
                 'scan_map_agreement': 0.85, 'agreement_age_seconds': 0.1,
                 'pose_jump': False}, **changes)


def armed():
    core = LocalizationHealthCore(stable_seconds=1.0)
    assert core.evaluate(0.0, healthy())['level'] == 'warning'
    assert core.evaluate(1.1, healthy())['level'] == 'ok'
    return core


def test_mapping_mode_ignores_changing_map_and_missing_amcl_but_never_allows_auto():
    core = LocalizationHealthCore(stable_seconds=1.0, fault_seconds=.5,
                                  mapping_mode=True)
    changing_map = healthy(map_available=False, map_tf_age_seconds=None,
                           position_std_m=None, yaw_std_rad=None,
                           scan_map_agreement=None, agreement_age_seconds=None,
                           pose_jump=True, moving=True)
    for now in (0., 1.1, 5., 10.):
        status = core.evaluate(now, changing_map)
        assert status['level'] == 'warning'
        assert status['mapping_mode'] is True
        assert status['reasons'] == ['mapping_mode']
        assert status['allow_localization'] is True
        assert status['autonomous_allowed'] is False
        assert not status['stop_latched']
    assert not core.acquired


@pytest.mark.parametrize('fault', [
    {'scan_age_seconds': 2.}, {'odom_header_age_seconds': 2.},
    {'odom_tf_valid': False},
])
def test_mapping_mode_stops_manual_on_real_sensor_loss_and_requires_reset(fault):
    core = LocalizationHealthCore(stable_seconds=1.0, fault_seconds=.5,
                                  mapping_mode=True)
    core.evaluate(0., healthy())
    core.evaluate(1.1, healthy())
    bad = healthy(**fault)
    blocked = core.evaluate(2., bad)
    assert blocked['level'] == 'critical'
    assert blocked['allow_localization'] is False
    assert blocked['autonomous_allowed'] is False
    assert not blocked['stop_latched']
    latched = core.evaluate(2.6, bad)
    assert latched['stop_latched']
    assert not core.operator_reset(2.6)
    core.evaluate(3., healthy())
    assert core.evaluate(4.1, healthy())['stop_latched']
    assert core.operator_reset(4.1)
    resumed = core.evaluate(4.2, healthy())
    assert resumed['level'] == 'warning'
    assert not resumed['stop_latched']
    assert not resumed['autonomous_allowed']


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


def test_physical_sensor_jitter_margin_is_bounded_below_independent_guard():
    package = Path(__file__).resolve().parents[1]
    settings = yaml.safe_load((package/'config/localization_health.yaml').read_text())[
        'localization_health']['ros__parameters']
    guard = yaml.safe_load((package/'config/ekf_hardware.yaml').read_text())[
        'ekf_sensor_guard']['ros__parameters']
    assert settings['sensor_timeout'] == pytest.approx(.7)
    assert settings['sensor_timeout'] < guard['stale_seconds'] == 1.5

    core = LocalizationHealthCore(stable_seconds=1.0,
                                  sensor_timeout=settings['sensor_timeout'])
    core.evaluate(0., healthy())
    core.evaluate(1.1, healthy())
    assert core.evaluate(2., healthy(scan_age_seconds=.6,
                                     scan_header_age_seconds=.6,
                                     odom_age_seconds=.6,
                                     odom_header_age_seconds=.6))['autonomous_allowed']
    delayed = core.evaluate(2.1, healthy(scan_age_seconds=.71))
    assert not delayed['autonomous_allowed']
    assert not delayed['stop_latched']


@pytest.mark.parametrize('spatial_fault', [
    {'map_tf_age_seconds': None}, {'odom_tf_valid': False},
    {'scan_map_agreement': None},
    {'map_tf_age_seconds': None, 'odom_tf_valid': False,
     'scan_map_agreement': None},
])
def test_short_spatial_gap_holds_zero_then_resumes_without_reset(spatial_fault):
    package = Path(__file__).resolve().parents[1]
    settings = yaml.safe_load((package/'config/localization_health.yaml').read_text())[
        'localization_health']['ros__parameters']
    assert settings['fault_seconds'] == pytest.approx(.5)
    assert settings['spatial_fault_seconds'] == pytest.approx(1.25)
    core = LocalizationHealthCore(stable_seconds=1.,
                                  fault_seconds=settings['fault_seconds'],
                                  spatial_fault_seconds=settings['spatial_fault_seconds'])
    core.evaluate(0., healthy())
    assert core.evaluate(1.1, healthy())['autonomous_allowed']
    clock = [1.1]
    mux = MotionMuxCore(clock=lambda: clock[0], require_localization_health=True)
    mux.receive_localization_health(core.last_result)
    assert mux.receive_auto(.1, 0.)
    assert mux.output() == pytest.approx((.1, 0.))

    clock[0] = 2.
    blocked = core.evaluate(2., healthy(**spatial_fault))
    assert blocked['level'] == 'warning'
    assert not blocked['autonomous_allowed'] and not blocked['stop_latched']
    mux.receive_localization_health(blocked)
    assert mux.receive_auto(.1, 0.)
    assert mux.output() == (0., 0.)
    clock[0] = 2.9
    blocked = core.evaluate(2.9, healthy(**spatial_fault))
    assert not blocked['autonomous_allowed'] and not blocked['stop_latched']
    mux.receive_localization_health(blocked)
    assert mux.receive_auto(.1, 0.)
    assert mux.output() == (0., 0.)

    clock[0] = 3.0
    recovered = core.evaluate(3., healthy())
    assert recovered['autonomous_allowed'] and not recovered['stop_latched']
    assert recovered['level'] == 'ok'
    mux.receive_localization_health(recovered)
    assert mux.receive_auto(.1, 0.)
    assert mux.output() == pytest.approx((.1, 0.))


@pytest.mark.parametrize('fault,expected', [
    ({'map_tf_age_seconds': None}, 'map_tf_unavailable'),
    ({'odom_tf_valid': False}, 'odom_tf_unavailable'),
    ({'scan_map_agreement': None}, 'scan_map_check_unavailable'),
])
def test_sustained_spatial_gap_latches_and_needs_operator_reset(fault, expected):
    core = armed()
    core.evaluate(2., healthy(**fault))
    warning = core.evaluate(3.2, healthy(**fault))
    assert not warning['stop_latched'] and not warning['autonomous_allowed']
    latched = core.evaluate(3.3, healthy(**fault))
    assert latched['level'] == 'critical' and latched['stop_latched']
    assert expected in latched['reasons']
    assert not core.operator_reset(3.3)
    assert not core.evaluate(3.4, healthy())['autonomous_allowed']


@pytest.mark.parametrize('fault,expected', [
    ({'scan_age_seconds': 2.}, 'scan_unavailable'),
    ({'odom_age_seconds': 2.}, 'odom_unavailable'),
    ({'pose_jump': True}, 'localization_jump'),
])
def test_sensor_or_pose_fault_keeps_fast_latch(fault, expected):
    core = armed()
    core.evaluate(2., healthy(**fault))
    latched = core.evaluate(2.6, healthy(**fault))
    assert latched['stop_latched'] and not latched['autonomous_allowed']
    assert expected in latched['reasons']


def test_even_zero_finite_scan_map_match_is_advisory_without_waiving_live_inputs():
    core = LocalizationHealthCore(stable_seconds=1.0)
    blocked = healthy(scan_map_agreement=0.0)
    assert not core.evaluate(0.0, blocked)['autonomous_allowed']
    ready = core.evaluate(1.1, blocked)
    assert ready['level'] == 'ok' and ready['autonomous_allowed']
    assert ready['advisories'] == ['scan_map_disagreement']
    assert not core.evaluate(7.1, blocked)['stop_latched']
    low = core.evaluate(7.2, healthy(scan_map_agreement=0.4))
    assert low['advisories'] == ['scan_map_agreement_low']
    stale = core.evaluate(7.3, healthy(scan_age_seconds=2.0))
    assert not stale['autonomous_allowed']
    assert 'scan_unavailable' in stale['reasons']


def test_unchanged_amcl_epoch_expires_after_moving_time_even_with_fresh_age_metrics():
    core = armed()
    assert core.evaluate(2.0, healthy())['level'] == 'ok'
    moving = healthy(moving=True)
    assert core.evaluate(2.25, moving)['level'] == 'ok'
    assert core.last_result['autonomous_allowed']
    for now in (2.5, 3., 3.5, 4., 4.5, 5., 5.5, 6., 6.5):
        core.evaluate(now, moving)
    assert 'amcl_not_updating_while_moving' in core.last_result['reasons']
    assert core.evaluate(7.1, moving)['level'] == 'critical'


def test_stop_go_pulses_accumulate_and_only_new_amcl_sample_resets_motion_budget():
    core = armed()
    for index in range(20):
        core.evaluate(2.+index*.5, healthy(moving=index%2 == 0))
    assert core.amcl_motion_without_update_seconds >= 4.
    assert core.stop_latched
    prior = core.amcl_motion_without_update_seconds
    core.evaluate(12., healthy(moving=False))
    assert core.amcl_motion_without_update_seconds >= prior
    core.evaluate(12.1, healthy(moving=False, amcl_age_seconds=.0, amcl_update_epoch=12.1))
    assert core.amcl_motion_without_update_seconds == 0.
    assert core.stop_latched  # New pose does not automatically release safety stop.


@pytest.mark.parametrize('age_field', ['amcl_age_seconds', 'amcl_header_age_seconds'])
def test_stale_amcl_posterior_blocks_autonomous_even_when_covariance_is_low(age_field):
    core = armed()
    stale = healthy(**{age_field: core.amcl_pose_timeout+.1})
    first = core.evaluate(2., stale)
    assert not first['autonomous_allowed'] and not first['stop_latched']
    assert 'amcl_pose_stale' in first['reasons']
    latched = core.evaluate(2.6, stale)
    assert latched['stop_latched']
    assert 'amcl_pose_stale' in latched['reasons']


def test_manual_reacquisition_cannot_arm_from_old_amcl_covariance():
    core = armed()
    core.reinitialize()
    stale = healthy(amcl_age_seconds=9., amcl_header_age_seconds=9.)
    for now in (2., 4.5, 10.):
        result = core.evaluate(now, stale)
        assert not result['autonomous_allowed'] and not core.acquired
    fresh = healthy(amcl_age_seconds=.1, amcl_header_age_seconds=.1,
                    amcl_update_epoch=10.1)
    assert not core.evaluate(10.1, fresh)['autonomous_allowed']
    assert core.evaluate(12.2, fresh)['autonomous_allowed']


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
                                    {'amcl_moving_timeout': -1.},
                                    {'spatial_fault_seconds': 0.},
                                    {'spatial_fault_seconds': math.nan}])
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
    assert core.evaluate(3.3, healthy(map_tf_age_seconds=None))['stop_latched']
    assert not core.operator_reset(3.3)  # Reset alone cannot bypass localization failure.
    core.reinitialize()
    acquiring = healthy(map_tf_age_seconds=None, scan_map_agreement=0.1,
                         position_std_m=0.6)
    core.evaluate(3.4, acquiring)
    assert not core.operator_reset(3.5)
    assert core.evaluate(4.5, acquiring)['stop_latched']
    assert core.operator_reset(4.5)
    status = core.evaluate(4.6, acquiring)
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
    status = core.evaluate(3.3 if reason in {'map_tf_unavailable', 'odom_tf_unavailable'}
                           else 2.6, healthy(**change))
    assert status['level'] == 'critical'
    assert reason in status['reasons']


def test_warning_covariance_reduces_confidence_without_latching_stop():
    core = armed()
    status = core.evaluate(2.0, healthy(position_std_m=0.4))
    assert status['level'] == 'warning' and status['autonomous_allowed']
    assert not status['stop_latched']


def test_testbed_0446m_amcl_uncertainty_acquires_without_bypassing_critical_limit():
    core = LocalizationHealthCore()
    first = core.evaluate(0.0, healthy(position_std_m=.446))
    assert not first['autonomous_allowed']
    ready = core.evaluate(2.01, healthy(position_std_m=.446))
    assert ready['level'] == 'warning' and ready['autonomous_allowed']
    assert 'position_std_m_elevated' in ready['reasons']
    assert core.position_warning == .35 and core.position_acquisition_limit == .50
    assert core.position_critical == .8
    mux = MotionMuxCore(clock=lambda: 2.01, require_localization_health=True)
    assert mux.receive_localization_health(ready)
    assert mux.receive_auto(.2, 0.)
    assert mux.output() == pytest.approx((.1, 0.))
    fault = core.evaluate(2.02, healthy(position_std_m=.8))
    assert 'position_std_m_high' in fault['reasons']
    assert not fault['autonomous_allowed']


def test_position_over_testbed_acquisition_limit_and_other_warnings_still_wait():
    for sample in (healthy(position_std_m=.501),
                   healthy(position_std_m=.446, yaw_std_rad=.6)):
        core = LocalizationHealthCore()
        core.evaluate(0., sample)
        status = core.evaluate(2.01, sample)
        assert not status['autonomous_allowed'] and not core.acquired


def test_bounded_position_warning_can_acquire_with_low_finite_scan_map_match():
    core = LocalizationHealthCore()
    sample = healthy(position_std_m=.446, scan_map_agreement=.4)
    core.evaluate(0., sample)
    status = core.evaluate(2.01, sample)
    assert status['autonomous_allowed'] and core.acquired
    assert status['level'] == 'warning'
    assert status['advisories'] == ['scan_map_agreement_low']


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


def manual_pose_confirmation_fixture(monkeypatch):
    monkeypatch.setattr('smart_vision_amr.localization_health.time.monotonic', lambda: 100.)
    ros_ns = 10_000_000_000
    scan = LaserScan()
    scan.header.stamp.sec, scan.header.stamp.nanosec = 9, 800_000_000
    robot_odom = Odometry()
    robot_odom.header.stamp.sec, robot_odom.header.stamp.nanosec = 9, 800_000_000
    core = LocalizationHealthCore()
    core.acquired = core.stop_latched = True

    def lookup(target, source):
        return SimpleNamespace(
            header=SimpleNamespace(frame_id=target, stamp=SimpleNamespace(
                sec=9, nanosec=900_000_000)), child_frame_id=source,
            transform=SimpleNamespace(translation=SimpleNamespace(x=0., y=0.),
                                      rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))

    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=ros_ns)),
        core=core, map_frame='map', odom_frame='odom', base_frame='base_footprint',
        _tf=lookup, map_distance=object(), scan=scan, odom=robot_odom,
        scan_valid=True, odom_valid=True, scan_at=99.8, odom_at=99.8,
        last_amcl_stamp_ns=9_500_000_000, amcl_at=99.8,
        position_std=.4, yaw_std=.6, previous_amcl=object(), jump_until=101.,
        agreement=.8, agreement_at=99.8, agreement_stamp=9_800_000_000,
        agreement_scan=scan, agreement_scan_at=99.8, agreement_pending=deque([scan]),
        last_manual_confirmed_stamp_ns=0, last_manual_confirmed_amcl_stamp_ns=0)
    request = SetParametersAtomically.Request()
    request.parameters = [Parameter(name=name, value=ParameterValue(
        type=ParameterType.PARAMETER_INTEGER, integer_value=value))
        for name, value in (('initial_pose_stamp_ns', 9_000_000_000),
                            ('verified_amcl_stamp_ns', 9_500_000_000))]
    return node, request


def test_manual_service_confirmation_requires_post_seed_amcl_and_keeps_estop(monkeypatch):
    node, request = manual_pose_confirmation_fixture(monkeypatch)
    node.last_amcl_stamp_ns = 8_900_000_000
    rejected = LocalizationHealth.confirm_manual_pose(
        node, request, SetParametersAtomically.Response())
    assert not rejected.result.successful
    assert rejected.result.reason == 'amcl_not_seen_yet'
    assert node.core.acquired and node.core.stop_latched
    node.last_amcl_stamp_ns = 9_500_000_000
    accepted = LocalizationHealth.confirm_manual_pose(
        node, request, SetParametersAtomically.Response())
    assert accepted.result.successful
    assert not node.core.acquired and node.core.reinitialization_requested
    assert node.core.stop_latched  # Confirmation is never an ESTOP reset.
    assert node.position_std == .4 and node.yaw_std == .6 and node.amcl_at == 99.8
    assert node.agreement is None and not node.agreement_pending
    # Simulate a lost successful service response. Retrying the same pair
    # must acknowledge it without reinitializing the health state again.
    node.core.acquired = True
    previous_amcl = node.previous_amcl = object()
    duplicate = LocalizationHealth.confirm_manual_pose(
        node, request, SetParametersAtomically.Response())
    assert duplicate.result.successful
    assert duplicate.result.reason == 'already_confirmed_stop_latch_retained'
    assert node.core.acquired and node.core.stop_latched
    assert node.previous_amcl is previous_amcl
    request.parameters[1].value.integer_value += 1
    conflict = LocalizationHealth.confirm_manual_pose(
        node, request, SetParametersAtomically.Response())
    assert not conflict.result.successful
    assert conflict.result.reason == 'invalid_or_stale_stamp'


def test_stopped_manual_pose_accepts_bounded_amcl_age_only_with_new_scan_map_proof(monkeypatch):
    node, request = manual_pose_confirmation_fixture(monkeypatch)
    # AMCL can remain motion-gated while stopped. Its latest posterior is
    # four seconds old, but the lidar, odometry and map TF are current.
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=14_000_000_000))
    node.amcl_at = 96.
    node.scan.header.stamp.sec = node.odom.header.stamp.sec = 13
    node.scan.header.stamp.nanosec = node.odom.header.stamp.nanosec = 800_000_000
    node.agreement_stamp = 13_800_000_000
    node.agreement_at = 99.8

    def lookup(target, source):
        return SimpleNamespace(
            header=SimpleNamespace(frame_id=target, stamp=SimpleNamespace(
                sec=13, nanosec=900_000_000)), child_frame_id=source,
            transform=SimpleNamespace(translation=SimpleNamespace(x=0., y=0.),
                                      rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))

    node._tf = lookup
    node.agreement_at = 98.
    rejected = LocalizationHealth.confirm_manual_pose(
        node, request, SetParametersAtomically.Response())
    assert not rejected.result.successful and rejected.result.reason == 'scan_map_not_fresh'
    assert node.core.stop_latched and node.core.acquired

    node.agreement_at = 99.8
    node.odom.twist.twist.linear.x = .02
    moving = LocalizationHealth.confirm_manual_pose(
        node, request, SetParametersAtomically.Response())
    assert not moving.result.successful and moving.result.reason == 'robot_not_stopped'
    node.odom.twist.twist.linear.x = 0.

    accepted = LocalizationHealth.confirm_manual_pose(
        node, request, SetParametersAtomically.Response())
    assert accepted.result.successful
    assert not node.core.acquired and node.core.reinitialization_requested
    assert node.core.stop_latched


def test_manual_pose_age_never_exceeds_health_watchdog(monkeypatch):
    node, request = manual_pose_confirmation_fixture(monkeypatch)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=18_000_000_000))
    rejected = LocalizationHealth.confirm_manual_pose(
        node, request, SetParametersAtomically.Response())
    assert not rejected.result.successful
    assert rejected.result.reason == 'invalid_or_stale_stamp'
    assert node.core.acquired and node.core.stop_latched


@pytest.mark.parametrize('change,reason', [
    (lambda node: setattr(node, 'scan_at', 98.), 'sensor_not_fresh'),
    (lambda node: setattr(node, 'map_distance', None), 'sensor_not_fresh'),
    (lambda node: setattr(node.odom.twist.twist.angular, 'z', .1), 'robot_not_stopped'),
])
def test_manual_confirmation_rejects_missing_stopped_sensor_proof(monkeypatch, change, reason):
    node, request = manual_pose_confirmation_fixture(monkeypatch)
    change(node)
    rejected = LocalizationHealth.confirm_manual_pose(
        node, request, SetParametersAtomically.Response())
    assert not rejected.result.successful and rejected.result.reason == reason
    assert node.core.acquired and node.core.stop_latched


def test_missing_amcl_covariance_requests_bounded_filter_updates_without_motion():
    requests = []
    node = SimpleNamespace(
        no_motion_future=None, position_std=None, yaw_std=.1, scan=LaserScan(), map_distance=object(),
        scan_at=10.0, core=LocalizationHealthCore(sensor_timeout=1.0), last_no_motion_at=None,
        no_motion_client=SimpleNamespace(
            service_is_ready=lambda: True,
            call_async=lambda request: requests.append(request) or SimpleNamespace(done=lambda: True)))
    node.scan.header.stamp.sec = 10
    LocalizationHealth.request_initial_covariance(node, 10.1)
    assert len(requests) == 1
    LocalizationHealth.request_initial_covariance(node, 10.5)
    assert len(requests) == 1
    node.scan_at = 12.2
    node.scan.header.stamp.sec = 12
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
