"""Visual marker rotation cannot use the unlocalized/manual safety exception."""

import math

import pytest

from smart_vision_amr.motion_mux import MotionMuxCore


class Harness:
    def __init__(self, **parameters):
        self.now, self.sequence = 10., 0
        self.mux = MotionMuxCore(clock=lambda: self.now, **parameters)

    def command(self, operation='marker_alignment', mode='manual', **velocity):
        self.sequence += 1
        return self.mux.receive_control({
            'source_id': 'web-existing-source', 'seq': self.sequence,
            'sent_at_ns': 20_000_000_000, 'mode': mode,
            'operation': operation, 'linear': 0., 'angular': .12, **velocity,
        }, 20_000_000_000)

    def health(self, level='healthy', allowed=True, allow_localization=False):
        assert self.mux.receive_localization_health({
            'level': level, 'autonomous_allowed': allowed,
            'allow_localization': allow_localization,
        })


def test_marker_operation_requires_health_even_in_compatibility_runtime():
    harness = Harness(require_localization_health=False)
    assert harness.command()
    assert harness.mux.output() == (0., 0.)


def test_unlocalized_warning_cannot_reuse_bootstrap_rotation_permission():
    harness = Harness()
    harness.health(level='warning', allowed=False, allow_localization=True)
    assert harness.command()
    assert harness.mux.output() == (0., 0.)
    # The deliberate localization workflow retains its separate exception.
    assert harness.command(operation='localization')
    assert harness.mux.output() == (0., .12)


def test_health_report_must_explicitly_allow_autonomous_operation():
    harness = Harness()
    assert harness.mux.receive_localization_health({'level': 'healthy'})
    assert harness.command()
    assert harness.mux.output() == (0., 0.)


def test_marker_operation_never_translates():
    harness = Harness()
    harness.health()
    assert harness.command(linear=.04, angular=.1)
    assert harness.mux.output() == (0., 0.)


def test_marker_rotation_limit_and_warning_factor_both_apply():
    harness = Harness()
    harness.health()
    for desired, expected in ((.8, .15), (-.8, -.15), (.03, .03)):
        assert harness.command(angular=desired)
        assert harness.mux.output() == (0., expected)
    harness.health(level='warning', allowed=True)
    assert harness.command(angular=.8)
    assert harness.mux.output() == (0., .075)


def test_health_expiry_stops_rotation_before_control_lease_expires():
    harness = Harness(localization_health_timeout=.25)
    harness.health()
    assert harness.command()
    assert harness.mux.output() == (0., .12)
    harness.now += .3
    assert harness.mux.mode == 'manual'
    assert harness.mux.output() == (0., 0.)
    harness.health(level='critical', allowed=True)
    assert harness.command()
    assert harness.mux.output() == (0., 0.)


def test_ordinary_manual_and_same_source_joystick_remain_unchanged():
    harness = Harness()
    assert harness.command(operation='drive', linear=.1, angular=-.3)
    assert harness.mux.output() == (.1, -.3)
    harness.health()
    assert harness.command()
    assert harness.mux.output() == (0., .12)
    assert harness.command(operation='drive', linear=-.08, angular=.2)
    assert harness.mux.output() == (-.08, .2)
    assert harness.mux.source_id == 'web-existing-source'


def test_common_stop_reset_and_deadman_still_take_precedence():
    harness = Harness()
    harness.health()
    assert harness.command()
    harness.now += .51
    assert harness.mux.output() == (0., 0.)
    assert harness.mux.mode == 'hold'
    assert harness.command(mode='estop')
    assert not harness.command()
    assert harness.mux.output() == (0., 0.)
    assert harness.command(mode='reset')
    assert harness.mux.mode == 'hold'
    assert harness.mux.output() == (0., 0.)


@pytest.mark.parametrize('limit', [0., -.1, math.nan, math.inf, 1.1])
def test_invalid_marker_rotation_limit_is_rejected(limit):
    with pytest.raises(ValueError):
        MotionMuxCore(marker_alignment_rotation_limit=limit)


def test_node_exposes_marker_rotation_limit_on_an_isolated_domain(monkeypatch):
    import rclpy
    from rclpy.context import Context
    from smart_vision_amr.motion_mux import MotionMuxNode

    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = Context()
    rclpy.init(context=context, domain_id=222)
    node = None
    try:
        node = MotionMuxNode(context=context)
        assert node.get_parameter('marker_alignment_rotation_limit').value == .15
        assert node.core.marker_alignment_rotation_limit == .15
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()
