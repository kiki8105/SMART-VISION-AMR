"""The isolated Gazebo fault schedule is deterministic and cannot mask stamps."""

import importlib.util
from pathlib import Path


MODULE = Path(__file__).resolve().parents[4] / 'scripts/classroom_fault_profile.py'
spec = importlib.util.spec_from_file_location('classroom_fault_profile', MODULE)
faults = importlib.util.module_from_spec(spec)
import sys
sys.modules[spec.name] = faults
spec.loader.exec_module(faults)


def test_clean_until_explicit_switch():
    profile = faults.FaultProfile()
    assert all(profile.decide(topic, 100).delay_seconds == 0
               for topic in faults.TOPICS)
    profile.set_profile('jitter', 100)
    assert profile.decide('scan', 100).delay_seconds == 0
    assert profile.decide('scan', 100.1).delay_seconds == .04


def test_outages_are_stream_specific_and_bounded():
    profile = faults.FaultProfile()
    profile.set_profile('outages', 100)
    assert profile.decide('scan', 104.5).reason == 'outage'
    assert not profile.decide('odom', 104.5).drop
    assert profile.decide('odom', 110.4).drop
    assert profile.decide('tf', 116.4).drop
    assert profile.decide('scan', 106.99).drop
    assert not profile.decide('scan', 107.).drop
    assert profile.decide('odom', 112.99).drop
    assert not profile.decide('odom', 113.).drop
    assert profile.decide('tf', 118.99).drop
    assert not profile.decide('tf', 119.).drop


def test_web_transient_has_short_latency_then_bounded_loss_and_recovers():
    profile = faults.FaultProfile()
    profile.set_profile('transient', 100.)
    for topic in faults.TOPICS:
        first = profile.decide(topic, 100.)
        assert not first.drop and first.delay_seconds == .35
        assert profile.decide(topic, 100.79).reason == 'transient_delay'
        assert profile.decide(topic, 100.81).reason == 'clean'
        assert not profile.decide(topic, 101.54).drop
        dropped = profile.decide(topic, 101.56)
        assert dropped.drop and dropped.reason == 'transient'
        assert profile.decide(topic, 102.34).drop
        recovered = profile.decide(topic, 102.36)
        assert not recovered.drop and recovered.delay_seconds == 0.


def test_jitter_and_packet_loss_repeat_exactly():
    first, second = faults.FaultProfile(), faults.FaultProfile()
    for profile in (first, second):
        profile.set_profile('mixed', 100.)
    a = [first.decide('scan', 101.) for _ in range(64)]
    b = [second.decide('scan', 101.) for _ in range(64)]
    assert a == b
    assert any(decision.reason == 'packet_loss' for decision in a)
    assert max(decision.delay_seconds for decision in a) == .22


def test_invalid_profile_or_topic_rejected():
    profile = faults.FaultProfile()
    import pytest
    with pytest.raises(ValueError):
        profile.set_profile('unsafe', 0.)
    with pytest.raises(ValueError):
        profile.decide('cmd_vel', 0.)
