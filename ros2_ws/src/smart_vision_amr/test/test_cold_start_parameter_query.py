"""Cold-prior read retries never seed, move, or erase the original observers."""
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest
from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.task import Future

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'scripts'))
import verify_classroom_web_dwell as dwell


class QueryClient:
    def __init__(self, outcomes):
        self.outcomes = list(outcomes)
        self.requests, self.futures, self.removed = [], [], []
        self.wait_timeouts = []

    def wait_for_service(self, *, timeout_sec):
        self.wait_timeouts.append(timeout_sec)
        return True

    def call_async(self, request):
        self.requests.append(request)
        future = Future()
        future.query_outcome = self.outcomes.pop(0)
        self.futures.append(future)
        return future

    def remove_pending_request(self, future):
        self.removed.append(future)


@pytest.fixture
def query(monkeypatch):
    import rclpy
    clock = [100.]
    monkeypatch.setattr(dwell.time, 'monotonic', lambda: clock[0])

    def spin(_node, future, *, timeout_sec):
        outcome = future.query_outcome
        clock[0] += timeout_sec if outcome == 'timeout' else min(.01, timeout_sec)
        if outcome == 'timeout':
            return
        if outcome == 'error':
            future.set_exception(RuntimeError('read transport unavailable'))
            return
        kind = (ParameterType.PARAMETER_INTEGER if outcome == 'invalid'
                else ParameterType.PARAMETER_BOOL)
        value = ParameterValue(type=kind, bool_value=outcome is True)
        future.set_result(SimpleNamespace(values=[value]))

    monkeypatch.setattr(rclpy, 'spin_until_future_complete', spin)

    def run(outcomes, *, budget=75.):
        client = QueryClient(outcomes)
        node = SimpleNamespace(create_client=lambda *_: client)
        proof = {}
        return node, client, proof, clock, clock[0]+budget
    return run


def test_timeout_then_false_observation_retries_only_the_read(query):
    node, client, proof, clock, deadline = query(['timeout', False])
    assert dwell._read_amcl_launch_prior(node, deadline, proof) is False
    assert [request.names for request in client.requests] == [['set_initial_pose']]*2
    assert client.removed == [client.futures[0]]
    assert not client.futures[0].done()
    detail = proof['launch_prior_query']
    assert detail['attempt_count'] == 2 and detail['outcome'] == 'parameter_observed'
    assert [entry['outcome'] for entry in detail['attempts']] == [
        'response_timeout', 'parameter_observed']
    assert detail['elapsed_s'] == pytest.approx(5.01)
    assert clock[0] < deadline


def test_all_failed_responses_end_after_three_reads(query):
    node, client, proof, clock, deadline = query(['timeout']*3)
    with pytest.raises(RuntimeError, match='bounded read retries'):
        dwell._read_amcl_launch_prior(node, deadline, proof)
    assert len(client.requests) == len(client.removed) == 3
    assert proof['launch_prior_query']['outcome'] == 'attempts_exhausted'
    assert proof['launch_prior_query']['elapsed_s'] == 15.


def test_absolute_budget_caps_first_timeout_and_prevents_another_read(query):
    node, client, proof, clock, deadline = query(['timeout']*3, budget=2.)
    with pytest.raises(RuntimeError, match='bounded read retries'):
        dwell._read_amcl_launch_prior(node, deadline, proof)
    assert len(client.requests) == 1 and client.wait_timeouts == [2.]
    assert proof['launch_prior_query']['attempts'][0]['timeout_s'] == 2.
    assert proof['launch_prior_query']['outcome'] == 'deadline_exhausted'
    assert clock[0] == deadline


def test_transport_error_can_retry_but_true_prior_is_not_retried_as_false(query):
    node, client, proof, _, deadline = query(['error', True, False])
    assert dwell._read_amcl_launch_prior(node, deadline, proof) is True
    assert len(client.requests) == 2 and not client.removed
    assert proof['launch_prior_query']['attempts'][0]['outcome'] == 'transport_error'
    assert proof['launch_prior_query']['attempts'][1]['value'] is True
    assert not dwell.cold_start_unseeded(
        {'robot': {'pose': None}, 'safety': {'latched': True, 'stop_acknowledged': True}},
        {'amcl_set_initial_pose': True, 'initial_pose_messages': 0,
         'amcl_messages': 0, 'map_tf_messages': 0})


def test_non_boolean_parameter_is_a_semantic_failure_not_a_retry(query):
    node, client, proof, _, deadline = query(['invalid', False])
    with pytest.raises(RuntimeError, match='not boolean'):
        dwell._read_amcl_launch_prior(node, deadline, proof)
    assert len(client.requests) == 1
    assert proof['launch_prior_query']['outcome'] == 'invalid_parameter_response'


def test_observed_prior_during_failed_read_is_not_erased_before_retry(query, monkeypatch):
    import rclpy
    import rclpy.node
    node, client, _, clock, _ = query(['timeout', False])
    callbacks, manual_requests = {}, []

    class Observer:
        def __init__(self, _name):
            pass
        def create_subscription(self, _type, topic, callback, _qos):
            callbacks[topic] = callback
        def create_client(self, *_):
            return client
        def destroy_node(self):
            pass

    original_spin = rclpy.spin_until_future_complete
    def spin_query(observer, future, *, timeout_sec):
        if future.query_outcome == 'timeout':
            callbacks['/amcl_pose'](object())
        original_spin(observer, future, timeout_sec=timeout_sec)

    state = {'robot': {'pose': None},
             'safety': {'latched': True, 'stop_acknowledged': True},
             'localization': {'manual_pose': {'available': True}}}
    monkeypatch.setattr(rclpy, 'init', lambda: None)
    monkeypatch.setattr(rclpy, 'shutdown', lambda: None)
    monkeypatch.setattr(rclpy.node, 'Node', Observer)
    monkeypatch.setattr(rclpy, 'spin_until_future_complete', spin_query)
    monkeypatch.setattr(rclpy, 'spin_once', lambda *_, **__: clock.__setitem__(0, clock[0]+.1))
    monkeypatch.setattr(dwell, 'request', lambda *args: state)
    monkeypatch.setattr(dwell, 'verify_initial_home_pose', lambda *_, **__: manual_requests.append(True))
    result = {}
    with pytest.raises(RuntimeError, match='already had a localization prior'):
        dwell.verify_cold_start_home_pose(None, 'http://127.0.0.1:8792', result)
    assert len(client.requests) == 2 and not manual_requests
    assert result['cold_start_proof']['before']['amcl_messages'] == 1
    assert result['cold_start_proof']['passed'] is False
