"""Same recorded route/pose: old generation-wide skip versus bounded setup.

Only transport ownership is simulated; no ROS executor or motion is started.
The historical input contains no recorded physical stop certificate, and the
new policy must therefore wait rather than claiming that a stop occurred.
"""
import importlib.util
import json
from math import cos, sin
from pathlib import Path

import pytest
from geometry_msgs.msg import TransformStamped

from test_navigation_input_preflight import healthy_follower
from test_path_follower_cancel import goal_for
from test_sharp_replan_alignment_independent import path_message

ARTIFACT = (Path(__file__).resolve().parents[4] / 'benchmarks' /
            'driving_parameter_tuning_20261104' / 'accepted_route_tracking_20261105')


@pytest.mark.parametrize('case_name', ['v2', 'local025'])
def test_recorded_material_entry_old_drives_but_new_waits_for_real_stop(case_name):
    spec = importlib.util.spec_from_file_location(
        'archived_pre_material_alignment_follower',
        ARTIFACT / 'path_follower_before_material_replan_alignment.py')
    baseline = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(baseline)
    case = json.loads((ARTIFACT / 'actual_entry_fixture.json').read_text())['cases'][case_name]
    previous, replacement = path_message(case['old_path']), path_message(case['replacement_path'])
    node = healthy_follower()
    node.parameters.update({'initial_heading_alignment_enabled': True,
                            'initial_heading_alignment_prune_distance': .4})
    transform = TransformStamped()
    transform.header.stamp.sec = node.clock_seconds
    pose = case['estimated_pose']
    transform.transform.translation.x, transform.transform.translation.y = pose['x'], pose['y']
    transform.transform.rotation.z, transform.transform.rotation.w = sin(pose['yaw']/2), cos(pose['yaw']/2)
    node.tf_buffer.lookup_transform.side_effect = None
    node.tf_buffer.lookup_transform.return_value = transform
    node.goal_generation = 1
    node.requested_goal = goal_for(replacement, stamp=1)
    reference = {'generation': 1, 'points': tuple(node._points(previous)),
                 'path_stamp_ns': case['old_path']['stamp_ns']}
    node._route_alignment_reference = reference
    assert abs(node._initial_heading_error(replacement)) > .55

    # Only the archived implementation uses its old redundant completion flag.
    node.initial_alignment_completed_generation = 1
    baseline.PathFollower._send_path(node, replacement)
    del node.initial_alignment_completed_generation
    node.client.send_goal_async.assert_called_once()
    node.spin_client.send_goal_async.assert_not_called()

    # Restore precisely the same no-action, post-terminal input for the new
    # dispatch method. No fake stationary history is supplied to pass a gate.
    node.client.send_goal_async.reset_mock()
    node.goal_request_pending = False
    node.request_context = None
    node.active_path = None
    node._route_alignment_reference = reference
    node._send_path(replacement)
    node.client.send_goal_async.assert_not_called()
    node.spin_client.send_goal_async.assert_not_called()
    assert node._pending_replan_alignment['path'] is replacement
