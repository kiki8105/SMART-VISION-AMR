"""Removal proof contains only valid, measurement-time free rays, never TTL guesses."""
import math
from unittest.mock import Mock

from test_live_obstacles import harness


def test_visibility_distinguishes_free_hit_and_unobserved_cells_without_map_mutation():
    with harness() as node:
        node.visibility_pub = Mock()
        original = list(node.map['message'].data)
        scan = node.scan(2.)
        msg = node.visibility_pub.publish.call_args.args[0]
        assert msg.header.stamp == scan.header.stamp
        assert msg.header.frame_id == 'map'
        assert msg.info == node.map['message'].info
        assert msg.data[1*6+0] == 0 and msg.data[1*6+1] == 0
        assert msg.data[1*6+2] == 100
        assert msg.data[1*6+3] == -1 and msg.data[2*6+2] == -1
        node.scan(math.inf)
        assert node.visibility_pub.publish.call_args.args[0].data[1*6+2] == 0
        assert list(node.map['message'].data) == original


def test_all_invalid_beams_publish_no_new_positive_removal_proof():
    with harness() as node:
        node.visibility_pub = Mock()
        node.scan(2.)
        count = node.visibility_pub.publish.call_count
        for value in (math.nan, -math.inf, 0., -.1, 5.):
            node.scan(value)
        assert node.visibility_pub.publish.call_count == count


def test_partial_invalid_beam_keeps_its_tracked_cell_unknown():
    with harness() as node:
        node.visibility_pub = Mock()
        node.scan(2.)
        scan = node.scan(math.nan)
        scan.angle_increment = math.pi/2
        scan.ranges = [math.nan, 1.]
        node.on_scan(scan)
        msg = node.visibility_pub.publish.call_args.args[0]
        assert msg.data[1*6+2] == -1
        assert msg.data[2*6+0] == 100


def test_ttl_expired_hit_is_unknown_until_an_actual_free_ray_crosses_it():
    with harness() as node:
        node.visibility_pub = Mock()
        node.scan(2.)
        node.clock.return_value = 15.
        node.scan(1., math.pi/2)
        assert 8 not in node.observed()
        assert node.visibility_pub.publish.call_args.args[0].data[8] == -1
        node.scan(math.inf)
        assert node.visibility_pub.publish.call_args.args[0].data[8] == 0
