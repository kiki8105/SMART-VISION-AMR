"""The bridge must not drive before lease acknowledgement or after input loss."""
import unittest

from controller_manual_bridge import ManualRelay


class FakeClock:
    def __init__(self):
        self.now = 1.

    def __call__(self):
        return self.now


class FakeWeb:
    def __init__(self):
        self.requests = []
        self.manual_state = 'IDLE'
        self.available = True
        self.fail = False
        self.ack_count = 0

    def state(self):
        if self.fail:
            raise OSError('unavailable')
        return {'manual': {'state': self.manual_state, 'available': self.available}}

    def manual(self, payload):
        if self.fail:
            raise OSError('unavailable')
        self.requests.append(payload.copy())
        if not payload['active']:
            self.manual_state = 'IDLE'
            return {'state': 'RELEASING', 'owner': True, 'available': True}
        self.ack_count += 1
        self.manual_state = 'ACTIVE' if self.ack_count >= 2 else 'ACQUIRING'
        return {'state': self.manual_state, 'owner': True, 'available': True}


class ManualRelayTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.web = FakeWeb()
        self.relay = ManualRelay(self.web, clock=self.clock)

    def step(self, delta=.1):
        self.clock.now += delta
        self.relay.tick()

    def test_zero_until_active_ack_then_release_on_zero(self):
        self.relay.on_twist(.1, .2)
        self.relay.tick()
        self.assertEqual(self.web.requests[-1]['linear'], 0.)
        self.assertFalse(self.relay.armed)
        self.step()
        self.assertEqual(self.web.requests[-1]['linear'], 0.)
        self.assertTrue(self.relay.armed)
        self.relay.on_twist(.1, .2)
        self.step()
        self.assertEqual(self.web.requests[-1]['linear'], .1)
        self.relay.on_twist(0., 0.)
        self.step()
        self.assertFalse(self.web.requests[-1]['active'])
        self.assertEqual(self.web.requests[-1]['linear'], 0.)
        self.assertIsNone(self.relay.control_id)

    def test_lost_gui_input_releases_lease_and_clears_stale_vector(self):
        self.relay.on_twist(.1, 0.)
        self.relay.tick()
        self.step()
        self.relay.on_twist(.1, 0.)
        self.step()
        self.step(.29)
        self.assertFalse(self.web.requests[-1]['active'])
        self.assertEqual(self.relay.vector, (0., 0.))
        count = len(self.web.requests)
        self.step()
        self.assertEqual(len(self.web.requests), count)

    def test_web_unavailable_or_occupied_never_acquires(self):
        self.web.manual_state = 'RELEASING'
        self.relay.on_twist(.1, 0.)
        self.relay.tick()
        self.assertEqual(self.web.requests, [])
        self.web.manual_state = 'IDLE'
        self.web.fail = True
        self.step(.4)
        self.assertEqual(self.web.requests, [])
        self.assertFalse(self.relay.available)

    def test_web_loss_after_motion_releases_locally(self):
        self.relay.on_twist(.1, 0.)
        self.relay.tick()
        self.step()
        self.relay.on_twist(.1, 0.)
        self.step()
        self.web.fail = True
        self.relay.on_twist(.1, 0.)
        self.step()
        self.assertIsNone(self.relay.control_id)
        self.assertEqual(self.relay.vector, (0., 0.))


if __name__ == '__main__':
    unittest.main()
