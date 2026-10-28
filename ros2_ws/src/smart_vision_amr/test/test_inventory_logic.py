"""Inventory persistence and FIFO delivery decisions, without ROS or a robot."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from smart_vision_amr.inventory_logic import InventoryQueue


MARKERS = [{'id': index, 'x': float(index), 'y': 0., 'yaw': 0.}
           for index in range(4)]


class InventoryLogicTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.file = Path(self.directory.name) / 'nested' / 'inventory.json'
        self.queue = InventoryQueue(MARKERS, self.file)

    def submit(self, user=1, marker=2, item='볼트', **extras):
        return self.queue.submit(user, dict(marker_id=marker, item=item,
                                            quantity=2, **extras))

    def requests(self):
        return self.queue.snapshot()['requests']

    def test_request_is_validated_and_committed_before_success(self):
        request = self.submit(note='  선반 2번  ')
        self.assertEqual(request['id'], 1)
        self.assertEqual(request['user_id'], 1)
        self.assertEqual(request['status'], 'PENDING')
        self.assertEqual(request['note'], '선반 2번')
        self.assertIsNone(request['trip_id'])
        self.assertGreater(request['created_at'], 0.)
        self.assertEqual(request['created_at'], request['updated_at'])
        saved = json.loads(self.file.read_text())
        self.assertEqual(saved['requests'][0], request)
        self.assertEqual(list(self.file.parent.iterdir()), [self.file])

    def test_invalid_user_ids_never_create_requests(self):
        for user_id in (0, 100, -1, True, False, 1.0, '1', None):
            with self.subTest(user_id=user_id), self.assertRaises(ValueError):
                self.submit(user=user_id)
        self.assertEqual(self.requests(), [])

    def test_invalid_payloads_and_exact_integer_validation(self):
        valid = {'marker_id': 2, 'item': '볼트', 'quantity': 1}
        payloads = [None, [], '',
                    dict(valid, marker_id=True), dict(valid, marker_id=2.),
                    dict(valid, marker_id='2'), dict(valid, marker_id=99),
                    dict(valid, item=' '), dict(valid, item='a'*81),
                    dict(valid, item=3), dict(valid, quantity=True),
                    dict(valid, quantity=0), dict(valid, quantity=10000),
                    dict(valid, quantity=1.1), dict(valid, quantity='2'),
                    dict(valid, note='a'*301), dict(valid, note=[]),
                    dict(valid, client_request_id='a'*81),
                    dict(valid, client_request_id=1)]
        for payload in payloads:
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                self.queue.submit(1, payload)
        self.assertEqual(self.requests(), [])

    def test_idempotency_is_per_user_and_survives_restart(self):
        first = self.submit(client_request_id='phone-123')
        same = self.submit(marker=3, item='다른 값', client_request_id='phone-123')
        other = self.submit(user=2, client_request_id='phone-123')
        self.assertEqual(first, same)
        self.assertNotEqual(first['id'], other['id'])
        recovered = InventoryQueue(MARKERS, self.file)
        self.assertEqual(recovered.submit(1, {'marker_id': 3, 'item': '아무 값',
                                             'client_request_id': 'phone-123'}), first)
        self.assertEqual(len(recovered.snapshot()['requests']), 2)

    def test_empty_idempotency_key_does_not_merge_separate_requests(self):
        self.submit()
        self.submit()
        self.assertEqual([item['id'] for item in self.requests()], [1, 2])

    def test_manager_cannot_cancel_other_request_or_dispatched_request(self):
        request = self.submit()
        with self.assertRaises(PermissionError):
            self.queue.cancel_pending(2, request['id'])
        self.queue.start_trip()
        with self.assertRaises(ValueError):
            self.queue.cancel_pending(1, request['id'])
        self.assertEqual(self.requests()[0]['status'], 'IN_PROGRESS')

    def test_pending_cancel_is_persisted_and_not_dispatched(self):
        first, second = self.submit(), self.submit(user=99)
        canceled = self.queue.cancel_pending(1, first['id'])
        self.assertEqual(canceled['status'], 'CANCELED')
        self.assertEqual(self.queue.start_trip()['request_ids'], [second['id']])
        with self.assertRaises(ValueError):
            self.queue.cancel_pending(1, first['id'])

    def test_delete_own_finished_history_only_and_preserve_pending_work(self):
        canceled = self.submit()
        pending = self.submit()
        self.queue.cancel_pending(1, canceled['id'])
        with self.assertRaises(PermissionError):
            self.queue.delete_finished_history(2, canceled['id'])
        with self.assertRaises(ValueError):
            self.queue.delete_finished_history(1, pending['id'])
        self.assertEqual(self.queue.delete_finished_history(1, canceled['id']),
                         {'deleted': True, 'deferred': False})
        self.assertEqual([item['id'] for item in self.requests()], [pending['id']])
        self.assertEqual(InventoryQueue(MARKERS, self.file).snapshot(), self.queue.snapshot())
        self.assertGreater(self.submit()['id'], pending['id'])

    def test_delete_delivered_record_in_active_trip_waits_for_home(self):
        first, second = self.submit(), self.submit()
        self.queue.start_trip()
        self.queue.complete_request(first['id'])
        result = self.queue.delete_finished_history(1, first['id'])
        self.assertEqual(result, {'deleted': False, 'deferred': True})
        self.assertEqual(self.queue.snapshot()['deferred_history_ids'], [first['id']])
        self.assertEqual([item['id'] for item in self.requests()], [first['id'], second['id']])
        self.assertEqual(self.queue.snapshot()['active_trip']['current_request_id'], second['id'])
        self.queue.cancel_trip()
        self.queue.close_trip()
        self.assertEqual(self.queue.clear_deferred_history_at_home(), 1)
        self.assertEqual([item['id'] for item in self.requests()], [second['id']])

    def test_delete_finished_history_rolls_back_on_disk_failure(self):
        request = self.submit()
        self.queue.cancel_pending(1, request['id'])
        self.assert_rollback(lambda: self.queue.delete_finished_history(1, request['id']))
        self.assertEqual(self.requests()[0]['status'], 'CANCELED')

    def test_clear_finished_history_keeps_unfinished_work_and_active_trip(self):
        delivered = self.submit()
        self.queue.start_trip()
        self.queue.complete_request(delivered['id'])
        self.queue.close_trip()
        canceled = self.submit()
        self.queue.cancel_pending(1, canceled['id'])
        pending = self.submit()
        other = self.submit(user=2)
        self.queue.cancel_pending(2, other['id'])
        active_done, active_work = self.submit(), self.submit()
        active_trip = self.queue.start_trip([active_done['id'], active_work['id']])
        self.queue.complete_request(active_done['id'])

        self.assertEqual(self.queue.clear_finished_history(), 3)
        self.assertEqual([item['id'] for item in self.requests()],
                         [pending['id'], active_done['id'], active_work['id']])
        self.assertEqual(self.queue.snapshot()['active_trip'], active_trip | {
            'current_request_id': active_work['id']})
        self.assertEqual(self.queue.clear_finished_history(), 0)
        self.queue.complete_request(active_work['id'])
        self.queue.close_trip()
        self.assertEqual(self.queue.clear_finished_history(), 2)
        self.assertEqual([item['id'] for item in self.requests()], [pending['id']])
        self.assertEqual(self.submit()['id'], active_work['id'] + 1)
        recovered = InventoryQueue(MARKERS, self.file)
        self.assertEqual(recovered.snapshot(), self.queue.snapshot())

    def test_clear_finished_history_rolls_back_on_storage_error(self):
        canceled = self.submit()
        self.queue.cancel_pending(1, canceled['id'])
        self.assert_rollback(self.queue.clear_finished_history)
        self.assertEqual(self.requests()[0]['status'], 'CANCELED')

    def test_deferred_history_waits_for_verified_home_after_trip_closes(self):
        delivered, unfinished = self.submit(), self.submit()
        self.queue.start_trip()
        self.queue.complete_request(delivered['id'])
        self.assert_rollback(lambda: self.queue.clear_finished_history(defer_active_trip=True))
        self.assertEqual(self.queue.clear_finished_history(defer_active_trip=True), 0)
        self.assertEqual(self.queue.snapshot()['deferred_history_ids'], [delivered['id']])
        self.assertEqual(json.loads(self.file.read_text())['deferred_history_ids'], [delivered['id']])
        self.queue.cancel_trip()
        self.assert_rollback(self.queue.close_trip)
        self.queue.close_trip()
        self.assertEqual(self.queue.snapshot()['deferred_history_ids'], [delivered['id']])
        self.assert_rollback(self.queue.clear_deferred_history_at_home)
        self.assertEqual(self.queue.clear_deferred_history_at_home(), 1)
        self.assertEqual([(item['id'], item['status']) for item in self.requests()],
                         [(unfinished['id'], 'PENDING')])
        self.assertEqual(self.queue.snapshot()['deferred_history_ids'], [])
        self.assertEqual(InventoryQueue(MARKERS, self.file).snapshot(), self.queue.snapshot())

    def test_deferred_history_intent_survives_restart_recovery(self):
        delivered, unfinished = self.submit(), self.submit()
        self.queue.start_trip()
        self.queue.complete_request(delivered['id'])
        self.queue.clear_finished_history(defer_active_trip=True)
        recovered = InventoryQueue(MARKERS, self.file)
        self.assertIsNone(recovered.snapshot()['active_trip'])
        self.assertEqual(recovered.snapshot()['deferred_history_ids'], [delivered['id']])
        self.assertEqual([(item['id'], item['status'])
                          for item in recovered.snapshot()['requests']],
                         [(delivered['id'], 'DELIVERED'), (unfinished['id'], 'PENDING')])
        self.assertEqual(recovered.clear_deferred_history_at_home(), 1)
        self.assertEqual([(item['id'], item['status'])
                          for item in recovered.snapshot()['requests']],
                         [(unfinished['id'], 'PENDING')])
        self.assertEqual(recovered.submit(1, {'marker_id': 2, 'item': '너트'})['id'],
                         unfinished['id'] + 1)

    def test_old_storage_without_deferred_ids_still_loads(self):
        self.submit()
        stored = json.loads(self.file.read_text())
        del stored['deferred_history_ids']
        self.file.write_text(json.dumps(stored))
        self.assertEqual(InventoryQueue(MARKERS, self.file).snapshot()['deferred_history_ids'], [])

    def test_new_cancellation_after_reset_is_not_deleted_at_home(self):
        delivered, unfinished = self.submit(), self.submit()
        self.queue.start_trip([delivered['id'], unfinished['id']])
        self.queue.complete_request(delivered['id'])
        self.queue.clear_finished_history(defer_active_trip=True)
        later = self.submit(user=2)
        self.queue.cancel_pending(2, later['id'])
        self.queue.cancel_trip()
        self.queue.close_trip()
        self.assertEqual(self.queue.clear_deferred_history_at_home(), 1)
        self.assertEqual([(item['id'], item['status']) for item in self.requests()],
                         [(unfinished['id'], 'PENDING'), (later['id'], 'CANCELED')])

    def test_fifo_separates_repeated_location_and_selected_ids_are_fifo(self):
        one, two, three = self.submit(), self.submit(marker=2), self.submit(marker=1)
        trip = self.queue.start_trip([three['id'], one['id'], two['id']])
        self.assertEqual(trip['request_ids'], [one['id'], two['id'], three['id']])
        self.assertEqual(trip['current_request_id'], one['id'])
        self.assertEqual([item['status'] for item in self.requests()],
                         ['IN_PROGRESS', 'QUEUED', 'QUEUED'])
        self.assertEqual({item['trip_id'] for item in self.requests()}, {trip['id']})
        trip = self.queue.complete_request(one['id'])
        self.assertEqual(trip['current_request_id'], two['id'])
        self.assertEqual(self.requests()[0]['status'], 'DELIVERED')

    def test_explicit_selection_leaves_other_pending_requests_unassigned(self):
        first, second = self.submit(), self.submit()
        self.assertEqual(self.queue.start_trip([second['id']])['request_ids'], [second['id']])
        self.assertEqual(self.requests()[0]['status'], 'PENDING')
        self.assertIsNone(self.requests()[0]['trip_id'])

    def test_dispatch_errors_do_not_change_storage(self):
        with self.assertRaises(ValueError):
            self.queue.start_trip()
        self.submit()
        before = self.queue.snapshot()
        for ids in ([], [True], [1.], [1, 1], [999], '1', [1]*101):
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                self.queue.start_trip(ids)
            self.assertEqual(self.queue.snapshot(), before)
        self.queue.start_trip()
        with self.assertRaises(ValueError):
            self.queue.start_trip()

    def test_initial_dispatch_takes_at_most_one_hundred_fifo_requests(self):
        queue = InventoryQueue(MARKERS, None)
        for _ in range(101):
            queue.submit(1, {'marker_id': 2, 'item': '볼트'})
        trip = queue.start_trip()
        self.assertEqual(trip['request_ids'], list(range(1, 101)))
        self.assertEqual(queue.snapshot()['requests'][-1]['status'], 'PENDING')

    def test_appending_during_active_trip_preserves_order_and_is_idempotent(self):
        first, second = self.submit(), self.submit(marker=3)
        self.queue.start_trip()
        third = self.submit(user=3)
        trip = self.queue.append_to_trip(third['id'])
        self.assertEqual(trip['request_ids'], [first['id'], second['id'], third['id']])
        self.assertEqual(trip['current_request_id'], first['id'])
        self.assertEqual(self.requests()[2]['status'], 'QUEUED')
        self.assertEqual(self.queue.append_to_trip(third['id']), trip)
        self.assertEqual(self.queue.complete_request(first['id'])['current_request_id'], second['id'])
        self.assertEqual(self.queue.complete_request(second['id'])['current_request_id'], third['id'])

    def test_new_request_during_automatic_return_reactivates_trip(self):
        first = self.submit()
        original = self.queue.start_trip()
        returning = self.queue.complete_request(first['id'])
        self.assertEqual(returning['status'], 'RETURNING')
        self.assertTrue(returning['accepting_updates'])
        self.assertIsNone(returning['current_request_id'])
        second = self.submit(user=4)
        active = self.queue.append_to_trip(second['id'])
        self.assertEqual(active['id'], original['id'])
        self.assertEqual(active['status'], 'ACTIVE')
        self.assertEqual(active['current_request_id'], second['id'])
        self.assertEqual(active['request_ids'], [first['id'], second['id']])
        self.assertEqual(self.requests()[1]['status'], 'IN_PROGRESS')

    def test_complete_requires_current_delivery_and_cannot_fake_other_request(self):
        first, second = self.submit(), self.submit()
        with self.assertRaises(ValueError):
            self.queue.complete_request(first['id'])
        self.queue.start_trip()
        for request_id in (second['id'], 99, True, 1.):
            with self.subTest(request_id=request_id), self.assertRaises(ValueError):
                self.queue.complete_request(request_id)
        self.queue.complete_request(first['id'])
        with self.assertRaises(ValueError):
            self.queue.complete_request(first['id'])

    def test_cancel_trip_returns_remaining_to_pending_and_keeps_delivered_history(self):
        first, second, third = self.submit(), self.submit(), self.submit()
        initial = self.queue.start_trip()
        self.queue.complete_request(first['id'])
        trip = self.queue.cancel_trip()
        self.assertEqual(trip['status'], 'RETURNING')
        self.assertFalse(trip['accepting_updates'])
        self.assertEqual(trip['request_ids'], initial['request_ids'])
        self.assertEqual([item['status'] for item in self.requests()],
                         ['DELIVERED', 'PENDING', 'PENDING'])
        self.assertEqual([item['trip_id'] for item in self.requests()],
                         [initial['id'], None, None])
        with self.assertRaises(ValueError):
            self.queue.start_trip()
        with self.assertRaises(ValueError):
            self.queue.append_to_trip(second['id'])
        self.queue.close_trip()  # Caller has separately confirmed Homepoint arrival.
        retry = self.queue.start_trip()
        self.assertEqual(retry['request_ids'], [second['id'], third['id']])
        self.assertGreater(retry['id'], initial['id'])

    def test_freeze_disallows_additions_without_faking_completion(self):
        first = self.submit()
        self.queue.start_trip()
        second = self.submit()
        trip = self.queue.freeze_updates()
        self.assertFalse(trip['accepting_updates'])
        self.assertEqual(trip['current_request_id'], first['id'])
        self.assertEqual(self.requests()[0]['status'], 'IN_PROGRESS')
        with self.assertRaises(ValueError):
            self.queue.append_to_trip(second['id'])
        self.assertEqual(self.queue.complete_request(first['id'])['status'], 'RETURNING')
        self.assertFalse(self.queue.snapshot()['active_trip']['accepting_updates'])

    def test_close_requires_finished_work_then_keeps_durable_history(self):
        request = self.submit()
        self.queue.start_trip()
        with self.assertRaises(ValueError):
            self.queue.close_trip()
        self.queue.complete_request(request['id'])
        self.queue.close_trip()
        self.assertIsNone(self.queue.snapshot()['active_trip'])
        self.assertEqual(self.requests()[0]['status'], 'DELIVERED')
        self.assertIsNone(InventoryQueue(MARKERS, self.file).snapshot()['active_trip'])
        self.queue.close_trip()  # Repeated home acknowledgments are harmless.

    def test_restart_recovers_only_unfulfilled_requests_without_auto_dispatch(self):
        first, second, third = self.submit(), self.submit(), self.submit()
        self.queue.start_trip([first['id'], second['id']])
        self.queue.complete_request(first['id'])
        recovered = InventoryQueue(MARKERS, self.file).snapshot()
        self.assertIsNone(recovered['active_trip'])
        self.assertEqual([item['status'] for item in recovered['requests']],
                         ['DELIVERED', 'PENDING', 'PENDING'])
        self.assertIsNone(recovered['requests'][1]['trip_id'])
        self.assertEqual(json.loads(self.file.read_text())['active_trip'], None)
        newer = InventoryQueue(MARKERS, self.file).submit(
            1, {'marker_id': 2, 'item': '새 요청'})
        self.assertGreater(newer['id'], third['id'])

    def test_restart_recovers_queued_and_returning_trips_without_auto_drive(self):
        first, second = self.submit(), self.submit()
        self.queue.start_trip()
        recovered = InventoryQueue(MARKERS, self.file)
        self.assertEqual([item['status'] for item in recovered.snapshot()['requests']],
                         ['PENDING', 'PENDING'])
        recovered.start_trip()
        recovered.complete_request(first['id'])
        recovered.complete_request(second['id'])
        again = InventoryQueue(MARKERS, self.file).snapshot()
        self.assertIsNone(again['active_trip'])
        self.assertEqual([item['status'] for item in again['requests']], ['DELIVERED']*2)

    def test_every_returned_object_is_a_deep_copy(self):
        request = self.submit()
        request['status'] = 'DELIVERED'
        snapshot = self.queue.snapshot()
        snapshot['requests'].clear()
        self.assertEqual(self.requests()[0]['status'], 'PENDING')
        trip = self.queue.start_trip()
        trip['request_ids'].clear()
        self.assertEqual(self.queue.snapshot()['active_trip']['request_ids'], [1])
        trip = self.queue.complete_request(1)
        trip['accepting_updates'] = False
        self.assertTrue(self.queue.snapshot()['active_trip']['accepting_updates'])

    def assert_rollback(self, operation):
        before = self.queue.snapshot()
        disk = self.file.read_bytes() if self.file.exists() else None
        with patch('smart_vision_amr.inventory_logic.os.replace', side_effect=OSError('disk full')):
            with self.assertRaises(OSError):
                operation()
        self.assertEqual(self.queue.snapshot(), before)
        self.assertEqual(self.file.read_bytes() if self.file.exists() else None, disk)
        self.assertFalse(list(self.file.parent.glob('*.tmp')))
        self.assertFalse(list(self.file.parent.glob('.*.tmp')))

    def test_storage_failure_rolls_back_submit_cancel_start_and_all_trip_mutations(self):
        self.assert_rollback(lambda: self.submit())
        first = self.submit()
        self.assertEqual(first['id'], 1)
        self.assert_rollback(lambda: self.queue.cancel_pending(1, first['id']))
        self.assert_rollback(self.queue.start_trip)
        self.queue.start_trip()
        second = self.submit()
        self.assert_rollback(lambda: self.queue.append_to_trip(second['id']))
        self.assert_rollback(lambda: self.queue.complete_request(first['id']))
        self.assert_rollback(self.queue.freeze_updates)
        self.assert_rollback(self.queue.cancel_trip)
        self.queue.complete_request(first['id'])
        self.assert_rollback(self.queue.close_trip)

    def test_interrupted_restart_write_failure_is_not_silently_ignored(self):
        self.submit()
        self.queue.start_trip()
        data = self.file.read_bytes()
        with patch('smart_vision_amr.inventory_logic.os.replace', side_effect=OSError('readonly')):
            with self.assertRaises(OSError):
                InventoryQueue(MARKERS, self.file)
        self.assertEqual(self.file.read_bytes(), data)

    def test_corrupt_storage_fails_closed_without_overwriting_file(self):
        self.submit()
        valid = json.loads(self.file.read_text())
        mutations = [lambda value: value.update(version=2),
                     lambda value: value.update(requests='lost'),
                     lambda value: value.update(next_request_id=1),
                     lambda value: value['requests'][0].update(status='BOGUS'),
                     lambda value: value['requests'][0].update(user_id=True),
                     lambda value: value['requests'][0].update(created_at=float('nan'))]
        for mutate in mutations:
            data = deepcopy(valid)
            mutate(data)
            encoded = json.dumps(data)
            self.file.write_text(encoded)
            with self.subTest(data=data), self.assertRaises(ValueError):
                InventoryQueue(MARKERS, self.file)
            self.assertEqual(self.file.read_text(), encoded)

    def test_missing_request_and_invalid_ids_rejected(self):
        self.submit()
        for request_id in (True, '1', 1., -1, 0, 99):
            for method in (lambda value: self.queue.cancel_pending(1, value),
                           self.queue.append_to_trip, self.queue.complete_request):
                with self.subTest(request_id=request_id), self.assertRaises(ValueError):
                    method(request_id)

    def test_mapping_marker_ids_and_empty_operations(self):
        queue = InventoryQueue({2: {'name': '선반'}}, None)
        self.assertIsNone(queue.freeze_updates())
        self.assertIsNone(queue.cancel_trip())
        self.assertIsNone(queue.close_trip())
        self.assertEqual(queue.submit(99, {'marker_id': 2, 'item': '부품'})['quantity'], 1)
        for markers in ([], [True], [{'name': 'missing'}], ['2']):
            with self.subTest(markers=markers), self.assertRaises(ValueError):
                InventoryQueue(markers, None)


if __name__ == '__main__':
    unittest.main()
