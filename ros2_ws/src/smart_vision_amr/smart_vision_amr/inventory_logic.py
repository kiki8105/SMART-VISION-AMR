"""Persistent inventory delivery queue, independent of ROS and HTTP.

``submit(user_id, payload)`` creates a PENDING request. An operator's explicit
``start_trip()`` assigns pending requests FIFO; ``complete_request()`` must only
be called after the mission has independently verified delivery. These methods
never publish goals or infer delivery from a timer or a position.

``snapshot()`` returns requests, active_trip, and deferred_history_ids. The trip
has id, request_ids (delivery order), current_request_id, status (ACTIVE or
RETURNING), and accepting_updates. A normal return still accepts new work;
``freeze_updates()`` / ``cancel_trip()`` prohibit additions. The caller invokes
``close_trip()`` only after actual Homepoint arrival. Every returned record is a
copy, and failed persistence rolls the entire mutation back.
"""
from copy import deepcopy
from pathlib import Path
import json
import math
import os
import tempfile
import threading
import time


REQUEST_STATUSES = frozenset({
    'PENDING', 'QUEUED', 'IN_PROGRESS', 'DELIVERED', 'CANCELED',
})


class InventoryQueue:
    """One server-owned queue. ``storage_file=None`` is useful for unit tests.

    Markers can be the existing list of marker dictionaries, a mapping keyed by
    integer marker IDs, or an iterable of integer IDs. Durable active trips are
    recovered as PENDING on construction: restarting a server never starts a
    robot or assumes an interrupted delivery succeeded.
    """

    def __init__(self, markers, storage_file):
        marker_values = markers.keys() if isinstance(markers, dict) else markers
        try:
            marker_ids = [item['id'] if isinstance(item, dict) else item
                          for item in marker_values]
        except (KeyError, TypeError):
            raise ValueError('등록된 마커 목록이 유효하지 않습니다.') from None
        if not marker_ids or any(type(item) is not int for item in marker_ids):
            raise ValueError('등록된 마커 ID는 정수여야 합니다.')
        self.marker_ids = frozenset(marker_ids)
        self.storage_file = Path(storage_file) if storage_file is not None else None
        self._lock = threading.RLock()
        self._data = {'version': 1, 'next_request_id': 1, 'next_trip_id': 1,
                      'requests': [], 'active_trip': None,
                      'deferred_history_ids': []}
        if self.storage_file is not None and self.storage_file.exists():
            with self.storage_file.open(encoding='utf-8') as stream:
                self._data = self._validate_storage(json.load(stream))
            changed = self._data['active_trip'] is not None
            now = time.time()
            for request in self._data['requests']:
                if request['status'] in {'QUEUED', 'IN_PROGRESS'}:
                    request.update(status='PENDING', trip_id=None, updated_at=now)
                    changed = True
            self._data['active_trip'] = None
            # Keep a history-reset reservation across restart. Recovery drops
            # the interrupted trip, but only a fresh verified Home pose may
            # release its protected finished records.
            if changed:
                self._persist()

    @staticmethod
    def _positive_id(value, label):
        if type(value) is not int or value < 1:
            raise ValueError(f'{label}는 양의 정수여야 합니다.')
        return value

    @staticmethod
    def _user_id(value):
        if type(value) is not int or not 1 <= value <= 99:
            raise ValueError('재고 관리자 ID는 1~99 정수여야 합니다.')
        return value

    @staticmethod
    def _text(value, label, limit, required=False):
        if not isinstance(value, str):
            raise ValueError(f'{label}는 문자열이어야 합니다.')
        value = value.strip()
        if (required and not value) or len(value) > limit:
            raise ValueError(f'{label}는 {1 if required else 0}~{limit}자여야 합니다.')
        return value

    def _payload(self, payload):
        if not isinstance(payload, dict):
            raise ValueError('요청 본문은 객체여야 합니다.')
        marker_id = payload.get('marker_id')
        if type(marker_id) is not int or marker_id not in self.marker_ids:
            raise ValueError('등록된 정수 마커 ID를 선택하세요.')
        quantity = payload.get('quantity', 1)
        if type(quantity) is not int or not 1 <= quantity <= 9999:
            raise ValueError('수량은 1~9999 정수여야 합니다.')
        return {
            'marker_id': marker_id,
            'item': self._text(payload.get('item'), '품목', 80, required=True),
            'quantity': quantity,
            'note': self._text(payload.get('note', ''), '메모', 300),
            'client_request_id': self._text(
                payload.get('client_request_id', ''), '중복 방지 ID', 80),
        }

    def _validate_storage(self, data):
        """Fail closed on damaged storage rather than silently losing requests."""
        if not isinstance(data, dict) or type(data.get('version')) is not int or data['version'] != 1:
            raise ValueError('지원하지 않거나 손상된 재고 요청 저장 파일입니다.')
        requests = data.get('requests')
        if not isinstance(requests, list):
            raise ValueError('저장된 요청 목록이 유효하지 않습니다.')
        ids, keys = set(), set()
        for request in requests:
            if not isinstance(request, dict):
                raise ValueError('저장된 요청이 유효하지 않습니다.')
            request_id = self._positive_id(request.get('id'), '요청 ID')
            user_id = self._user_id(request.get('user_id'))
            self._payload(request)
            if request_id in ids or request.get('status') not in REQUEST_STATUSES:
                raise ValueError('저장된 요청 ID 또는 상태가 유효하지 않습니다.')
            ids.add(request_id)
            key = (user_id, request.get('client_request_id', ''))
            if key[1] and key in keys:
                raise ValueError('저장된 중복 방지 ID가 중복됩니다.')
            keys.add(key)
            trip_id = request.get('trip_id')
            if trip_id is not None:
                self._positive_id(trip_id, '운행 ID')
            for key in ('created_at', 'updated_at'):
                timestamp = request.get(key)
                if (type(timestamp) not in (int, float)
                        or not math.isfinite(timestamp) or timestamp < 0):
                    raise ValueError('저장된 요청 시간이 유효하지 않습니다.')
        next_id = self._positive_id(data.get('next_request_id'), '다음 요청 ID')
        next_trip = self._positive_id(data.get('next_trip_id'), '다음 운행 ID')
        if next_id <= max(ids, default=0):
            raise ValueError('다음 요청 ID가 기존 ID와 충돌합니다.')
        recorded_trip_ids = [item['trip_id'] for item in requests
                             if item.get('trip_id') is not None]
        trip = data.get('active_trip')
        if trip is not None:
            if not isinstance(trip, dict):
                raise ValueError('저장된 운행이 유효하지 않습니다.')
            recorded_trip_ids.append(self._positive_id(trip.get('id'), '운행 ID'))
            request_ids = trip.get('request_ids')
            if (not isinstance(request_ids, list)
                    or any(type(item) is not int or item not in ids for item in request_ids)
                    or len(set(request_ids)) != len(request_ids)
                    or trip.get('status') not in {'ACTIVE', 'RETURNING'}
                    or type(trip.get('accepting_updates')) is not bool):
                raise ValueError('저장된 운행 정보가 유효하지 않습니다.')
            current_id = trip.get('current_request_id')
            if current_id is not None and (type(current_id) is not int or current_id not in request_ids):
                raise ValueError('저장된 현재 요청 ID가 유효하지 않습니다.')
        if next_trip <= max(recorded_trip_ids, default=0):
            raise ValueError('다음 운행 ID가 기존 ID와 충돌합니다.')
        deferred = data.get('deferred_history_ids', [])
        if (not isinstance(deferred, list)
                or any(type(item) is not int or item not in ids for item in deferred)
                or len(set(deferred)) != len(deferred)
                or any(request['status'] not in {'DELIVERED', 'CANCELED'}
                       for request in requests if request['id'] in deferred)):
            raise ValueError('저장된 배송 기록 초기화 예약이 유효하지 않습니다.')
        validated = deepcopy(data)
        validated['deferred_history_ids'] = list(deferred)
        return validated

    def _persist(self):
        if self.storage_file is None:
            return
        target = self.storage_file
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(
                    mode='w', encoding='utf-8', dir=target.parent,
                    prefix=f'.{target.name}.', suffix='.tmp', delete=False) as stream:
                temporary = Path(stream.name)
                json.dump(self._data, stream, ensure_ascii=False, allow_nan=False)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, target)
            temporary = None
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)

    def _change(self, operation):
        with self._lock:
            original = deepcopy(self._data)
            try:
                result = operation()
                self._persist()
            except Exception:
                self._data = original
                raise
            return deepcopy(result)

    def _request(self, request_id):
        self._positive_id(request_id, '요청 ID')
        for request in self._data['requests']:
            if request['id'] == request_id:
                return request
        raise ValueError('존재하지 않는 요청입니다.')

    def snapshot(self):
        with self._lock:
            return deepcopy({'requests': self._data['requests'],
                             'active_trip': self._data['active_trip'],
                             'deferred_history_ids': self._data['deferred_history_ids']})

    def submit(self, user_id, payload):
        user_id, fields = self._user_id(user_id), self._payload(payload)
        with self._lock:
            if fields['client_request_id']:
                for request in self._data['requests']:
                    if (request['user_id'] == user_id
                            and request['client_request_id'] == fields['client_request_id']):
                        return deepcopy(request)

            def create():
                now = time.time()
                request = dict(fields, id=self._data['next_request_id'],
                               user_id=user_id, created_at=now, updated_at=now,
                               status='PENDING', trip_id=None)
                self._data['next_request_id'] += 1
                self._data['requests'].append(request)
                return request

            return self._change(create)

    def cancel_pending(self, user_id, request_id):
        self._user_id(user_id)

        def cancel():
            request = self._request(request_id)
            if request['user_id'] != user_id:
                raise PermissionError('본인의 재고 요청만 취소할 수 있습니다.')
            if request['status'] != 'PENDING':
                raise ValueError('아직 배차되지 않은 요청만 취소할 수 있습니다.')
            request.update(status='CANCELED', updated_at=time.time())
            return request

        return self._change(cancel)

    def delete_finished_history(self, user_id, request_id):
        """Delete one manager-owned finished record without changing a trip.

        A finished request referenced by an active trip must remain until the
        trip closes at verified Home. Reserve that ID for deferred deletion.
        """
        self._user_id(user_id)

        def delete():
            request = self._request(request_id)
            if request['user_id'] != user_id:
                raise PermissionError('본인의 배송 기록만 삭제할 수 있습니다.')
            if request['status'] not in {'DELIVERED', 'CANCELED'}:
                raise ValueError('배송 완료·취소된 기록만 삭제할 수 있습니다.')
            trip = self._data['active_trip']
            deferred = set(self._data['deferred_history_ids'])
            if (trip is not None and request_id in trip['request_ids']) or request_id in deferred:
                deferred.add(request_id)
                self._data['deferred_history_ids'] = sorted(deferred)
                return {'deleted': False, 'deferred': True}
            self._data['requests'] = [item for item in self._data['requests']
                                      if item['id'] != request_id]
            return {'deleted': True, 'deferred': False}

        return self._change(delete)

    def clear_finished_history(self, *, defer_active_trip=False):
        """Remove all managers' unreferenced, finished request records.

        An active trip can still contain a delivered request. Keep all of its
        referenced records, plus unfinished work. With ``defer_active_trip``,
        remember the exact finished IDs for deletion after verified Home.
        Request IDs never reuse removed records' identities.
        """
        def clear():
            trip = self._data['active_trip']
            referenced = set(trip['request_ids']) if trip is not None else set()
            if defer_active_trip:
                deferred = set(self._data['deferred_history_ids'])
                deferred.update(
                    request['id'] for request in self._data['requests']
                    if request['id'] in referenced and request['status'] in {'DELIVERED', 'CANCELED'})
                self._data['deferred_history_ids'] = sorted(deferred)
            protected = referenced | set(self._data['deferred_history_ids'])
            before = len(self._data['requests'])
            self._data['requests'] = [
                request for request in self._data['requests']
                if not (request['id'] not in protected
                        and request['status'] in {'DELIVERED', 'CANCELED'})
            ]
            return before - len(self._data['requests'])

        return self._change(clear)

    def start_trip(self, request_ids=None):
        """Explicit operator dispatch; a selection is still ordered by FIFO ID."""
        def start():
            if self._data['active_trip'] is not None:
                raise ValueError('홈 복귀를 완료한 뒤 새 운행을 시작하세요.')
            pending = [item for item in self._data['requests'] if item['status'] == 'PENDING']
            if request_ids is not None:
                if (not isinstance(request_ids, list) or not request_ids
                        or len(request_ids) > 100
                        or any(type(item) is not int for item in request_ids)
                        or len(set(request_ids)) != len(request_ids)):
                    raise ValueError('배차할 요청 ID는 중복 없는 1~100개 정수 배열이어야 합니다.')
                available = {item['id'] for item in pending}
                if any(item not in available for item in request_ids):
                    raise ValueError('대기 중인 요청만 배차할 수 있습니다.')
                selected = set(request_ids)
                pending = [item for item in pending if item['id'] in selected]
            pending = sorted(pending, key=lambda item: item['id'])[:100]
            if not pending:
                raise ValueError('배차할 재고 요청이 없습니다.')
            trip_id = self._data['next_trip_id']
            self._data['next_trip_id'] += 1
            now = time.time()
            for index, request in enumerate(pending):
                request.update(status='QUEUED' if index else 'IN_PROGRESS',
                               trip_id=trip_id, updated_at=now)
            trip = {'id': trip_id, 'request_ids': [item['id'] for item in pending],
                    'current_request_id': pending[0]['id'], 'status': 'ACTIVE',
                    'accepting_updates': True}
            self._data['active_trip'] = trip
            return trip

        return self._change(start)

    def append_to_trip(self, request_id):
        """Append one pending request; automatic home return can be interrupted."""
        def append():
            request = self._request(request_id)
            trip = self._data['active_trip']
            if trip is None or not trip['accepting_updates']:
                raise ValueError('현재 운행에는 요청을 추가할 수 없습니다.')
            if request_id in trip['request_ids'] and request['trip_id'] == trip['id']:
                return trip
            if request['status'] != 'PENDING':
                raise ValueError('대기 중인 요청만 운행에 추가할 수 있습니다.')
            trip['request_ids'].append(request_id)
            is_next = trip['current_request_id'] is None
            request.update(status='IN_PROGRESS' if is_next else 'QUEUED',
                           trip_id=trip['id'], updated_at=time.time())
            if is_next:
                trip.update(current_request_id=request_id, status='ACTIVE')
            return trip

        return self._change(append)

    def complete_request(self, request_id):
        """Record externally verified delivery, never just goal publication."""
        def complete():
            request = self._request(request_id)
            trip = self._data['active_trip']
            if (trip is None or trip['current_request_id'] != request_id
                    or request['status'] != 'IN_PROGRESS'):
                raise ValueError('현재 배송 중인 요청만 완료할 수 있습니다.')
            now = time.time()
            request.update(status='DELIVERED', updated_at=now)
            queued = [self._request(item) for item in trip['request_ids']
                      if self._request(item)['status'] == 'QUEUED']
            if queued:
                queued[0].update(status='IN_PROGRESS', updated_at=now)
                trip.update(current_request_id=queued[0]['id'], status='ACTIVE')
            else:
                trip.update(current_request_id=None, status='RETURNING')
            return trip

        return self._change(complete)

    def freeze_updates(self):
        """Use for explicit home/cancel/emergency, never for automatic return."""
        def freeze():
            trip = self._data['active_trip']
            if trip is not None:
                trip['accepting_updates'] = False
            return trip

        return self._change(freeze)

    def cancel_trip(self):
        """Unfulfilled work returns to PENDING; keep the trip until home arrival."""
        def cancel():
            trip = self._data['active_trip']
            if trip is None:
                return None
            now = time.time()
            for request_id in trip['request_ids']:
                request = self._request(request_id)
                if request['status'] in {'QUEUED', 'IN_PROGRESS'}:
                    request.update(status='PENDING', trip_id=None, updated_at=now)
            trip.update(current_request_id=None, status='RETURNING', accepting_updates=False)
            return trip

        return self._change(cancel)

    def close_trip(self):
        """Caller must independently confirm Homepoint arrival before calling."""
        def close():
            trip = self._data['active_trip']
            if trip is not None:
                if trip['status'] != 'RETURNING' or trip['current_request_id'] is not None:
                    raise ValueError('배송을 마치거나 취소한 뒤 홈 복귀를 완료하세요.')
                self._data['active_trip'] = None

        return self._change(close)

    def clear_deferred_history_at_home(self):
        """Delete only IDs reserved by a reset, after the caller verifies Home."""
        with self._lock:
            if not self._data['deferred_history_ids']:
                return 0

            def clear():
                if self._data['active_trip'] is not None:
                    raise ValueError('홈 복귀 운행을 종료한 뒤 예약된 기록을 정리하세요.')
                deferred = set(self._data['deferred_history_ids'])
                before = len(self._data['requests'])
                self._data['requests'] = [item for item in self._data['requests']
                                          if item['id'] not in deferred]
                self._data['deferred_history_ids'] = []
                return before-len(self._data['requests'])

            return self._change(clear)
