"""Ordered, single-owner joystick lease; independent of ROS and mission storage."""
import math
import re
import time

from smart_vision_amr.mission_logic import MissionError


class ManualOverride:
    def __init__(self, clock=None):
        self.clock = clock or (lambda: time.monotonic())
        self.state = 'IDLE'
        self.owner = self.control_id = None
        self.seq = 0
        self.deadline = 0.
        self.linear = self.angular = 0.
        self.resume_mission = False
        self.reason = ''
        self.closed = {}
        self.max_linear, self.max_angular = .12, .60

    @property
    def active(self):
        return self.state in {'ACQUIRING', 'ACTIVE'}

    @property
    def in_progress(self):
        return self.state != 'IDLE'

    def _close(self, owner, control_id, seq):
        self.closed[(owner, control_id)] = (seq, self.clock())
        if len(self.closed) > 512:
            self.closed.pop(next(iter(self.closed)))

    def request(self, owner, payload):
        control_id, seq, active = payload.get('control_id'), payload.get('seq'), payload.get('active')
        if (not isinstance(control_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]{8,128}', control_id)
                or type(seq) is not int or not 1 <= seq <= 2**53-1 or type(active) is not bool):
            raise MissionError('조종 세션과 순서 번호가 유효하지 않습니다.')
        values = []
        for key in ('linear', 'angular'):
            value = payload.get(key, 0.)
            if type(value) not in (int, float) or not math.isfinite(value):
                raise MissionError('조종 속도는 유한한 숫자여야 합니다.')
            values.append(float(value))
        self.expire()
        key = (owner, control_id)
        if key in self.closed:
            return 'ignored'
        matching = self.owner == owner and self.control_id == control_id
        if matching and seq <= self.seq:
            return 'ignored'
        if not active:
            # A release may overtake the initial press HTTP request.
            self._close(owner, control_id, seq)
            if not matching:
                return 'ignored'
            self.seq = seq
            self.release('조이스틱을 놓아 기존 임무 재개를 준비합니다.')
            return 'release'
        if self.active and not matching and self.owner != owner:
            raise MissionError('다른 운용자가 조이스틱을 사용 중입니다.')
        if not matching:
            if self.control_id is not None:
                self._close(self.owner, self.control_id, self.seq)
            self.owner, self.control_id = owner, control_id
            self.state, self.seq = 'ACQUIRING', seq
            result = 'acquire'
        else:
            self.seq = seq
            result = 'update'
        self.deadline = self.clock()+.5
        self.linear = max(-self.max_linear, min(self.max_linear, values[0]))
        self.angular = max(-self.max_angular, min(self.max_angular, values[1]))
        return result

    def mark_acquired(self):
        if self.state == 'ACQUIRING':
            self.state, self.reason = 'ACTIVE', '조이스틱 우선 조종 중입니다.'

    def release(self, reason='조종 신호가 끊겨 정지 후 기존 임무를 재개합니다.'):
        if self.active:
            self._close(self.owner, self.control_id, self.seq)
            self.state, self.reason = 'RELEASING', reason
            self.linear = self.angular = 0.

    def expire(self):
        if self.active and self.clock() >= self.deadline:
            self.release()
            return True
        return False

    def finish(self):
        if self.control_id is not None:
            self._close(self.owner, self.control_id, self.seq)
        self.state, self.owner, self.control_id = 'IDLE', None, None
        self.linear = self.angular = 0.
        self.deadline = 0.
        self.resume_mission = False
        self.reason = ''

    def snapshot(self, owner=None, available=False, reason=''):
        return {'state': self.state, 'active': self.active,
                'owner': owner is not None and owner == self.owner,
                'available': bool(available), 'reason': self.reason or reason,
                'linear': self.linear if self.state == 'ACTIVE' else 0.,
                'angular': self.angular if self.state == 'ACTIVE' else 0.,
                'expires_in': max(0., self.deadline-self.clock()) if self.active else 0.,
                'limits': {'max_linear': self.max_linear, 'max_angular': self.max_angular}}
