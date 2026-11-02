"""Bounded, sensor-gated final yaw trim after a confirmed Home navigation result.

This module makes no ROS calls.  The bridge owns the stop/motion handshakes and
feeds it only current evidence.  A terminal FollowPath success is not proof of
the final physical heading.
"""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class HomeYawDecision:
    phase: str
    angular: float = 0.0
    reason: str = ''


class HomeYawTrim:
    def __init__(self, *, accepted_yaw_error=.15, trim_target=.12,
                 gain=.8, max_speed=.15, timeout=8., settle_time=.3,
                 max_initial_error=.8, max_rotation=.70):
        self.accepted_yaw_error = accepted_yaw_error
        self.trim_target = trim_target
        self.gain = gain
        self.max_speed = max_speed
        self.timeout = timeout
        self.settle_time = settle_time
        self.max_initial_error = max_initial_error
        self.max_rotation = max_rotation
        self.reset()

    def reset(self):
        self.goal_stamp = None
        self.started_at = None
        self.start_odom_yaw = None
        self.rotating = False
        self.settled_since = None
        self.phase = 'IDLE'

    @property
    def active(self):
        return self.phase in {'WAIT_STOP', 'WAIT_SAFE', 'TURNING', 'SETTLING'}

    def start(self, goal_stamp, now):
        if self.active and goal_stamp == self.goal_stamp:
            return
        self.reset()
        self.goal_stamp = goal_stamp
        self.started_at = now
        self.phase = 'WAIT_STOP'

    def step(self, *, now, goal_stamp, yaw_error, xy_error, xy_tolerance,
             odom_yaw, sensors_fresh, scan_clear, health_ready,
             control_ready, stop_ack, stopped, operator_clear):
        if not self.active or goal_stamp != self.goal_stamp:
            return HomeYawDecision('IDLE')
        if now-self.started_at >= self.timeout:
            self.phase = 'FAILED'
            return HomeYawDecision('FAILED', reason='홈 방향 보정 제한 시간을 넘었습니다.')
        if not operator_clear:
            self.phase = 'FAILED'
            return HomeYawDecision('FAILED', reason='조종기·위치 확인 또는 비상정지가 개입했습니다.')
        if not sensors_fresh:
            self.phase = 'WAIT_SAFE'
            self.settled_since = None
            return HomeYawDecision('WAIT_SAFE', reason='최신 TF·라이다·오도메트리를 기다립니다.')
        if (not all(isinstance(v, (int, float)) and not isinstance(v, bool) and isfinite(v)
                    for v in (now, yaw_error, xy_error, xy_tolerance, odom_yaw))
                or xy_tolerance <= 0. or now < self.started_at):
            self.phase = 'FAILED'
            return HomeYawDecision('FAILED', reason='홈 자세 입력이 유효하지 않습니다.')
        if xy_error > xy_tolerance:
            self.phase = 'FAILED'
            return HomeYawDecision('FAILED', reason='홈 위치 오차가 허용 범위를 넘습니다. 방향만 보정하지 않습니다.')
        if not health_ready:
            self.phase = 'WAIT_SAFE'
            self.settled_since = None
            return HomeYawDecision('WAIT_SAFE', reason='위치 신뢰도와 주행 허용 상태를 기다립니다.')

        if self.start_odom_yaw is not None:
            from math import atan2, cos, sin
            delta = atan2(sin(odom_yaw-self.start_odom_yaw), cos(odom_yaw-self.start_odom_yaw))
            if abs(delta) > self.max_rotation:
                self.phase = 'FAILED'
                return HomeYawDecision('FAILED', reason='허용된 제자리 회전 각도를 넘었습니다.')

        target = self.trim_target if self.rotating else self.accepted_yaw_error
        if abs(yaw_error) <= target:
            self.phase = 'SETTLING'
            if not (stopped and stop_ack):
                self.settled_since = None
                return HomeYawDecision('SETTLING', reason='회전을 멈추고 차량 정지를 확인합니다.')
            if self.settled_since is None:
                self.settled_since = now
            if now-self.settled_since >= self.settle_time:
                self.phase = 'DONE'
                return HomeYawDecision('DONE')
            return HomeYawDecision('SETTLING', reason='홈 방향과 차량 정지를 연속 확인합니다.')

        self.settled_since = None
        if not scan_clear:
            self.phase = 'FAILED'
            return HomeYawDecision('FAILED', reason='차체 회전 반경의 라이다 여유가 부족합니다.')
        if abs(yaw_error) > self.max_initial_error and not self.rotating:
            self.phase = 'FAILED'
            return HomeYawDecision('FAILED', reason='홈 방향 오차가 커서 자동 제자리 보정을 중단합니다.')
        if not control_ready:
            self.phase = 'WAIT_STOP'
            return HomeYawDecision('WAIT_STOP', reason='제어권과 정지 응답을 기다립니다.')
        if not self.rotating:
            if not (stop_ack and stopped):
                self.phase = 'WAIT_STOP'
                return HomeYawDecision('WAIT_STOP', reason='회전 전 실제 정지를 확인합니다.')
            self.start_odom_yaw = odom_yaw
            self.rotating = True
        self.phase = 'TURNING'
        angular = max(-self.max_speed, min(self.max_speed, self.gain*yaw_error))
        return HomeYawDecision('TURNING', angular, '홈 방향으로 저속 정렬 중입니다.')
