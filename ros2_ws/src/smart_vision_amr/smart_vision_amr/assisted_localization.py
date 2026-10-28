"""Bounded, stationary-translation AMCL acquisition independent of ROS.

This module never publishes a pose or velocity.  The bridge must acknowledge
navigation cancellation and actuator ownership before calling ``acquired`` and
the AMCL global-localization service before calling ``initialized``.  A small
covariance alone cannot prove localization: the caller also supplies a scan/map
consistency check through ``quality_ok``.
"""

import math
import time


def rotation_scan_clearance(ranges, range_min, range_max, clearance=.30,
                            min_readings=8):
    """Return ``(safe, nearest_metres)`` for a full-circle stationary turn.

    The caller checks scan freshness and angular coverage.  Infinite readings
    are not obstacles, but a scan without enough measured returns cannot verify
    clearance or localization.  A zero or below-minimum return is conservative:
    an object inside the sensor's near limit must not be interpreted as open.
    """
    if (not all(math.isfinite(v) for v in (range_min, range_max, clearance))
            or range_min < 0 or range_max <= range_min or clearance <= 0
            or type(min_readings) is not int or min_readings < 1):
        return False, None
    measured = []
    too_near = False
    for value in ranges:
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            continue
        if math.isnan(value):
            continue
        if value <= range_min:
            too_near = True
            continue
        if math.isfinite(value) and value <= range_max:
            measured.append(float(value))
    nearest = 0.0 if too_near else (min(measured) if measured else None)
    return (not too_near and len(measured) >= min_readings
            and nearest >= clearance), nearest


class AssistedLocalization:
    """One slow sweep followed by a checked stationary localization result.

    ``tick`` returns the desired angular velocity in rad/s, with linear velocity
    always zero in the integrating bridge.  All times are monotonic receipt
    times, never ROS or sensor-header timestamps.  Terminal states remain visible
    until another explicit ``start``; mission resumption is deliberately absent.
    """

    ACTIVE_STATES = {'ACQUIRING', 'INITIALIZING', 'ROTATING', 'SETTLING'}
    TERMINAL_STATES = {'SUCCEEDED', 'FAILED', 'CANCELED'}

    def __init__(self, clock=None, angular_speed=.20,
                 max_rotation=2.0 * math.pi, min_rotation=math.pi / 2.0,
                 timeout=65.0, sensor_timeout=1.0, estimate_timeout=2.5,
                 stable_duration=2.0, min_estimates=5,
                 position_std_limit=.25, yaw_std_limit=.26,
                 settle_duration=1.0, acquire_timeout=8.0,
                 initialize_timeout=10.0, motion_timeout=6.0):
        values = (angular_speed, max_rotation, min_rotation, timeout,
                  sensor_timeout, estimate_timeout, stable_duration,
                  position_std_limit, yaw_std_limit, settle_duration,
                  acquire_timeout, initialize_timeout, motion_timeout)
        if (not all(type(value) in (int, float) and math.isfinite(value)
                    and value > 0 for value in values)
                or angular_speed > .35 or max_rotation > 2.0 * math.pi
                or min_rotation > max_rotation
                or type(min_estimates) is not int or min_estimates < 2):
            raise ValueError('Invalid assisted-localization limits')
        self.clock = clock or time.monotonic
        self.angular_speed = float(angular_speed)
        self.max_rotation, self.min_rotation = max_rotation, min_rotation
        self.timeout, self.sensor_timeout = timeout, sensor_timeout
        self.estimate_timeout, self.stable_duration = estimate_timeout, stable_duration
        self.min_estimates = min_estimates
        self.position_std_limit, self.yaw_std_limit = position_std_limit, yaw_std_limit
        self.settle_duration = settle_duration
        self.acquire_timeout, self.initialize_timeout = acquire_timeout, initialize_timeout
        self.motion_timeout = motion_timeout
        self.state = 'IDLE'
        self.reason = ''
        self.angular = 0.0
        self.started_at = self.state_since = None
        self.rotation = 0.0
        self.estimate = None
        self.last_odom_at = self.last_estimate_at = None
        self._last_yaw = None
        self._last_motion_at = None
        self._good_since = None
        self._good_count = 0
        self._last_good_pose = None
        self._last_good_at = None
        self._settle_updates = 0

    @property
    def active(self):
        return self.state in self.ACTIVE_STATES

    @property
    def succeeded(self):
        return self.state == 'SUCCEEDED'

    def _now(self, now):
        value = self.clock() if now is None else now
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError('Monotonic time must be finite')
        return float(value)

    def _transition(self, state, reason, now):
        self.state, self.reason, self.state_since = state, reason, now
        self.angular = 0.0

    def start(self, now=None):
        if self.active:
            return False
        now = self._now(now)
        self.started_at = now
        self.rotation = 0.0
        self.estimate = None
        self.last_odom_at = self.last_estimate_at = None
        self._last_yaw = self._last_motion_at = None
        self._clear_confidence()
        self._settle_updates = 0
        self._transition('ACQUIRING', '주행을 멈추고 위치 확인 제어권을 확보합니다.', now)
        return True

    def acquired(self, now=None):
        if self.state != 'ACQUIRING':
            return False
        self._transition('INITIALIZING', '저장된 지도에서 AMCL 위치 가설을 초기화합니다.', self._now(now))
        return True

    def initialized(self, now=None):
        if self.state != 'INITIALIZING':
            return False
        now = self._now(now)
        # Old low-covariance estimates must not survive global initialization.
        self.rotation = 0.0
        self.estimate = None
        self.last_estimate_at = self.last_odom_at = None
        self._last_yaw = None
        self._last_motion_at = now
        self._clear_confidence()
        self._transition('ROTATING', '천천히 회전하며 라이다와 지도를 대조합니다.', now)
        return True

    def _clear_confidence(self):
        self._good_since = self._last_good_at = None
        self._good_count = 0
        self._last_good_pose = None

    def pause_for_sensor_gap(self, now=None):
        """Stop a blind sweep without counting unobserved yaw on recovery."""
        if self.state not in {'ROTATING', 'SETTLING'}:
            return
        now = self._now(now)
        self.angular = 0.0
        self.last_odom_at = self._last_yaw = None
        self.last_estimate_at = None
        self.estimate = None
        self._clear_confidence()
        self._settle_updates = 0
        self._last_motion_at = now

    def on_odom(self, yaw, now=None):
        if self.state not in {'ROTATING', 'SETTLING'}:
            return False
        now = self._now(now)
        if type(yaw) not in (int, float) or not math.isfinite(yaw):
            self.fail('오도메트리 방향 값이 유효하지 않아 위치 확인을 중단합니다.', now)
            return False
        if self.last_odom_at is not None and now <= self.last_odom_at:
            return False
        if self.last_odom_at is not None:
            dt = now - self.last_odom_at
            if dt > self.sensor_timeout:
                self.fail('오도메트리 갱신이 끊겨 회전을 중단합니다.', now)
                return False
            delta = math.atan2(math.sin(yaw - self._last_yaw),
                               math.cos(yaw - self._last_yaw))
            # Reject frame resets and pose jumps instead of counting them as a
            # completed sweep.  Ordinary yaw wrap at +/-pi remains continuous.
            limit = max(.12, self.angular_speed * dt * 3.0 + .04)
            if abs(delta) > min(.5, limit):
                self.fail('오도메트리 방향이 갑자기 변해 위치 확인을 중단합니다.', now)
                return False
            if self.state == 'ROTATING':
                self.rotation += abs(delta)
                if abs(delta) >= .002:
                    self._last_motion_at = now
        self.last_odom_at, self._last_yaw = now, float(yaw)
        return True

    def on_estimate(self, x, y, yaw, position_std, yaw_std, now=None, *, quality_ok=True):
        if self.state not in {'ROTATING', 'SETTLING'}:
            return False
        now = self._now(now)
        if self.last_estimate_at is not None and now <= self.last_estimate_at:
            return False
        values = (x, y, yaw, position_std, yaw_std)
        if (not all(type(value) in (int, float) and math.isfinite(value) for value in values)
                or position_std < 0 or yaw_std < 0):
            self._clear_confidence()
            return False
        previous_at = self.last_estimate_at
        self.last_estimate_at = now
        self.estimate = {'x': float(x), 'y': float(y), 'yaw': float(yaw),
                         'position_std': float(position_std), 'yaw_std': float(yaw_std),
                         'quality_ok': bool(quality_ok)}
        good = (quality_ok is True and position_std <= self.position_std_limit
                and yaw_std <= self.yaw_std_limit)
        if not good:
            self._clear_confidence()
            return True
        if previous_at is not None and now - previous_at > self.estimate_timeout:
            self._clear_confidence()
        if self._last_good_pose is not None:
            last_x, last_y, last_yaw = self._last_good_pose
            yaw_delta = abs(math.atan2(math.sin(yaw-last_yaw), math.cos(yaw-last_yaw)))
            dt = now - self._last_good_at
            if (math.hypot(x-last_x, y-last_y) > .35
                    or yaw_delta > max(.5, dt * self.angular_speed * 3.0)):
                self._clear_confidence()
        if self._good_since is None:
            self._good_since = now
        self._good_count += 1
        self._last_good_pose, self._last_good_at = (x, y, yaw), now
        if self.state == 'SETTLING' and now > self.state_since:
            self._settle_updates += 1
        return True

    def _confident(self, now):
        return (self._good_since is not None
                and self.last_estimate_at is not None
                and 0 <= now - self.last_estimate_at <= self.estimate_timeout
                and self._good_count >= self.min_estimates
                and now - self._good_since >= self.stable_duration)

    def tick(self, now=None, *, sensors_ready=True, rotation_clear=True):
        now = self._now(now)
        self.angular = 0.0
        if not self.active:
            return self.angular
        if not sensors_ready:
            self.fail('라이다 또는 오도메트리 수신이 끊겨 위치 확인을 중단합니다.', now)
        elif not rotation_clear:
            self.fail('차체 주변 회전 공간이 부족하여 위치 확인을 중단합니다.', now)
        elif now - self.started_at >= self.timeout:
            self.fail('제한 시간 안에 위치를 확인하지 못했습니다. 수동 초기 위치를 지정하세요.', now)
        elif self.state == 'ACQUIRING' and now - self.state_since >= self.acquire_timeout:
            self.fail('주행 정지와 제어권 확보를 확인하지 못했습니다.', now)
        elif self.state == 'INITIALIZING' and now - self.state_since >= self.initialize_timeout:
            self.fail('AMCL 위치 초기화 응답이 없어 위치 확인을 중단합니다.', now)
        if not self.active or self.state in {'ACQUIRING', 'INITIALIZING'}:
            return self.angular
        if self.last_odom_at is None:
            if now - self.state_since >= self.sensor_timeout:
                self.fail('새 오도메트리 방향을 받지 못해 회전하지 않습니다.', now)
            return self.angular
        if now - self.last_odom_at > self.sensor_timeout:
            self.fail('오도메트리 갱신이 끊겨 회전을 중단합니다.', now)
            return self.angular
        if self.last_estimate_at is not None and now - self.last_estimate_at > self.estimate_timeout:
            self._clear_confidence()
        if self.state == 'ROTATING':
            if (self.rotation >= self.max_rotation
                    or (self.rotation >= self.min_rotation and self._confident(now))):
                self._settle_updates = 0
                self._transition('SETTLING', '회전을 멈추고 지도상 위치의 안정성을 확인합니다.', now)
            elif now - self._last_motion_at >= self.motion_timeout:
                self.fail('회전 명령 후 실제 차체 회전을 확인하지 못했습니다.', now)
            else:
                self.angular = self.angular_speed
        elif (now - self.state_since >= self.settle_duration
              and self._settle_updates >= 1 and self._confident(now)):
            self._transition('SUCCEEDED', '라이다와 지도 대조로 위치를 확인했습니다. 임무는 일시정지 상태입니다.', now)
        return self.angular

    def fail(self, reason, now=None):
        if self.active:
            self._transition('FAILED', str(reason), self._now(now))

    def cancel(self, reason='위치 확인을 취소했습니다. 차량은 정지합니다.', now=None):
        if self.active:
            self._transition('CANCELED', str(reason), self._now(now))
            return True
        return False

    def snapshot(self, now=None):
        now = self._now(now)
        return {'state': self.state, 'active': self.active, 'reason': self.reason,
                'angular': self.angular if self.state == 'ROTATING' else 0.0,
                'rotation_degrees': math.degrees(min(self.rotation, self.max_rotation)),
                'progress': min(1.0, self.rotation / self.max_rotation),
                'elapsed': max(0.0, now - self.started_at) if self.started_at is not None else 0.0,
                'estimate': dict(self.estimate) if self.estimate is not None else None,
                'good_estimates': self._good_count,
                'quality_ok': bool(self.estimate and self.estimate['quality_ok'])}
