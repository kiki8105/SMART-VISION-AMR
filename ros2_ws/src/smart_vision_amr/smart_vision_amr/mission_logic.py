"""Small deterministic mission state machine; deliberately independent of ROS/HTTP."""
from copy import deepcopy
from math import atan2, cos, hypot, isfinite, sin
import time

ACTIVE_STATES = frozenset({'NAVIGATING', 'RETURNING_HOME', 'WAIT_REPLAN', 'CONFIRMING'})
START_STATES = frozenset({'IDLE', 'COMPLETED', 'HOME_REACHED', 'CANCELED', 'ERROR'})


def stamp_key(value):
    """Normalize JSON/ROS goal stamps without relying on wall/simulation clock order."""
    if isinstance(value, dict):
        return int(value.get('sec', 0)), int(value.get('nanosec', 0))
    if isinstance(value, (list, tuple)) and len(value) == 2:
        return int(value[0]), int(value[1])
    return None


class MissionError(ValueError):
    pass


class MarkerMission:
    def __init__(self, markers, route=None, confirmations=3, pose_tolerance=.30,
                 yaw_tolerance=.35, clock=time.monotonic):
        self.markers = {int(item['id']): dict(item) for item in markers}
        self.route = self.validate_route(route or [2, 3])
        self.pending_route = None
        self.current_index = 0
        self.completed_ids = []
        self.state = 'IDLE'
        self.reason = '경유 순서를 확인한 뒤 임무를 시작하세요.'
        self.estop_latched = False
        self.goal_stamp = None
        self.home = None
        self.returning_home = False
        self.auto_return_home = False
        self.auto_returning_home = False
        self.home_navigation_succeeded = False
        self.confirmations = 0
        self.required_confirmations = max(1, int(confirmations))
        self.pose_tolerance = float(pose_tolerance)
        self.yaw_tolerance = float(yaw_tolerance)
        self.clock = clock
        self.observation_since = None
        self.last_frame = None
        self.commands = []
        self.events = []
        self.revision = 0

    def validate_route(self, route):
        if not isinstance(route, list) or not route or len(route) > 100:
            raise MissionError('경유 순서는 1~100개의 마커 ID 배열이어야 합니다.')
        if any(type(item) is not int or item not in self.markers for item in route):
            raise MissionError('등록된 정수 마커 ID만 사용할 수 있습니다.')
        return list(route)

    @property
    def expected_id(self):
        if self.returning_home:
            return None
        return self.route[self.current_index] if self.current_index < len(self.route) else None

    def event(self, message, level='info'):
        self.events.append({'at': time.time(), 'level': level, 'message': message})
        self.events = self.events[-60:]
        self.revision += 1

    def update_route(self, route):
        route = self.validate_route(route)
        if self.state in ACTIVE_STATES or self.state == 'PAUSED' or self.returning_home:
            self.pending_route = route
            self.event('새 경유 순서를 저장했습니다. 다음 시작 시 적용됩니다.')
        else:
            self.route, self.pending_route = route, None
            self.current_index, self.completed_ids = 0, []
            self.goal_stamp = None
            self.confirmations = 0
            self.last_frame = None
            self.observation_since = None
            if not self.estop_latched:
                self.state = 'IDLE'
                self.reason = '새 경유 순서를 저장했습니다. 시작 버튼을 누르면 출발합니다.'
            self.event('경유 순서를 저장했습니다.')

    def start(self, home=None, auto_return_home=False):
        if self.estop_latched:
            raise MissionError('비상정지를 먼저 해제하세요. 해제 후에도 시작 버튼이 필요합니다.')
        if self.state not in START_STATES:
            raise MissionError('진행 중인 임무를 취소하거나 완료한 뒤 새 임무를 시작하세요.')
        if home is not None:
            try:
                home = {key: float(home[key]) for key in ('x', 'y', 'yaw')}
                if not all(isfinite(value) for value in home.values()):
                    raise ValueError()
            except (KeyError, TypeError, ValueError):
                raise MissionError('홈으로 기록할 현재 map 위치가 유효하지 않습니다.')
        if auto_return_home and home is None:
            raise MissionError('자동 복귀 임무에는 고정된 홈 위치가 필요합니다.')
        self.home = deepcopy(home)
        self.returning_home = False
        self.auto_return_home = bool(auto_return_home)
        self.auto_returning_home = False
        self.home_navigation_succeeded = False
        if self.pending_route is not None:
            self.route, self.pending_route = self.pending_route, None
        self.current_index, self.completed_ids = 0, []
        self.event('사용자가 임무를 시작했습니다.')
        self._send_current_goal()

    def append_waypoints(self, marker_ids):
        """Extend a delivery mission without replacing its active navigation goal.

        Only an automatic completion return may be interrupted by another
        delivery. A manual home/cancel request closes this extension window.
        Paused missions always require an explicit resume, including when new
        deliveries replace their paused automatic return.
        """
        marker_ids = self.validate_route(marker_ids)
        if self.estop_latched:
            raise MissionError('비상정지 중에는 배달 목표를 추가 실행할 수 없습니다.')
        if (not self.auto_return_home
                or self.state not in ACTIVE_STATES | {'PAUSED'}
                or (self.returning_home and not self.auto_returning_home)):
            raise MissionError('진행 중인 자동 복귀 배달 임무에만 목표를 추가할 수 있습니다.')
        if len(self.route) + len(marker_ids) > 100:
            raise MissionError('한 임무의 경유 마커는 최대 100개까지 추가할 수 있습니다.')
        was_auto_returning = self.auto_returning_home
        self.route.extend(marker_ids)
        self.event(f'추가 배달 목표 {marker_ids}을 현재 임무 뒤에 연결했습니다.')
        if not was_auto_returning:
            return
        self.returning_home = False
        self.auto_returning_home = False
        self.home_navigation_succeeded = False
        self.goal_stamp = None
        # Replace any not-yet-dispatched home command as well. The ROS adapter
        # must acknowledge cancellation before dispatching this new goal.
        self.commands = [('cancel', None), ('stop', None)]
        if self.state == 'PAUSED':
            self.reason = '추가 배달 목표를 대기열에 넣었습니다. 재개 버튼으로 출발합니다.'
        else:
            self._send_current_goal()

    def _send_current_goal(self):
        self.goal_stamp = None
        self.confirmations = 0
        self.last_frame = None
        self.observation_since = None
        if self.current_index >= len(self.route):
            if self.auto_return_home:
                self.commands = [('cancel', None), ('stop', None)]
                self.returning_home = True
                self.auto_returning_home = True
                self.event('모든 배달을 완료해 고정 홈으로 자동 복귀합니다.')
                self._send_home_goal()
                return
            self.state, self.reason = 'COMPLETED', '모든 경유 마커를 카메라로 확인했습니다.'
            self.commands.append(('stop', None))
            self.event(self.reason)
            return
        self.state = 'NAVIGATING'
        self.reason = f'마커 {self.expected_id} 관측 위치로 이동 중입니다.'
        self.commands.append(('goal', dict(self.markers[self.expected_id])))
        self.revision += 1

    def restart(self):
        """Explicitly replace a route without silently redefining mission home."""
        if self.estop_latched:
            raise MissionError('비상정지를 먼저 해제하세요.')
        if self.state != 'PAUSED':
            raise MissionError('임무를 일시정지한 뒤 새 경유 순서로 시작하세요.')
        if self.home is None:
            raise MissionError('기록된 홈이 없습니다. 새 임무 시작으로 현재 위치를 기록하세요.')
        self.commands = [('cancel', None), ('stop', None)]
        self.returning_home = False
        self.auto_returning_home = False
        self.home_navigation_succeeded = False
        if self.pending_route is not None:
            self.route, self.pending_route = self.pending_route, None
        self.current_index, self.completed_ids = 0, []
        self.event('홈 위치를 유지하고 새 경유 순서로 임무를 다시 시작합니다.')
        self._send_current_goal()

    def _send_home_goal(self):
        self.goal_stamp = None
        self.confirmations = 0
        self.last_frame = None
        self.observation_since = None
        self.home_navigation_succeeded = False
        self.state = 'RETURNING_HOME'
        self.reason = '임무 시작 시 기록한 홈 위치로 복귀 중입니다.'
        self.commands.append(('goal', dict(self.home)))
        self.revision += 1

    def return_home(self, canceled=False):
        if self.estop_latched:
            self.commands = [('cancel', None), ('stop', None)]
            raise MissionError('비상정지를 먼저 해제하세요. 비상정지는 홈 복귀를 시작하지 않습니다.')
        if self.home is None:
            if self.state in ACTIVE_STATES or self.state == 'PAUSED':
                self.state, self.reason = 'CANCELED', '기록된 홈이 없어 현재 임무를 정지했습니다.'
                self.goal_stamp = None
                self.commands = [('cancel', None), ('stop', None)]
                self.event(self.reason, 'warning')
            raise MissionError('기록된 홈이 없습니다. 임무를 시작하면 현재 map 위치를 홈으로 기록합니다.')
        # A user-directed return is final for this dispatch: later requests must
        # wait for a new explicit departure rather than diverting the robot.
        self.auto_return_home = False
        self.auto_returning_home = False
        if self.returning_home and self.state in ACTIVE_STATES:
            return
        self.commands = [('cancel', None), ('stop', None)]
        self.returning_home = True
        self.event('사용자가 임무를 취소하고 홈 복귀를 요청했습니다.' if canceled else '사용자가 홈 복귀를 요청했습니다.')
        self._send_home_goal()

    def goal_dispatched(self, stamp):
        if self.state in {'NAVIGATING', 'RETURNING_HOME'}:
            self.goal_stamp = stamp_key(stamp)

    def pause(self):
        if self.state not in ACTIVE_STATES:
            raise MissionError('진행 중인 임무만 일시정지할 수 있습니다.')
        self.state, self.reason = 'PAUSED', '사용자가 일시정지했습니다. 재개 버튼으로만 다시 출발합니다.'
        self.goal_stamp = None
        self.commands = [('cancel', None), ('stop', None)]
        self.event(self.reason)

    def resume(self):
        if self.estop_latched or self.state != 'PAUSED':
            raise MissionError('비상정지가 해제된 일시정지 상태에서만 재개할 수 있습니다.')
        self.event('사용자가 홈 복귀를 재개했습니다.' if self.returning_home else '사용자가 기존 경유 순서로 재개했습니다.')
        if self.returning_home:
            self._send_home_goal()
        else:
            self._send_current_goal()

    def cancel(self):
        self.return_home(canceled=True)

    def emergency_stop(self):
        self.estop_latched = True
        self.auto_return_home = False
        self.auto_returning_home = False
        self.state, self.reason = 'ESTOP', '비상정지 중입니다. 원인을 확인한 뒤 명시적으로 해제하세요.'
        self.goal_stamp = None
        self.commands = [('cancel', None), ('stop', None)]
        self.event(self.reason, 'critical')

    def reset_emergency(self):
        if not self.estop_latched:
            return
        self.estop_latched = False
        self.returning_home = False
        self.auto_returning_home = False
        self.home_navigation_succeeded = False
        self.state, self.reason = 'CANCELED', '비상정지를 해제했습니다. 새 시작 명령을 기다립니다.'
        # Reset acknowledges the latch; it must not erase an undelivered cancel.
        self.commands = [('cancel', None), ('stop', None)]
        self.event(self.reason)

    def on_navigation(self, payload, pose=None, stationary=False):
        if self.state not in ACTIVE_STATES or self.goal_stamp is None:
            return False
        if stamp_key(payload.get('goal_stamp')) != self.goal_stamp:
            return False
        status = str(payload.get('status', '')).upper()
        if status == 'SUCCEEDED':
            if self.returning_home:
                self.home_navigation_succeeded = True
                self.state = 'RETURNING_HOME'
                self.reason = '홈 주행 완료를 수신했습니다. 현재 위치와 정지를 확인합니다.'
                self.observe_home(pose, stationary)
            elif self.state != 'CONFIRMING':
                self.state = 'CONFIRMING'
                self.reason = f'도착했습니다. 카메라로 마커 {self.expected_id} 확인 중입니다.'
                self.observation_since = self.clock()
                self.confirmations = 0
                self.last_frame = None
                self.event(self.reason)
        elif status == 'RECOVERING':
            self.home_navigation_succeeded = False
            self.state = 'WAIT_REPLAN'
            self.reason = '안전한 후진으로 경로를 회복 중입니다. 기존 목표를 유지합니다.'
            self.event(self.reason, 'warning')
        elif status in {'FAILED', 'WAIT_REPLAN', 'NO_PATH', 'WAIT_POSE', 'WAIT_MAP',
                        'RETRYING', 'RECOVERY_SUCCEEDED', 'RECOVERY_FAILED',
                        'RECOVERY_STOPPED'}:
            self.home_navigation_succeeded = False
            self.state = 'WAIT_REPLAN'
            self.reason = '목표를 유지하고 정지 상태에서 경로 재계획을 기다립니다.'
            self.event(self.reason, 'warning')
        elif status in {'FOLLOWING', 'ACTIVE', 'PLANNED', 'NAVIGATING', 'MOVING', 'READY'}:
            if self.state != 'CONFIRMING':
                self.state = 'RETURNING_HOME' if self.returning_home else 'NAVIGATING'
                self.reason = '홈 위치로 복귀 중입니다.' if self.returning_home else f'마커 {self.expected_id} 관측 위치로 이동 중입니다.'
        elif status == 'INVALID_GOAL':
            self.state, self.reason = 'ERROR', '목표 좌표 또는 방향이 유효하지 않습니다.'
            self.commands = [('cancel', None), ('stop', None)]
            self.event(self.reason, 'warning')
        elif status == 'CANCELED' and payload.get('manual_stop'):
            self.state, self.reason = 'PAUSED', '외부 조종기에서 주행을 중지했습니다. 명시적으로 재개하세요.'
            self.goal_stamp = None
            self.commands = [('stop', None)]
            self.event(self.reason, 'warning')
        self.revision += 1
        return True

    def observe_home(self, pose, stationary):
        """A matching navigation success plus fresh stopped pose acknowledges home."""
        if (not self.returning_home or not self.home_navigation_succeeded
                or self.state != 'RETURNING_HOME' or pose is None or not stationary):
            return False
        distance = hypot(pose['x']-self.home['x'], pose['y']-self.home['y'])
        yaw_error = atan2(sin(pose['yaw']-self.home['yaw']), cos(pose['yaw']-self.home['yaw']))
        if distance > self.pose_tolerance or abs(yaw_error) > self.yaw_tolerance:
            return False
        self.state, self.reason = 'HOME_REACHED', '임무 시작 위치로 복귀를 완료했습니다.'
        self.returning_home = False
        self.auto_returning_home = False
        self.goal_stamp = None
        self.commands = [('stop', None)]
        self.event(self.reason)
        return True

    def stop_at_home(self, pose):
        """Cancel an old goal when the caller has verified a fresh stopped home pose.

        This is for an explicit at-home command, where issuing another home goal
        would make a robot that is already parked start moving again.  Unlike
        ``observe_home``, no navigation success is needed because no new goal is
        sent; the caller must independently verify fresh pose and odometry.
        """
        if self.estop_latched:
            raise MissionError('비상정지를 먼저 해제하세요.')
        if self.home is None or pose is None:
            raise MissionError('기록된 홈 위치와 현재 위치를 확인할 수 없습니다.')
        distance = hypot(pose['x']-self.home['x'], pose['y']-self.home['y'])
        yaw_error = atan2(sin(pose['yaw']-self.home['yaw']), cos(pose['yaw']-self.home['yaw']))
        if distance > self.pose_tolerance or abs(yaw_error) > self.yaw_tolerance:
            raise MissionError('현재 위치가 홈포인트와 일치하지 않습니다.')
        self.commands = [('cancel', None), ('stop', None)]
        self.state, self.reason = 'HOME_REACHED', '이미 홈포인트입니다.'
        self.returning_home = False
        self.auto_return_home = False
        self.auto_returning_home = False
        self.home_navigation_succeeded = False
        self.goal_stamp = None
        self.confirmations = 0
        self.last_frame = None
        self.observation_since = None
        self.event(self.reason)

    def observe(self, frame_id, ids, pose, stationary, frame_time, camera_fresh=True,
                alignment_verified=False):
        """A post-arrival frame needs explicit stable visual alignment proof.

        The adapter verifies horizontal camera centering separately. That
        visual yaw supersedes the nominal waypoint yaw, while fresh pose,
        positional tolerance, stop and expected marker ID remain mandatory.
        """
        if self.state != 'CONFIRMING' or self.observation_since is None:
            return False
        if frame_id == self.last_frame or frame_time < self.observation_since:
            return False
        self.last_frame = frame_id
        target = self.markers[self.expected_id]
        try:
            pose_ok = (pose is not None and all(isfinite(pose[name]) for name in ('x', 'y', 'yaw'))
                       and hypot(pose['x']-target['x'], pose['y']-target['y']) <= self.pose_tolerance)
        except (TypeError, ValueError, KeyError):
            pose_ok = False
        valid = (alignment_verified is True and camera_fresh and stationary
                 and pose_ok and self.expected_id in ids)
        self.confirmations = self.confirmations + 1 if valid else 0
        if self.confirmations >= self.required_confirmations:
            confirmed = self.expected_id
            self.completed_ids.append(confirmed)
            self.current_index += 1
            self.event(f'마커 {confirmed}을 실제 카메라 영상에서 확인했습니다.')
            self._send_current_goal()
            return True
        self.revision += 1
        return False

    def snapshot(self):
        return deepcopy({'state': self.state, 'route': self.route, 'pending_route': self.pending_route,
                         'current_index': self.current_index, 'current_marker_id': self.expected_id,
                         'completed_ids': self.completed_ids, 'reason': self.reason,
                         'can_resume': self.state == 'PAUSED' and not self.estop_latched,
                         'estop_latched': self.estop_latched,
                         'home': self.home, 'returning_home': self.returning_home,
                         'auto_return_home': self.auto_return_home,
                         'auto_returning_home': self.auto_returning_home,
                         'can_return_home': self.home is not None and not self.estop_latched
                         and not (self.returning_home and self.state in ACTIVE_STATES)})

    def drain_commands(self):
        commands, self.commands = self.commands, []
        return commands
