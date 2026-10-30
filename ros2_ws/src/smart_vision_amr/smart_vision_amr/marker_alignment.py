"""Bounded, rotation-only ArUco search and horizontal image-center alignment.

Pure logic: no ROS, OpenCV, coordinates, motor publisher or mission completion.
All times use the same monotonic clock. The adapter must reject camera captures
older than mission arrival and supply normalized, upright image coordinates.
``confidence`` is geometric corner/detection quality, not an ML probability.
"""

from math import atan2, isfinite, pi, sin, cos
from numbers import Real


def angle_difference(a, b):
    return atan2(sin(a-b), cos(a-b))


def finite_number(value):
    try:
        return isinstance(value, Real) and not isinstance(value, bool) and isfinite(value)
    except OverflowError:
        return False


class MarkerAlignment:
    """Lifecycle contract for the camera/mission adapter.

    start(marker_id, camera, start_yaw, now)
    observe(frame_id, detections, camera, frame_time, now)
      detections: [{id: int, center_x: -1..1, confidence: 0..1}]
      center_x is -1 at image left, +1 at image right, zero at center.
      frame_time and now use monotonic seconds; ID is a unique camera stamp.
    observe_wall(evidence, now)
      reliable, heading_error_rad, frame_id (scan stamp), observed_at, reason.
      The camera-normal angle is independent feedback from a single wall fit.
    tick(now, yaw, *, pose_ok, scan_clear, stationary, translation_stopped,
         manual_active, estop, camera_fresh) -> angular velocity in rad/s.
    snapshot() -> public status. ``linear`` is always exactly zero.

    A failed/confirmed attempt never restarts itself. The adapter must pause
    the mission on FAILED and retain the target for an explicit retry.
    """

    ACTIVE = frozenset({'SEARCHING', 'CENTERING', 'WALL_ALIGNING', 'HOLD', 'SETTLING'})

    def __init__(self, *, search_speed=.12, max_angular_velocity=.15,
                 center_gain=.35, center_tolerance=.08,
                 stable_seconds=.6, required_frames=3, min_confidence=.75,
                 sweep_angle=1.0, max_total_rotation=4.2, max_bounces=2,
                 timeout=45.0, frame_timeout=.5, camera_loss_timeout=1.0,
                 front_yaw_sign=-1.0, rear_yaw_sign=-1.0,
                 require_wall_alignment=False, strict_center_tolerance=.05,
                 wall_angle_tolerance=.05, wall_gain=.5,
                 wall_max_angular_velocity=.08, wall_timeout=.5,
                 wall_required_samples=3):
        positive = (search_speed, max_angular_velocity, center_gain, center_tolerance,
                    stable_seconds, sweep_angle, max_total_rotation,
                    timeout, frame_timeout, camera_loss_timeout, strict_center_tolerance,
                    wall_angle_tolerance, wall_gain, wall_max_angular_velocity, wall_timeout)
        if (not all(finite_number(v) and v > 0.0 for v in positive)
                or search_speed > max_angular_velocity
                or not 0.0 < center_tolerance < 1.0
                or not 0.0 <= min_confidence <= 1.0
                or required_frames < 2 or max_bounces < 1
                or type(wall_required_samples) is not int or wall_required_samples < 2
                or strict_center_tolerance >= 1.0
                or wall_angle_tolerance >= pi/2
                or sweep_angle >= pi
                or front_yaw_sign not in (-1., 1.) or rear_yaw_sign not in (-1., 1.)):
            raise ValueError('Invalid marker alignment limits')
        self.search_speed, self.max_angular_velocity = search_speed, max_angular_velocity
        self.require_wall_alignment = bool(require_wall_alignment)
        self.center_gain = center_gain
        self.center_tolerance = min(center_tolerance, strict_center_tolerance) if self.require_wall_alignment else center_tolerance
        self.wall_angle_tolerance, self.wall_gain = wall_angle_tolerance, wall_gain
        self.wall_max_angular_velocity = min(wall_max_angular_velocity, max_angular_velocity)
        self.wall_timeout, self.wall_required_samples = wall_timeout, wall_required_samples
        self._reset_wall()
        self.stable_seconds, self.required_frames = stable_seconds, int(required_frames)
        self.min_confidence = min_confidence
        self.sweep_angle, self.max_total_rotation = sweep_angle, max_total_rotation
        self.max_bounces, self.timeout = int(max_bounces), timeout
        self.frame_timeout, self.camera_loss_timeout = frame_timeout, camera_loss_timeout
        # With an upright non-mirrored image, pixel-right needs negative robot
        # yaw for both cameras. Rear mounting yaw=pi alone does not reverse it.
        self.yaw_sign = {'front': front_yaw_sign, 'rear': rear_yaw_sign}
        self.state, self.reason = 'IDLE', '마커 정렬 대기'
        self.marker_id = self.camera = None
        self.started_at = self.last_yaw = self.last_frame_id = None
        self.last_frame_at = self.last_seen_at = self.centered_since = None
        self.stationary_since = self.camera_gap_since = None
        self.clearance_gap_since = None
        self.heading_offset = self.rotation = self.angular = 0.0
        self.direction, self.bounces, self.frames = 1.0, 0, 0
        self.center_error = None
        self.last_tick_at = None

    def start(self, marker_id, camera, start_yaw, now):
        if (type(marker_id) is not int or marker_id < 0 or camera not in self.yaw_sign
                or not all(finite_number(value) for value in (start_yaw, now))):
            raise ValueError('A valid marker ID, selected camera, yaw and time are required')
        self.marker_id, self.camera, self.started_at = marker_id, camera, now
        self.last_yaw, self.last_tick_at = start_yaw, now
        self._reset_wall()
        self.last_frame_id = self.last_frame_at = self.last_seen_at = None
        self.centered_since = self.stationary_since = self.camera_gap_since = None
        self.clearance_gap_since = None
        self.heading_offset = self.rotation = self.angular = 0.0
        self.direction, self.bounces, self.frames = 1.0, 0, 0
        self.center_error = None
        self.state, self.reason = 'SEARCHING', f'선택한 {camera} 카메라로 마커 {marker_id}을 저속 탐색합니다.'

    def observe(self, frame_id, detections, camera, frame_time, now):
        if (self.state not in self.ACTIVE | {'CONFIRMED'} or camera != self.camera
                or not all(finite_number(value) for value in (frame_time, now))
                or frame_time < self.started_at or not 0.0 <= now-frame_time <= self.frame_timeout
                or (self.last_frame_at is not None and frame_time <= self.last_frame_at)
                or frame_id == self.last_frame_id
                or (type(frame_id) is int and type(self.last_frame_id) is int
                    and frame_id <= self.last_frame_id)):
            return False
        if self.last_frame_at is not None and frame_time-self.last_frame_at > self.frame_timeout:
            self._clear_centering()
        self.last_frame_id, self.last_frame_at = frame_id, frame_time
        candidates = []
        for item in detections:
            if not isinstance(item, dict) or item.get('id') != self.marker_id:
                continue
            error, quality = item.get('center_x'), item.get('confidence')
            if (type(error) not in (float, int) or type(quality) not in (float, int)
                    or not finite_number(error) or not finite_number(quality)
                    or not -1.0 <= error <= 1.0
                    or not self.min_confidence <= quality <= 1.0):
                continue
            candidates.append((float(quality), float(error)))
        if not candidates:
            self._clear_centering()
            self.center_error = self.last_seen_at = None
            self.state, self.reason = 'SEARCHING', '목표 마커 미검출 · 제한된 범위에서 저속 탐색합니다.'
            return True
        _, self.center_error = max(candidates, key=lambda pair: pair[0])
        self.last_seen_at = frame_time
        if abs(self.center_error) <= self.center_tolerance:
            if self.centered_since is None:
                self.centered_since = now
            self.frames += 1
            if self.state != 'CONFIRMED':
                self.state, self.reason = 'SETTLING', '마커가 가로 중앙에 있습니다. 정지 상태에서 안정성을 확인합니다.'
        else:
            self._clear_centering()
            self.state, self.reason = 'CENTERING', '목표 마커를 선택한 카메라의 가로 중앙으로 정렬합니다.'
        return True

    def _reset_wall(self):
        self.wall_reliable = False
        self.wall_heading_error = None
        self.wall_frame_id = self.wall_observed_at = self.wall_aligned_since = None
        self.wall_samples = 0
        self.wall_reason = '벽 방향 측정을 기다립니다.'

    def _clear_wall_proof(self):
        self.wall_samples = 0
        self.wall_aligned_since = None

    def observe_wall(self, evidence, now):
        """Receive an independently fitted, timestamped wall-normal estimate.

        The adapter checks acquisition age and camera/LiDAR transforms. Error
        is wall normal yaw minus the selected camera yaw, in base coordinates;
        it is not inferred from image skew or an assumed field of view.
        """
        if self.state not in self.ACTIVE | {'CONFIRMED'} or not finite_number(now):
            return False
        evidence = evidence if isinstance(evidence, dict) else {}
        stamp, observed_at = evidence.get('frame_id'), evidence.get('observed_at')
        error = evidence.get('heading_error_rad')
        valid = (evidence.get('reliable') is True and type(stamp) is int and stamp >= 0
                 and finite_number(observed_at) and self.started_at <= observed_at <= now
                 and now-observed_at <= self.wall_timeout and finite_number(error)
                 and abs(error) <= pi/2)
        if not valid:
            self.wall_reliable = False
            self.wall_reason = str(evidence.get('reason') or '벽 방향 또는 센서 시각을 확인할 수 없습니다.')[:240]
            self._clear_wall_proof()
            self.stationary_since = None
            if self.require_wall_alignment and self.state == 'CONFIRMED':
                self.state, self.reason = 'HOLD', self.wall_reason
            return False
        if (self.wall_frame_id is not None and stamp <= self.wall_frame_id
                or self.wall_observed_at is not None and observed_at <= self.wall_observed_at):
            return False
        if self.wall_observed_at is not None and observed_at-self.wall_observed_at > self.wall_timeout:
            self._clear_wall_proof()
            self.stationary_since = None
        self.wall_frame_id, self.wall_observed_at = stamp, float(observed_at)
        self.wall_heading_error, self.wall_reliable = float(error), True
        self.wall_reason = str(evidence.get('reason') or '단일 벽 방향 측정 확인')[:240]
        centered = (self.center_error is not None and abs(self.center_error) <= self.center_tolerance
                    and self.last_seen_at is not None and 0 <= now-self.last_seen_at <= self.frame_timeout)
        if centered and abs(error) <= self.wall_angle_tolerance:
            if self.wall_aligned_since is None:
                self.wall_aligned_since = now
            self.wall_samples += 1
        else:
            self._clear_wall_proof()
            self.stationary_since = None
            if self.require_wall_alignment and self.state == 'CONFIRMED':
                self.state, self.reason = 'WALL_ALIGNING', '벽 방향 정렬 승인을 해제했습니다.'
        return True

    def _wall_fresh(self, now):
        return (self.wall_reliable and self.wall_observed_at is not None
                and 0 <= now-self.wall_observed_at <= self.wall_timeout)

    def _clear_centering(self):
        self.frames = 0
        self.centered_since = self.stationary_since = None
        self._clear_wall_proof()

    def fail(self, reason):
        self.angular = 0.0
        self._clear_centering()
        self.state, self.reason = 'FAILED', str(reason)

    def tick(self, now, yaw, *, pose_ok=True, scan_clear=True, stationary=False,
             translation_stopped=True, manual_active=False, estop=False,
             camera_fresh=True):
        self.angular = 0.0
        if self.state not in self.ACTIVE | {'CONFIRMED'}:
            return 0.0
        if not all(finite_number(value) for value in (now, yaw)) or now < self.last_tick_at:
            self.fail('위치 또는 제어 시각이 유효하지 않아 마커 정렬을 일시정지합니다.')
            return 0.0
        if now-self.last_tick_at > min(self.frame_timeout, self.wall_timeout):
            self.stationary_since = None
        self.last_tick_at = now
        if estop or manual_active or not pose_ok or not translation_stopped:
            reason = ('비상정지 우선' if estop else '조이스틱 조작 우선' if manual_active
                      else '위치 신뢰도 부족' if not pose_ok
                      else '전진·후진 정지가 확인되지 않음')
            self.fail(reason+' · 목표를 유지하고 마커 정렬을 일시정지합니다.')
            return 0.0
        if not scan_clear:
            # A single dropped scan or laser transform must stop the rotation,
            # but should not permanently pause a mission whose next scan is safe.
            # No image or wall stability proof may accumulate while clearance
            # is unknown or physically insufficient.
            if self.clearance_gap_since is None:
                self.clearance_gap_since = now
            self._clear_centering()
            self.center_error = self.last_seen_at = None
            if now-self.clearance_gap_since >= 1.0:
                self.fail('회전 공간 부족 · 목표를 유지하고 마커 정렬을 일시정지합니다.')
            else:
                self.state, self.reason = 'HOLD', '라이다 회전 여유를 재확인하며 정지합니다.'
            return 0.0
        if self.clearance_gap_since is not None:
            # Require fresh post-gap camera and wall proof before confirmation.
            self.clearance_gap_since = None
            self._clear_centering()
            self.center_error = self.last_seen_at = None
        delta = angle_difference(yaw, self.last_yaw)
        self.last_yaw = yaw
        self.heading_offset += delta
        self.rotation += abs(delta)
        if now-self.started_at >= self.timeout or self.rotation >= self.max_total_rotation:
            self.fail('마커 탐색 시간 또는 회전 한도 초과 · 목표를 유지하고 명시적 재시도를 기다립니다.')
            return 0.0
        fresh = camera_fresh and self.last_frame_at is not None and now-self.last_frame_at <= self.frame_timeout
        if not fresh:
            self._clear_centering()
            if self.state == 'CONFIRMED':
                self.state, self.reason = 'SETTLING', '최신 영상 확인을 기다리며 중앙 정렬 승인을 해제했습니다.'
            if self.camera_gap_since is None:
                self.camera_gap_since = now
            if now-self.camera_gap_since >= self.camera_loss_timeout:
                self.fail('선택한 카메라 영상이 끊겨 목표를 유지하고 정렬을 일시정지합니다.')
            return 0.0
        self.camera_gap_since = None
        detected = self.last_seen_at is not None and now-self.last_seen_at <= self.frame_timeout
        if detected and self.center_error is not None:
            if abs(self.center_error) <= self.center_tolerance:
                if self.require_wall_alignment:
                    if not self._wall_fresh(now):
                        if self.wall_reliable:
                            self.wall_reliable = False
                            self.wall_reason = 'LiDAR 벽 방향 표본이 오래되었습니다. 최신 scan 수신을 확인하세요.'
                        self._clear_wall_proof()
                        self.stationary_since = None
                        self.state, self.reason = 'HOLD', ('마커 중앙 확인 · 벽 방향 확인 대기: '+self.wall_reason)
                        return 0.0
                    if abs(self.wall_heading_error) > self.wall_angle_tolerance:
                        self._clear_wall_proof()
                        self.stationary_since = None
                        self.state, self.reason = 'WALL_ALIGNING', '선택한 카메라가 벽의 표식을 정면으로 보도록 저속 회전합니다.'
                        speed = self.wall_gain*self.wall_heading_error
                        self.angular = max(-self.wall_max_angular_velocity, min(self.wall_max_angular_velocity, speed))
                        return self._bounded_rotation()
                self.state = 'SETTLING'
                if stationary:
                    if self.stationary_since is None:
                        self.stationary_since = now
                    if (self.frames >= self.required_frames and self.centered_since is not None
                            and now-self.centered_since >= self.stable_seconds
                            and now-self.stationary_since >= self.stable_seconds
                            and (not self.require_wall_alignment or (
                                self.wall_samples >= self.wall_required_samples
                                and self.wall_aligned_since is not None
                                and now-self.wall_aligned_since >= self.stable_seconds))):
                        self.state, self.reason = 'CONFIRMED', ('목표 마커 중앙·벽 직각 방향·정지 및 연속 센서 확인을 완료했습니다.'
                            if self.require_wall_alignment else '목표 마커의 가로 중앙 정렬과 정지·연속 영상 확인을 완료했습니다.')
                else:
                    self.stationary_since = None
                return 0.0
            self.state = 'CENTERING'
            speed = self.yaw_sign[self.camera]*self.center_gain*self.center_error
            self.angular = max(-self.max_angular_velocity, min(self.max_angular_velocity, speed))
        else:
            self.state = 'SEARCHING'
            if self.direction*self.heading_offset >= self.sweep_angle:
                if self.bounces >= self.max_bounces:
                    self.fail('제한된 마커 탐색 범위를 확인했지만 목표를 찾지 못했습니다. 임무를 유지합니다.')
                    return 0.0
                self.direction *= -1.0
                self.bounces += 1
            self.angular = self.direction*self.search_speed
        return self._bounded_rotation()

    def _bounded_rotation(self):
        # Centering also respects the same body-yaw envelope; a mistaken
        # detection must not turn a bounded search into an unbounded rotation.
        if ((self.heading_offset >= self.sweep_angle and self.angular > 0.0)
                or (self.heading_offset <= -self.sweep_angle and self.angular < 0.0)):
            self.fail('카메라 중앙 정렬에 필요한 선회가 제한 범위를 넘습니다. 위치를 확인하세요.')
            return 0.0
        return self.angular

    def snapshot(self):
        return {'state': self.state, 'reason': self.reason, 'marker_id': self.marker_id,
                'camera': self.camera, 'center_error': self.center_error,
                'center_tolerance': self.center_tolerance, 'frames': self.frames,
                'wall_required': self.require_wall_alignment, 'wall_reliable': self.wall_reliable,
                'wall_heading_error_rad': self.wall_heading_error,
                'wall_angle_tolerance': self.wall_angle_tolerance,
                'wall_samples': self.wall_samples, 'wall_required_samples': self.wall_required_samples,
                'wall_reason': self.wall_reason,
                'required_frames': self.required_frames, 'linear': 0.0,
                'angular': self.angular, 'rotation_degrees': self.rotation*180.0/pi,
                'max_seconds': self.timeout,
                'elapsed': 0.0 if self.started_at is None or self.last_tick_at is None
                else self.last_tick_at-self.started_at}
