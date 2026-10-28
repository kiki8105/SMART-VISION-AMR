"""Connect durable shortage requests to verified marker missions, never timers."""
from copy import deepcopy
import math
import time

from smart_vision_amr.inventory_logic import InventoryQueue
from smart_vision_amr.mission_logic import ACTIVE_STATES, START_STATES, MissionError


class DeliveryService:
    def __init__(self, app):
        self.app = app
        saved = app.saved.get('domains', {}).get(str(app.domain_id), {})
        initial = next((m for m in app.markers if m['id'] == 0), app.markers[0])
        self.registered_homepoint = {key: float(initial[key]) for key in ('x', 'y', 'yaw')}
        home = saved.get('homepoint', initial)
        try:
            self.homepoint = {key: float(home[key]) for key in ('x', 'y', 'yaw')}
            if not all(math.isfinite(value) for value in self.homepoint.values()):
                raise ValueError()
        except (ValueError, KeyError, TypeError):
            raise MissionError('저장된 Homepoint가 올바르지 않습니다.') from None
        path = app.config_file.with_name(f'{app.config_file.stem}.inventory-domain-{app.domain_id}.json')
        self.queue = InventoryQueue(app.markers, path)
        self.completed_count = 0
        self.error = ''
        app.mission.home = deepcopy(self.homepoint)

    @property
    def active(self):
        return self.queue.snapshot()['active_trip'] is not None

    def at_home(self):
        app, home = self.app, self.homepoint
        now = time.monotonic()
        if (app.pose is None or now-app.pose_at >= 2.0 or now-app.odom_at >= 2.0
                or app.mission.estop_latched or abs(app.speed['linear']) >= .025
                or abs(app.speed['angular']) >= .06):
            return False
        delta = app.pose['yaw']-home['yaw']
        return (math.hypot(app.pose['x']-home['x'], app.pose['y']-home['y']) <= .30
                and abs(math.atan2(math.sin(delta), math.cos(delta))) <= .35)

    def _verified_home_for_history(self):
        app = self.app
        return (app.connected and app.map is not None and not app.map_is_preview
                and app.map_matches_configured is not False
                and time.monotonic()-app.scan_at < 2.0
                and not app.localization.active and not app.localization_invalidated
                and app.pending_connection is None and not app.mission.estop_latched
                and not app.manual.in_progress and self.at_home())

    def snapshot(self):
        data = self.queue.snapshot()
        trip = data['active_trip']
        home_error = self.home_validation_error(self.homepoint)
        dispatch_missing = self.dispatch_missing()
        return dict(data, homepoint=deepcopy(self.homepoint), home_ready=self.at_home(),
                    registered_homepoint=deepcopy(self.registered_homepoint),
                    homepoint_custom=self.homepoint != self.registered_homepoint,
                    homepoint_valid=not home_error, homepoint_error=home_error,
                    dispatch_ready=not dispatch_missing, dispatch_missing=dispatch_missing,
                    active=trip is not None, accepting_updates=bool(trip and trip['accepting_updates']),
                    pending_count=sum(r['status'] == 'PENDING' for r in data['requests']),
                    operator_id=0, error=self.error)

    def home_validation_error(self, home):
        """Check the actual occupancy grid, never the decorative floorplan."""
        app = self.app
        metadata = app._display_map_metadata or app.map
        data = app._display_map_data
        # Lightweight applications may not have map geometry yet. Real map
        # reception supplies origin and raster together; partial real geometry
        # must fail closed rather than accepting an unchecked Homepoint.
        if data is None and (metadata is None or 'origin' not in metadata):
            return ''
        try:
            origin = metadata['origin']
            resolution = float(metadata['resolution'])
            width, height = int(metadata['width']), int(metadata['height'])
            ox, oy, yaw = (float(origin[key]) for key in ('x', 'y', 'yaw'))
            x, y, heading = (float(home[key]) for key in ('x', 'y', 'yaw'))
            if (resolution <= 0 or width <= 0 or height <= 0 or data is None
                    or not all(math.isfinite(value) for value in
                               (resolution, ox, oy, yaw, x, y, heading))
                    or len(data) != height or any(len(row) != width for row in data)):
                raise ValueError()
            dx, dy = x-ox, y-oy
            c, s = math.cos(yaw), math.sin(yaw)
            column = math.floor((c*dx+s*dy)/resolution)
            row = math.floor((-s*dx+c*dy)/resolution)
            if not (0 <= column < width and 0 <= row < height):
                return 'Homepoint가 운용 지도 범위 밖입니다. 등록 홈으로 복원하거나 위치를 다시 확인하세요.'
            occupancy = int(data[row][column])
            if occupancy < 0:
                return 'Homepoint가 지도에서 확인되지 않은 영역입니다.'
            if occupancy >= 65:
                return 'Homepoint가 지도상의 장애물과 겹칩니다.'
        except (KeyError, TypeError, ValueError, OverflowError):
            return 'Homepoint를 확인할 지도 데이터가 올바르지 않습니다.'
        return ''

    def dispatch_missing(self):
        """Explain departure blockers without changing requests or the mission."""
        app = self.app
        missing = list(app.readiness(for_home=False, marker_ids=self.pending_marker_ids())['missing'])
        if self.active:
            missing.append('기존 배송 운행 완료')
        if app.pending_connection is not None:
            missing.append('ROS 연결 변경 완료')
        if (app.navigation_active or app.cancel_pending
                or any(kind == 'cancel' for kind, _ in app.mission.commands)):
            missing.append('기존 주행 종료 확인')
        if app.mission.state not in START_STATES:
            missing.append('기존 임무 종료 또는 홈 복귀 완료')
        if app.mission.estop_latched:
            missing.append('비상정지 해제')
        home_error = self.home_validation_error(self.homepoint)
        if home_error:
            missing.append(home_error)
        now = time.monotonic()
        if app.pose is not None and now-app.pose_at < 2.0:
            distance = math.hypot(app.pose['x']-self.homepoint['x'], app.pose['y']-self.homepoint['y'])
            delta = app.pose['yaw']-self.homepoint['yaw']
            heading_error = abs(math.atan2(math.sin(delta), math.cos(delta)))
            if distance > .30:
                missing.append(f'Homepoint 위치 확인 또는 복귀 (홈까지 {distance:.2f} m, 허용 0.30 m)')
            if heading_error > .35:
                missing.append(f'Homepoint 방향 정렬 (오차 {math.degrees(heading_error):.0f}°, 허용 20°)')
        if abs(app.speed['linear']) >= .025 or abs(app.speed['angular']) >= .06:
            missing.append('로봇 정지 확인')
        if not any(item['status'] == 'PENDING' for item in self.queue.snapshot()['requests']):
            missing.append('접수된 대기 요청')
        return list(dict.fromkeys(missing))

    def pending_marker_ids(self, request_ids=None):
        """Preview FIFO dispatch goals without changing any request state."""
        pending = [item for item in self.queue.snapshot()['requests'] if item['status'] == 'PENDING']
        if isinstance(request_ids, list):
            selected = {item for item in request_ids if type(item) is int}
            pending = [item for item in pending if item['id'] in selected]
        return [item['marker_id'] for item in sorted(pending, key=lambda item: item['id'])[:100]]

    def _check_home_change(self):
        app = self.app
        if (self.active or app.mission.state not in START_STATES
                or app.mission.estop_latched or app.navigation_active or app.cancel_pending
                or app.pending_connection is not None
                or any(kind == 'cancel' for kind, _ in app.mission.commands)):
            raise MissionError('모든 운행을 마치고 정지한 뒤 Homepoint를 설정하세요.')

    def _save_home(self, home, message):
        app = self.app
        error = self.home_validation_error(home)
        if error:
            raise MissionError(error)
        previous = self.homepoint
        self.homepoint = deepcopy(home)
        try:
            app.save()
        except Exception:
            self.homepoint = previous
            raise
        app.mission.home = deepcopy(self.homepoint)
        app.mission.event(message)

    def set_home(self):
        app = self.app
        self._check_home_change()
        health = app.readiness(for_home=True)
        if not health['ready'] or abs(app.speed['linear']) >= .025 or abs(app.speed['angular']) >= .06:
            raise MissionError('최신 위치·센서와 정지 상태가 필요합니다: '+', '.join(health['missing']))
        self._save_home(app.pose, '현재 위치와 방향을 고정 Homepoint로 저장했습니다.')

    def reset_home(self):
        """Restore the registered Home target; never change localization or drive."""
        self._check_home_change()
        if abs(self.app.speed['linear']) >= .025 or abs(self.app.speed['angular']) >= .06:
            raise MissionError('로봇이 정지한 뒤 등록 Homepoint를 복원하세요.')
        self._save_home(self.registered_homepoint, '등록된 마커 0의 Homepoint를 복원했습니다. 차량 위치는 변경하지 않습니다.')

    def start(self, payload):
        app, mission = self.app, self.app.mission
        if payload.get('loaded') is not True:
            raise MissionError('적재 완료를 확인한 뒤 출발하세요.')
        if (self.active or app.pending_connection is not None or app.navigation_active or app.cancel_pending
                or mission.state not in START_STATES or mission.estop_latched
                or any(kind == 'cancel' for kind, _ in mission.commands)):
            raise MissionError('기존 운행과 정지를 확인한 뒤 새 배송을 출발시키세요.')
        if not self.at_home():
            raise MissionError('Homepoint에 복귀해 정지한 뒤 적재하고 출발하세요.')
        home_error = self.home_validation_error(self.homepoint)
        if home_error:
            raise MissionError(home_error)
        health = app.readiness(for_home=False, marker_ids=self.pending_marker_ids(payload.get('request_ids')))
        if not health['ready']:
            raise MissionError('배송 시작 조건을 확인하세요: '+', '.join(health['missing']))
        trip = self.queue.start_trip(payload.get('request_ids'))
        records = {item['id']: item for item in self.queue.snapshot()['requests']}
        route = [records[ident]['marker_id'] for ident in trip['request_ids']]
        mission.update_route(route)
        mission.start(home=self.homepoint, auto_return_home=True)
        self.completed_count = 0
        self.error = ''
        mission.event(f'운용자 0이 적재를 확인하고 배송 {len(route)}건을 출발시켰습니다.')

    def submit(self, user_id, payload):
        request = self.queue.submit(user_id, payload)
        self.app.mission.event(f'관리자 {user_id}: {request["item"]} {request["quantity"]}개 부족 요청 #{request["id"]}')
        self.sync()
        return next(item for item in self.queue.snapshot()['requests'] if item['id'] == request['id'])

    def sync(self, append=True):
        """Called under the application lock after navigation/camera decisions."""
        mission = self.app.mission
        data = self.queue.snapshot()
        trip = data['active_trip']
        if trip is None:
            if data['deferred_history_ids'] and self._verified_home_for_history():
                self.queue.clear_deferred_history_at_home()
            return
        while self.completed_count < len(mission.completed_ids):
            ident = trip['request_ids'][self.completed_count]
            record = next(item for item in data['requests'] if item['id'] == ident)
            if record['status'] == 'IN_PROGRESS':
                self.queue.complete_request(ident)
            self.completed_count += 1
            data = self.queue.snapshot()
            trip = data['active_trip']
        if mission.state == 'HOME_REACHED':
            self.queue.close_trip()
            if data['deferred_history_ids'] and self._verified_home_for_history():
                self.queue.clear_deferred_history_at_home()
            mission.event('모든 배송 운행을 마치고 Homepoint에서 다음 적재를 기다립니다.')
            return
        if (not append or not trip['accepting_updates'] or not mission.auto_return_home
                or mission.estop_latched or mission.state not in ACTIVE_STATES | {'PAUSED'}):
            return
        # One run is bounded; overflow remains durable PENDING for the next load.
        for record in data['requests']:
            if record['status'] != 'PENDING' or len(mission.route) >= 100:
                continue
            mission.validate_route([record['marker_id']])
            self.queue.append_to_trip(record['id'])
            mission.append_waypoints([record['marker_id']])

    def cancel(self):
        self.sync(append=False)
        if self.active:
            self.queue.cancel_trip()

    def emergency(self):
        if self.active:
            # Remaining work stays visible but cannot trigger another departure.
            self.queue.cancel_trip()

    def fail(self, exception):
        app, message = self.app, '재고 요청 저장/연결 오류로 정지합니다: '+str(exception)
        if app.mission.state in ACTIVE_STATES:
            app.mission.pause()
        else:
            app.mission.commands = [('cancel', None), ('stop', None)]
        if message != self.error:
            app.mission.event(message, 'warning')
        self.error = message
        app.mission.reason = message
