"""Operator-facing state and next actions, derived from actual backend state."""

from math import isfinite


_FAILURE_PHASES = {
    'AVOIDING': ('failure_avoiding', '실패 접근 회피 · 경로 재계획'),
    'WAIT_ALTERNATIVE': ('failure_wait', '목표 유지 · 대체 경로 대기'),
    'RECOVERING': ('failure_reversing', '안전 후진 · 재진입 준비'),
    'REJOINING': ('failure_rejoining', '새 경로 진입 확인 중'),
    'SENSOR_HOLD': ('sensor_hold', '센서 회복 대기 · 정지 유지'),
    'OBSTACLE_HOLD': ('obstacle_hold', '장애물 해소 대기'),
}

OBSTACLE_HOLD_MESSAGE = '장애물이 있어 경로 주행을 잠시 멈춥니다. 장애물을 치우면 경로를 재개합니다.'


def local_detour_snapshot(payload, goal_stamp, mission_state, age):
    """Read-only, fresh planner evidence scoped to the current mission goal."""
    empty = {'available': False, 'state': 'NONE'}
    if (not isinstance(payload, dict) or type(age) not in (int, float)
            or not 0 <= age < 2. or not isfinite(age)
            or mission_state not in {'NAVIGATING', 'RETURNING_HOME', 'WAIT_REPLAN'}):
        return empty

    def stamp(value):
        if isinstance(value, dict):
            value = (value.get('sec'), value.get('nanosec'))
        if (not isinstance(value, (list, tuple)) or len(value) != 2
                or any(type(item) is not int for item in value)
                or value[0] < 0 or not 0 <= value[1] < 1_000_000_000):
            return None
        return tuple(value)

    current = stamp(goal_stamp)
    if current is None or stamp(payload.get('goal_stamp')) != current:
        return empty
    kind, state = payload.get('plan_kind'), payload.get('local_detour_state')
    if kind not in {'LOCAL_DETOUR', 'GLOBAL'} or state not in {'ACTIVE', 'REJOINED', 'FAILED', 'NONE'}:
        return empty
    result = {'available': True, 'state': state, 'plan_kind': kind,
              'reason': str(payload.get('detour_reason') or '')[:240]}
    for name in ('local_attempt_id', 'global_plan_count', 'expansions'):
        value = payload.get(name)
        if type(value) is int and 0 <= value <= 1_000_000_000:
            result[name] = value
    for name in ('search_duration_ms', 'adapter_duration_ms', 'global_duration_ms', 'rejoin_progress'):
        value = payload.get(name)
        if type(value) in (int, float) and 0 <= value <= 1_000_000 and isfinite(value):
            result[name] = float(value)
    point = payload.get('rejoin_point')
    if isinstance(point, dict):
        values = [point.get('x'), point.get('y')]
        if all(type(value) in (int, float) and abs(value) <= 1_000_000
               and isfinite(value) for value in values):
            result['rejoin_point'] = dict(zip(('x', 'y'), values))
    return result


def failure_memory_snapshot(payload, goal_stamp, mission_state, age):
    """Expose only this goal's fresh status; display code cannot clear memory.

    Durable ROS messages may predate a new mission or a bridge reconnection.
    Match the exact goal, bound the data and age the TTL without changing the
    planner's memory. A missing/stale message must never look like recovery.
    """
    empty = {'state': 'CLEAR', 'count': 0, 'active': [], 'available': False}
    if (not isinstance(payload, dict) or not isfinite(age) or not 0. <= age < 2.
            or mission_state not in {'NAVIGATING', 'RETURNING_HOME', 'WAIT_REPLAN', 'PAUSED', 'ESTOP'}):
        return empty

    def stamp(value):
        if isinstance(value, dict):
            value = (value.get('sec'), value.get('nanosec'))
        if (not isinstance(value, (tuple, list)) or len(value) != 2
                or any(type(item) is not int for item in value)
                or value[0] < 0 or not 0 <= value[1] < 1000000000):
            return None
        return tuple(value)

    current = stamp(goal_stamp)
    if current is None or stamp(payload.get('goal_stamp')) != current:
        return empty
    state = payload.get('state')
    if state not in _FAILURE_PHASES and state != 'CLEAR':
        return empty
    active = []
    for item in (payload.get('active') if isinstance(payload.get('active'), list) else [])[:100]:
        if not isinstance(item, dict):
            continue
        values = [item.get('x'), item.get('y'), item.get('yaw', item.get('approach_heading')),
                  item.get('expires_in')]
        if any(type(value) not in (int, float) or not isfinite(value) for value in values):
            continue
        remaining = max(0., float(values[3])-age)
        if remaining <= 0.:
            continue
        failures = item.get('failures', 1)
        active.append({'id': str(item.get('id', ''))[:128],
                       'x': float(values[0]), 'y': float(values[1]), 'yaw': float(values[2]),
                       'reason': str(item.get('reason', ''))[:240], 'expires_in': round(remaining, 1),
                       'failures': min(100000, max(1, failures)) if type(failures) is int else 1})
    attempts = payload.get('recovery_attempts', 0)
    result = {'state': state, 'reason': str(payload.get('reason', ''))[:480],
            'count': len(active), 'active': active, 'available': True,
            'last_failure_id': str(payload.get('last_failure_id') or '')[:128],
            'rejoin_verified': payload.get('rejoin_verified') is True,
            'recovery_attempts': min(100000, max(0, attempts)) if type(attempts) is int else 0,
            'goal_stamp': {'sec': current[0], 'nanosec': current[1]}}
    for key in ('retry_count', 'retry_limit', 'blocked_obstacle_count'):
        value = payload.get(key)
        if type(value) is int and value >= 0:
            result[key] = min(100000, value)
    wait = payload.get('clear_wait_seconds')
    if type(wait) in (int, float) and isfinite(wait) and wait >= 0:
        result['clear_wait_seconds'] = min(3600., float(wait))
    return result

_LOCALIZATION_REASONS = {
    'initializing': '초기 위치 검증 중', 'operator_reset_required': '원인 확인 후 비상정지 해제 필요',
    'scan_unavailable': '라이다 수신·수치 확인 필요', 'odom_unavailable': '오도메트리 수신·수치 확인 필요',
    'odom_tf_unavailable': '차체 오도메트리 TF 대기', 'map_tf_unavailable': '지도상 차체 TF 대기',
    'map_unavailable': '저장 지도 수신 대기', 'amcl_covariance_unavailable': 'AMCL 위치 불확실성 수신 대기',
    'position_std_high': '위치 추정 오차 범위가 큼', 'yaw_std_high': '방향 추정 오차 범위가 큼',
    'position_std_m_elevated': '위치 추정 오차 범위가 큼',
    'yaw_std_rad_elevated': '방향 추정 오차 범위가 큼',
    'position_std_m_high': '위치 추정 오차 범위가 큼',
    'yaw_std_rad_high': '방향 추정 오차 범위가 큼',
    'position_high': '위치 추정 오차 범위가 큼', 'yaw_high': '방향 추정 오차 범위가 큼',
    'amcl_not_updating_while_moving': '이동 중 AMCL 위치 갱신 지연',
    'localization_jump': '오도메트리와 다른 큰 위치 보정 감지',
    'scan_map_check_unavailable': '측정 시각 TF와 라이다·지도 대조 대기',
    'scan_map_disagreement': '라이다와 저장 지도 불일치',
    'scan_map_agreement_low': '라이다와 저장 지도 일치도가 낮음',
}


def localization_explanations(health):
    return [_LOCALIZATION_REASONS.get(reason, str(reason)) for reason in health.get('reasons', [])]

_ACTIONS = (
    ('ROS 연결', '차량 연결 대기', '로봇과 PC의 전원·네트워크·ROS Domain ID를 확인하세요.', 'connection'),
    ('설정 지도와 다른', '지도 불일치', 'SLAM을 종료하고 등록된 지도와 같은 Nav2를 실행하세요.', 'diagnostics'),
    ('/map', '운용 지도 대기', '등록된 저장 지도로 내비게이션을 실행하세요.', 'diagnostics'),
    ('/odom', '차량 움직임 정보 대기', '로봇 bringup과 EKF 입력·출력 상태를 확인하세요.', 'diagnostics'),
    ('/scan', '라이다 수신 대기', '라이다 연결과 로봇 bringup을 확인하세요.', 'diagnostics'),
    ('위치 추정 상태 수신 대기', '위치 감시 노드 대기', '로봇 런타임과 PC 주행 스크립트를 실행하고 위치 신뢰도 수신을 확인하세요.', 'diagnostics'),
    ('위치 신뢰도', '위치 확인 필요', '차량을 멈추고 자세 초기화 또는 위치 확인을 실행하세요.', 'localize'),
    ('위치 추정', '위치 확인 필요', '실제 위치를 확인한 뒤 자세 초기화 또는 위치 확인을 실행하세요.', 'localize'),
    ('map→base', '지도상 위치 대기', '위치 추정 노드를 켜고 실제 위치에서 자세를 초기화하세요.', 'localize'),
    ('A*', '주행 노드 대기', '내비게이션 실행 터미널의 기동 상태를 확인하세요.', 'diagnostics'),
    ('취소 서비스', '주행 제어 연결 대기', '주행 노드를 다시 연결하고 정지 제어 상태를 확인하세요.', 'diagnostics'),
    ('카메라', '영상 수신 대기', '해당 카메라 노드와 압축 영상 토픽을 확인하세요.', 'camera'),
)


def explain_blockers(missing):
    """Choose one root action while retaining every original condition."""
    missing = [str(item) for item in missing]
    for fragment, title, action, target in _ACTIONS:
        matches = [item for item in missing if fragment in item]
        if matches:
            return {'title': title, 'detail': matches[0], 'action': action,
                    'target': target, 'conditions': missing}
    return {'title': '출발 조건 확인', 'detail': missing[0] if missing else '',
            'action': '표시된 준비 조건을 확인하세요.' if missing else '목적지와 적재 상태를 확인하세요.',
            'target': 'diagnostics', 'conditions': missing}


def operation_summary(snapshot):
    mission = snapshot.get('mission') or {}
    manual = snapshot.get('manual') or {}
    localization = snapshot.get('localization') or {}
    safety = snapshot.get('safety') or {}
    health = snapshot.get('health') or {}
    quality = snapshot.get('localization_health') or {}
    delivery = snapshot.get('delivery') or {}
    connection = snapshot.get('connection') or {}
    nav = snapshot.get('navigation') or {}
    alignment = snapshot.get('marker_alignment') or {}
    failure_memory = snapshot.get('failure_memory') or {}
    detour = snapshot.get('local_detour') or {}
    if (alignment.get('marker_id') is not None
            and mission.get('current_marker_id') is not None
            and alignment['marker_id'] != mission['current_marker_id']):
        alignment = {}  # Never carry a previous sector's result into this goal.
    level, phase, label = 'info', 'idle', '임무 대기'
    detail = ''
    action = explain_blockers(health.get('missing', []))
    if mission.get('estop_latched') or safety.get('latched'):
        level, phase, label = 'critical', 'estopped', '비상정지'
        detail = ('정지 명령 0 발행 응답 확인 · 실제 정지는 차량에서 확인하세요.' if safety.get('stop_acknowledged')
                  else '정지를 요청했습니다. 차량 정지 응답을 확인 중입니다.')
        if safety.get('pending') == 'reset':
            label = '비상정지 해제 검증 중'
            detail = '센서와 공통 제어기의 해제 응답을 기다립니다. 자동 출발하지 않습니다.'
        action = {'title': '정지 유지', 'detail': detail,
                  'action': ('실제 등록 홈에 있다면 비상정지 해제 창에서 홈 확인 후 초기 위치만 입력하세요. '
                             '홈이 아니라면 조종기에서 실제 위치를 지정하세요. 센서 안정 후 해제하고 위치 확인을 진행하세요.'
                             if quality.get('stop_latched') else
                             '원인과 주변을 확인한 뒤 비상정지를 해제하고 임무를 별도로 재개하세요.'),
                  'target': 'safety', 'conditions': health.get('missing', [])}
    elif not connection.get('connected'):
        phase, label = 'connecting', '차량 연결 대기'
    elif localization.get('active'):
        phase, label = 'localizing', '위치 확인 중'
        detail = localization.get('reason') or '저속 회전으로 라이다와 저장 지도를 대조합니다.'
        if localization.get('control', {}).get('state') == 'WAIT_HEALTH':
            level, label = 'warning', '위치 감시 복구 대기 · 정지'
            action = {'title': '위치 감시 입력 확인', 'detail': detail,
                      'action': '라이다·오도메트리와 좌표 변환 복구를 기다리며 정지합니다. '
                                '1초 이상 복구되지 않으면 위치 확인을 중단합니다.',
                      'target': 'localize', 'conditions': health.get('missing', [])}
    elif quality.get('level') == 'critical':
        level, phase, label = 'critical', 'localization_fault', '위치 신뢰도 확인 필요'
        detail = ' · '.join(localization_explanations(quality))
        action = explain_blockers(['위치 신뢰도: '+detail])
    elif manual.get('active') or manual.get('state') in {'ACQUIRING', 'ACTIVE', 'RELEASING', 'WAIT_REPLAN'}:
        phase, label = 'manual', '수동 조종 우선'
        detail = manual.get('reason') or '조이스틱을 놓으면 같은 목표로 재개를 준비합니다.'
    elif (mission.get('state') in {'NAVIGATING', 'RETURNING_HOME', 'WAIT_REPLAN'}
          and detour.get('available') is True and detour.get('plan_kind') == 'LOCAL_DETOUR'
          and detour.get('state') == 'ACTIVE'
          and not (failure_memory.get('available') is True
                   and failure_memory.get('state') in {'SENSOR_HOLD', 'OBSTACLE_HOLD', 'RECOVERING', 'WAIT_ALTERNATIVE'})):
        phase, label = 'local_detour', '장애물 국소 우회 · 원래 경로 합류 중'
        detail = '라이다와 차체 여유를 확인하며 가까운 장애물을 돌아 원래 경로의 앞쪽에 합류합니다.'
        action = {'title': '국소 우회 중', 'detail': detail,
                  'action': '우회 공간과 라이다 입력을 확인하세요. 조이스틱과 비상정지가 항상 우선합니다.',
                  'target': 'diagnostics', 'conditions': health.get('missing', [])}
    elif (mission.get('state') in {'NAVIGATING', 'RETURNING_HOME', 'WAIT_REPLAN'}
          and failure_memory.get('available') is True
          and failure_memory.get('state') in _FAILURE_PHASES):
        phase, label = _FAILURE_PHASES[failure_memory['state']]
        level = 'warning'
        detail = (OBSTACLE_HOLD_MESSAGE if failure_memory['state'] == 'OBSTACLE_HOLD' else
                  failure_memory.get('reason') or '현재 목표를 유지하며 같은 실패 접근의 반복을 피합니다.')
        action = {'title': '복구 상태 확인', 'detail': detail,
                  'action': ('장애물을 치운 뒤 경로가 열리는지 확인하세요. 조이스틱·일시정지·비상정지가 우선합니다.'
                             if failure_memory['state'] == 'OBSTACLE_HOLD' else
                             '센서가 회복될 때까지 정지합니다. 수신 상태와 위치 신뢰도를 확인하세요.'
                             if failure_memory['state'] == 'SENSOR_HOLD' else
                             '후진 공간과 새 경로를 확인하세요. 대체 경로가 없으면 목표를 유지하고 기다립니다. 조이스틱과 비상정지가 우선합니다.'),
                  'target': 'diagnostics', 'conditions': health.get('missing', [])}
    elif (mission.get('state') == 'WAIT_REPLAN' or
          (mission.get('state') in {'NAVIGATING', 'RETURNING_HOME'}
           and nav.get('planner_status') == 'WAIT_REPLAN')):
        level, phase, label = 'warning', 'replanning', '목표 유지 · 경로 재계획 중'
        detail = nav.get('message') or mission.get('reason', '')
    elif mission.get('state') == 'PAUSED':
        level, phase, label = 'warning', 'paused', '임무 일시정지'
        detail = mission.get('reason', '')
        if alignment.get('state') == 'FAILED':
            label = '표식 확인 일시정지'
            detail = alignment.get('reason') or '도착 표식의 중앙 정렬을 확인하지 못했습니다.'
            action = {'title': '카메라와 표식 확인', 'detail': detail,
                      'action': '카메라에 목표 표식이 보이는지 확인한 뒤 임무를 재개하세요. 자동 출발하지 않습니다.',
                      'target': 'camera', 'conditions': health.get('missing', [])}
    elif mission.get('returning_home'):
        phase, label = 'returning_home', '홈 복귀 중'
    elif mission.get('state') == 'CONFIRMING':
        phase, label = 'confirming', '도착 표식 확인 중'
        detail = '목적지에서 정지하고 해당 섹터의 카메라 표식을 확인합니다.'
        alignment_labels = {'SEARCHING': '도착 표식 탐색 중',
                            'CENTERING': '카메라 중앙 정렬 중',
                            'WALL_ALIGNING': '벽 직각 방향 정렬 중',
                            'HOLD': '벽 방향 확인 대기',
                            'SETTLING': '정지 · 표식 안정 확인 중',
                            'CONFIRMED': '표식 중앙 정렬 확인 완료',
                            'FAILED': '표식 확인 필요'}
        if alignment.get('state') in alignment_labels:
            label = alignment_labels[alignment['state']]
            detail = alignment.get('reason') or detail
            if alignment['state'] in {'FAILED', 'HOLD'}:
                level = 'warning'
            action = {'title': '카메라 표식 정렬', 'detail': detail,
                      'action': '선택된 카메라와 중심 오차를 확인하세요. 조이스틱과 비상정지가 항상 우선합니다.',
                      'target': 'camera', 'conditions': health.get('missing', [])}
            if alignment['state'] in {'WALL_ALIGNING', 'HOLD'}:
                action.update(title='표식과 벽 방향 확인', action=
                    '후방 카메라가 표식 벽을 정면으로 보고 차체의 앞뒤 축이 벽과 직각이 되어야 합니다. '
                    '벽 주변 장애물과 라이다 입력을 확인하세요. 조이스틱과 비상정지가 항상 우선합니다.')
    elif delivery.get('active') or mission.get('state') in {'NAVIGATING', 'ALIGNING', 'VERIFYING', 'WAIT_MARKER', 'RUNNING'}:
        phase, label = 'delivering', '배송 중'
    elif delivery.get('home_ready'):
        phase, label = 'home', '홈 대기'
    if health.get('missing') and phase in {'idle', 'home'}:
        phase, label = 'waiting', action['title']
    if quality.get('level') == 'warning' and level != 'critical':
        level = 'warning'
    return {'phase': phase, 'label': label, 'level': level, 'detail': detail,
            'next_action': action, 'mode': 'manual' if phase == 'manual' else 'auto',
            'control_owner': safety.get('output_owner', safety.get('source_id')),
            'control_source': safety.get('source_id'),
            'stop_acknowledged': bool(safety.get('stop_acknowledged'))}


def route_preview(snapshot):
    """Actual visit order with current/completed flags and an explicit final home."""
    mission = snapshot.get('mission') or {}
    markers = {item['id']: item for item in snapshot.get('markers', [])}
    route = mission.get('route') or []
    index = int(mission.get('current_index') or 0)
    returning = bool(mission.get('returning_home'))
    active = mission.get('state') in {'NAVIGATING', 'CONFIRMING', 'WAIT_REPLAN', 'PAUSED', 'WAIT_MARKER', 'ESTOP'}
    output = []
    for position, marker_id in enumerate(route):
        marker = markers.get(marker_id, {})
        output.append({'id': marker_id, 'name': marker.get('name', f'Sector {marker_id}'),
                       'order': position+1, 'current': active and position == index and not returning,
                       'completed': position < index, 'home': False})
    home = (snapshot.get('delivery') or {}).get('homepoint')
    if home:
        output.append({'id': 0, 'name': '홈 복귀', 'order': len(output)+1,
                       'current': returning, 'completed': mission.get('state') == 'HOME_REACHED',
                       'home': True})
    return output
