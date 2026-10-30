from smart_vision_amr.operation_status import (explain_blockers, operation_summary,
                                               route_preview, localization_explanations)
from smart_vision_amr.localization_health_math import LocalizationHealthCore


def base():
    return {'connection': {'connected': True}, 'health': {'missing': []},
            'mission': {'state': 'IDLE'}, 'delivery': {}}


def test_actual_covariance_health_reasons_have_operator_explanations():
    for position, yaw, expected in ((.446, .1, '위치 추정 오차 범위가 큼'),
                                    (.9, .1, '위치 추정 오차 범위가 큼'),
                                    (.1, .6, '방향 추정 오차 범위가 큼'),
                                    (.1, 1.1, '방향 추정 오차 범위가 큼')):
        status = LocalizationHealthCore().evaluate(1., {'position_std_m': position, 'yaw_std_rad': yaw})
        explanations = localization_explanations(status)
        assert expected in explanations
        assert not any('std_' in text for text in explanations)


def test_primary_cause_preserves_all_conditions():
    result = explain_blockers(['최신 카메라 영상', '최신 map→base TF', '최신 /odom'])
    assert result['target'] == 'diagnostics'
    assert result['title'] == '차량 움직임 정보 대기'
    assert len(result['conditions']) == 3


def test_requested_stop_is_not_reported_as_confirmed():
    state = base()
    state.update(safety={'latched': True, 'stop_acknowledged': False})
    result = operation_summary(state)
    assert result['phase'] == 'estopped'
    assert result['stop_acknowledged'] is False
    assert '확인 중' in result['detail']
    state['safety']['stop_acknowledged'] = True
    assert '명령 0 발행 응답 확인' in operation_summary(state)['detail']


def test_manual_and_home_return_are_distinct():
    state = base()
    state['mission']['returning_home'] = True
    assert operation_summary(state)['phase'] == 'returning_home'
    state['manual'] = {'active': True}
    assert operation_summary(state)['phase'] == 'manual'


def test_route_order_and_final_home():
    state = base()
    state['mission'].update(route=[2, 1, 3], current_index=1, state='NAVIGATING')
    state['markers'] = [{'id': 1, 'name': 'Sector 1'}]
    state['delivery']['homepoint'] = {'x': 0, 'y': 0}
    result = route_preview(state)
    assert [item['id'] for item in result] == [2, 1, 3, 0]
    assert result[0]['completed'] and result[1]['current'] and result[-1]['home']


def test_pending_reset_and_marker_verification_have_distinct_operator_states():
    state = base()
    state.update(safety={'latched': True, 'stop_acknowledged': True, 'pending': 'reset'})
    assert '해제 검증' in operation_summary(state)['label']
    state['safety'] = {}
    state['mission']['state'] = 'CONFIRMING'
    assert operation_summary(state)['phase'] == 'confirming'


def test_localization_policy_gap_reports_stopped_waiting_with_actual_reason():
    state = base()
    reason = '위치 감시 입력·오도메트리 좌표 변환 복구를 기다리며 정지합니다.'
    state['localization'] = {'active': True, 'state': 'ROTATING', 'reason': reason,
                             'control': {'state': 'WAIT_HEALTH'}}
    result = operation_summary(state)
    assert result['phase'] == 'localizing'
    assert result['label'] == '위치 감시 복구 대기 · 정지'
    assert result['detail'] == reason
    assert '1초 이상' in result['next_action']['action']


def test_stale_planner_wait_never_overrides_a_finished_or_paused_mission():
    state = base()
    state['navigation'] = {'planner_status': 'WAIT_REPLAN'}
    assert operation_summary(state)['phase'] == 'idle'
    state['mission']['state'] = 'PAUSED'
    assert operation_summary(state)['phase'] == 'paused'


def test_marker_alignment_phases_and_failed_resume_guidance():
    state = base()
    state['mission']['state'] = 'CONFIRMING'
    for phase, label in [('SEARCHING', '탐색'), ('CENTERING', '중앙 정렬'),
                         ('WALL_ALIGNING', '벽 직각 방향 정렬 중'), ('HOLD', '벽 방향 확인 대기'),
                         ('SETTLING', '안정 확인'), ('CONFIRMED', '확인 완료')]:
        state['marker_alignment'] = {'state': phase, 'reason': '실제 정렬 상태'}
        result = operation_summary(state)
        assert result['phase'] == 'confirming'
        assert label in result['label']
        assert result['detail'] == '실제 정렬 상태'
        assert result['next_action']['target'] == 'camera'
        if phase in {'WALL_ALIGNING', 'HOLD'}:
            assert '차체의 앞뒤 축이 벽과 직각' in result['next_action']['action']
            assert '라이다 입력' in result['next_action']['action']
    state['mission']['state'] = 'PAUSED'
    state['marker_alignment'] = {'state': 'FAILED', 'reason': '표식 탐색 시간 초과'}
    result = operation_summary(state)
    assert result['phase'] == 'paused' and result['level'] == 'warning'
    assert '표식' in result['label'] and '시간 초과' in result['detail']
    assert '재개' in result['next_action']['action']
    assert '자동 출발하지 않습니다' in result['next_action']['action']


def test_manual_and_estop_precede_camera_alignment_display():
    state = base()
    state['mission']['state'] = 'CONFIRMING'
    state['marker_alignment'] = {'state': 'CENTERING'}
    state['manual'] = {'active': True}
    assert operation_summary(state)['phase'] == 'manual'
    state['safety'] = {'latched': True}
    assert operation_summary(state)['phase'] == 'estopped'


def test_previous_sector_alignment_never_overrides_current_mission():
    state = base()
    state['mission'].update(state='PAUSED', current_marker_id=3, reason='운용자 일시정지')
    state['marker_alignment'] = {'state': 'FAILED', 'marker_id': 2, 'reason': '이전 섹터 오류'}
    result = operation_summary(state)
    assert result['label'] == '임무 일시정지'
    assert result['detail'] == '운용자 일시정지'
