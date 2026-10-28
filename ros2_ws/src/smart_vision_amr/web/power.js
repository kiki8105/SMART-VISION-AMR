'use strict';
// Shared presentation only: the ROS bridge decides whether a battery sample is fresh.
window.VehiclePower = Object.freeze({
  render(element, power = {}, disconnected = false) {
    if (!element) return;
    power = power || {};
    const validVoltage = typeof power.voltage === 'number' && Number.isFinite(power.voltage) && power.voltage > 0;
    const age = typeof power.age === 'number' && Number.isFinite(power.age) && power.age >= 0 ? power.age : null;
    const status = disconnected ? 'disconnected' : ['online', 'stale', 'invalid'].includes(power.status) ? power.status : 'waiting';
    const labels = {online: '전원 신호 수신', stale: '전원 신호 지연', invalid: '전압 확인 필요', waiting: '전원 신호 대기', disconnected: '웹 연결 확인'};
    const oldValue = validVoltage && (status === 'stale' || status === 'disconnected');
    const voltage = validVoltage ? `${power.voltage.toFixed(2)} V${oldValue ? ' · 이전값' : ''}` : '— V';
    const details = [
      `차량 전원 신호 · ${labels[status]} · ${voltage}`,
      `토픽: ${power.source || '/battery_state'}`,
      age === null ? '아직 전압 데이터를 받지 못했습니다.' : `마지막 수신: ${age.toFixed(1)}초 전`,
      status === 'invalid' ? '수신한 전압이 유효하지 않습니다.' : '',
      status === 'disconnected' ? '웹 서버와 연결이 끊겨 현재 상태를 확인할 수 없습니다.' : '',
      '전압 토픽 수신 상태를 표시합니다. 전원 차단과 네트워크 단절은 구분하지 못합니다.',
    ].filter(Boolean).join('\n');
    element.dataset.status = status;
    element.querySelector('.power-state').textContent = labels[status];
    element.querySelector('.power-voltage').textContent = voltage;
    element.title = details;
    element.setAttribute('aria-label', details.replaceAll('\n', '. '));
  },
});
