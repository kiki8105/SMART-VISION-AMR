#!/usr/bin/env python3
"""Relay the PyQt controller's private Twist requests through the web manual lease.

This process never publishes /cmd_vel. The web server and robot motion_mux retain
the only authority to emit vehicle velocity, including their existing ESTOP and
LiDAR/odom checks. Run on the same PC as the classroom web server.
"""
import argparse
import http.client
import json
import math
import signal
import time
import uuid


REQUEST_TOPIC = '/smart_vision_amr/controller_manual_request'
STATUS_TOPIC = '/smart_vision_amr/controller_manual_status'


class WebManualClient:
    def __init__(self, host='127.0.0.1', port=8790, domain_id=16, timeout=.35):
        self.host, self.port, self.domain_id, self.timeout = host, port, domain_id, timeout
        self.cookie = None

    def _request(self, method, path, payload=None):
        connection = http.client.HTTPConnection(self.host, self.port, timeout=self.timeout)
        headers = {'Content-Type': 'application/json'}
        if self.cookie:
            headers['Cookie'] = self.cookie
        try:
            connection.request(method, path, None if payload is None else json.dumps(payload), headers)
            response = connection.getresponse()
            body = response.read()
            result = json.loads(body)
            if response.status >= 400:
                if response.status == 401:
                    self.cookie = None  # A restarted web process invalidated this session.
                raise RuntimeError(result.get('error', f'HTTP {response.status}'))
            if path == '/api/pair':
                self.cookie = response.getheader('Set-Cookie', '').split(';', 1)[0]
                if not self.cookie.startswith('amr_session='):
                    raise RuntimeError('웹 조종 세션 인증 응답이 없습니다.')
            return result
        finally:
            connection.close()

    def pair(self):
        return self._request('POST', '/api/pair', {'domain_id': self.domain_id})

    def state(self):
        if not self.cookie:
            self.pair()
        return self._request('GET', '/api/state')

    def manual(self, payload):
        if not self.cookie:
            self.pair()
        return self._request('POST', '/api/manual', payload)['manual']


class ManualRelay:
    """Keep only the newest request; never arm nonzero motion on an unacknowledged lease."""
    def __init__(self, client, clock=time.monotonic):
        self.client, self.clock = client, clock
        self.vector = (0., 0.)
        self.input_at = -math.inf
        self.control_id = None
        self.seq = 0
        self.armed = False
        self.available = False
        self.reason = '웹 조종 연결을 기다립니다.'
        self.last_state_at = -math.inf
        self.last_state = None

    def on_twist(self, linear, angular):
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.vector = (0., 0.)
        else:
            self.vector = (max(-.12, min(.12, linear)), max(-.60, min(.60, angular)))
        self.input_at = self.clock()

    def _post(self, active, linear=0., angular=0.):
        self.seq += 1
        return self.client.manual({'control_id': self.control_id, 'seq': self.seq,
                                   'active': active, 'linear': linear, 'angular': angular})

    def release(self):
        if self.control_id is not None:
            try:
                self._post(False)
            except (OSError, RuntimeError, ValueError) as exc:
                self.reason = '조종 해제 응답 없음 · 서버의 0.5초 데드맨 정지 대기: '+str(exc)
            finally:
                self.control_id = None
                self.armed = False
                self.vector = (0., 0.)
                self.input_at = -math.inf
                self.available = False
                self.last_state = None
                if not self.reason.startswith('조종 해제 응답 없음'):
                    self.reason = '조종권 반환 확인 중'

    def tick(self):
        now = self.clock()
        moving = any(self.vector) and now-self.input_at <= .28
        if self.control_id is not None and not moving:
            self.release()
        if self.control_id is None:
            if now-self.last_state_at >= .35 or self.last_state is None:
                try:
                    self.last_state = self.client.state()
                    self.last_state_at = now
                    manual = self.last_state.get('manual') or {}
                    self.available = bool(manual.get('available')) and manual.get('state') == 'IDLE'
                    self.reason = manual.get('reason', '') if not self.available else '조종 입력 대기'
                except (OSError, RuntimeError, ValueError) as exc:
                    self.available, self.last_state = False, None
                    self.reason = '웹 조종 서버 연결 실패: '+str(exc)
            if not moving or not self.available:
                return
            self.control_id, self.seq, self.armed = 'pyqt-'+uuid.uuid4().hex, 0, False
        # Send zero until the web bridge acknowledges ACTIVE. On the next tick,
        # and only while a fresh hold-to-drive input is present, send motion.
        try:
            status = self._post(True, *(self.vector if self.armed else (0., 0.)))
            if status.get('owner') is not True or status.get('state') not in {'ACQUIRING', 'ACTIVE'}:
                self.reason = status.get('reason') or '웹 조종권을 얻지 못했습니다.'
                self.release()
                return
            self.armed = status['state'] == 'ACTIVE'
            self.available = status.get('available') is True
            self.reason = '조종권 연결됨' if self.armed else '조종권 확보 중'
            if not self.available:
                self.release()
        except (OSError, RuntimeError, ValueError) as exc:
            self.available = False
            self.reason = '조종 응답 실패 · 데드맨 정지 대기: '+str(exc)
            self.release()

    def status(self):
        return {'available': self.available, 'armed': self.armed,
                'reason': self.reason, 'input_age': max(0., self.clock()-self.input_at)
                if math.isfinite(self.input_at) else None}


def main():
    from geometry_msgs.msg import Twist
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--web-port', type=int, default=8790)
    parser.add_argument('--domain-id', type=int, default=16)
    args = parser.parse_args()
    relay = ManualRelay(WebManualClient(port=args.web_port, domain_id=args.domain_id))
    rclpy.init()
    node = Node('controller_manual_bridge')
    status_pub = node.create_publisher(String, STATUS_TOPIC, 1)
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Twist, REQUEST_TOPIC,
                             lambda msg: relay.on_twist(float(msg.linear.x), float(msg.angular.z)), qos)

    def tick():
        relay.tick()
        packet = String()
        packet.data = json.dumps(relay.status(), ensure_ascii=False)
        status_pub.publish(packet)

    node.create_timer(.1, tick)
    def stop_signal(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop_signal)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        relay.release()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
