#!/usr/bin/env python3
"""Read-only web window for an isolated, intermittently running Gazebo trial.

This server has no ROS node or publisher. It proxies GET requests to the sole
web mission owner on 8792. Only session pairing POST is forwarded; all control
and mutation POSTs are rejected. When a stage ends, the waiting page refreshes
until the next owner's web server appears.
"""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen


WAITING_HTML = '''<!doctype html><html lang="ko"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta http-equiv="refresh" content="3"><title>교실 AMR 관제 대기</title>
<style>body{font:17px system-ui,sans-serif;background:#f7f8fa;color:#243440;
max-width:700px;margin:12vh auto;padding:24px}a{color:#1468af}</style>
<h1>가제보 시험 대기 중</h1><p>현재 단계가 끝나 다음 시험을 준비하고 있습니다.
이 화면은 3초마다 새로고침됩니다.</p>
<p>시험 중에는 같은 주소에서 지도와 전·후방 카메라를 읽기 전용으로 표시합니다.
임무·조종은 이 창에서 실행되지 않습니다.</p></html>'''.encode('utf-8')


def make_handler(upstream):
    class ReadOnlyMonitor(BaseHTTPRequestHandler):
        server_version = 'ClassroomReadOnlyMonitor/1.0'

        def _send(self, code, body, content_type='text/plain; charset=utf-8', headers=()):
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            for key, value in headers:
                if key.lower() in {'set-cookie', 'content-disposition'}:
                    self.send_header(key, value)
            self.end_headers()
            self.wfile.write(body)

        def _upstream(self, method, body=None):
            parsed = urlsplit(self.path)
            if parsed.scheme or parsed.netloc or not parsed.path.startswith('/'):
                self._send(400, b'Invalid request target')
                return
            path = parsed.path+('?' + parsed.query if parsed.query else '')
            request_headers = {'Accept': self.headers.get('Accept', '*/*'),
                               'User-Agent': self.headers.get('User-Agent', 'classroom-monitor')}
            if self.headers.get('Cookie'):
                request_headers['Cookie'] = self.headers['Cookie']
            if body is not None:
                request_headers['Content-Type'] = self.headers.get(
                    'Content-Type', 'application/json')
            try:
                with urlopen(Request(upstream+path, data=body,
                                     headers=request_headers, method=method), timeout=5) as response:
                    data = response.read()
                    content_type = response.headers.get('Content-Type',
                                                        'application/octet-stream')
                    response_headers = [('Set-Cookie', value) for value in
                                        response.headers.get_all('Set-Cookie', [])]
                    if response.headers.get('Content-Disposition'):
                        response_headers.append(('Content-Disposition',
                                                 response.headers['Content-Disposition']))
                    if method == 'GET' and parsed.path in {'/', '/index.html'} and (
                            'text/html' in content_type):
                        banner = ('<div style="position:sticky;top:0;z-index:99999;'
                                  'padding:8px;background:#fff2c5;color:#513d00;'
                                  'text-align:center;font:600 14px system-ui">'
                                  '읽기 전용 관제 · 임무·조종 요청은 차단됩니다'
                                  '</div>').encode('utf-8')
                        close = data.lower().find(b'</body>')
                        data = data[:close]+banner+data[close:] if close >= 0 else data+banner
                    self._send(response.status, data, content_type, response_headers)
            except HTTPError as exc:
                self._send(exc.code, exc.read(), exc.headers.get('Content-Type', 'text/plain'))
            except (URLError, TimeoutError, OSError):
                if method == 'GET' and parsed.path in {'/', '/index.html'}:
                    self._send(200, WAITING_HTML, 'text/html; charset=utf-8')
                else:
                    self._send(503, b'{"error":"simulation stage is offline"}',
                               'application/json')

        def do_GET(self):
            if self.path == '/__readonly_proxy_health':
                body = json.dumps({'read_only_proxy': True, 'ros_owner': False,
                                   'upstream_port': 8792}).encode()
                self._send(200, body, 'application/json')
                return
            self._upstream('GET')

        def do_POST(self):
            if urlsplit(self.path).path != '/api/pair':
                self._send(405, b'{"error":"read-only monitor; open 8792 for controls"}',
                           'application/json')
                return
            length = int(self.headers.get('Content-Length', '0'))
            if length < 0 or length > 4096:
                self._send(413, b'Pairing request too large')
                return
            self._upstream('POST', self.rfile.read(length))

        def do_PUT(self):
            self._send(405, b'Read-only monitor')

        do_PATCH = do_PUT
        do_DELETE = do_PUT

    return ReadOnlyMonitor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8791)
    parser.add_argument('--upstream', default='http://127.0.0.1:8792')
    args = parser.parse_args()
    if args.upstream != 'http://127.0.0.1:8792':
        parser.error('Only the isolated local simulation owner 8792 is allowed')
    server = ThreadingHTTPServer((args.host, args.port), make_handler(args.upstream))
    print(f'Read-only simulation monitor: {args.host}:{args.port} → {args.upstream}', flush=True)
    try:
        server.serve_forever(poll_interval=.5)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
