#!/usr/bin/env python3
# ============================================================================
# live_view_server.py
# 이미 돌고 있는 taillight_detection_test.py(systemd 서비스)를 건드리지 않고,
# 그 스크립트가 계속 갱신하고 있는 ~/latest_frame.jpg 파일을 웹페이지로 계속
# 새로고침하며 보여주는 "가짜 실시간 스트리밍" 서버.
#
# (진짜 MJPEG 스트리밍은 아니고, 0.5초마다 이미지를 다시 불러오는 방식이라
#  완전히 매끄러운 동영상은 아니지만, "지금 카메라가 뭘 보고 있는지" 확인하는
#  용도로는 충분함. 외부 라이브러리(Flask 등) 설치 필요없이 파이썬 기본 기능만 씀)
#
# 사용법:
#   1) python3 live_view_server.py   ← Pi 터미널(SSH)에서 실행
#   2) VS Code로 Remote-SSH 연결돼있으면, 8080 포트가 자동으로 감지되면서
#      화면 오른쪽아래에 "포트 포워딩됨" 알림이 뜸 → "브라우저에서 열기" 클릭
#      (자동으로 안 뜨면 VS Code 하단 "PORTS" 탭에서 8080 수동으로 추가)
#   3) 브라우저(또는 VS Code 안의 Simple Browser)에서 http://localhost:8080 접속
#   4) 종료하려면 Ctrl+C
# ============================================================================
import http.server
import socketserver
import os

PORT = 8080
FRAME_PATH = os.path.expanduser("~/latest_frame.jpg")

HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>SecondEye 실시간 확인용</title>
  <style>
    body { background:#111; color:#eee; font-family:sans-serif; text-align:center; margin:0; padding:20px; }
    img { max-width:95%; border:2px solid #444; margin-top:10px; }
    #ts { color:#888; font-size:0.9em; margin-top:8px; }
  </style>
</head>
<body>
  <h2>SecondEye - 카메라 실시간 확인 (0.15초마다 갱신)</h2>
  <img id="frame" src="/frame.jpg">
  <div id="ts"></div>
  <script>
    function refresh() {
      const img = document.getElementById('frame');
      img.src = '/frame.jpg?t=' + Date.now();   // 캐시 방지용으로 매번 다른 주소로 요청
      document.getElementById('ts').innerText = '마지막 갱신: ' + new Date().toLocaleTimeString();
    }
    setInterval(refresh, 150);   // ★변경: 0.5초→0.15초 (taillight_detection_test.py의 SAVE_EVERY_N_FRAMES도 5→2로 같이 낮춤)
  </script>
</body>
</html>
"""

class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # 매 요청마다 터미널에 로그 찍히는 거 방지 (안 그러면 로그가 너무 시끄러움)

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            self.send_response(200)
            self.send_header("Content-type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode("utf-8"))
        elif self.path.startswith("/frame.jpg"):
            if os.path.exists(FRAME_PATH):
                with open(FRAME_PATH, "rb") as f:
                    data = f.read()
                self.send_response(200)
                self.send_header("Content-type", "image/jpeg")
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_response(404)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

if __name__ == "__main__":
    with socketserver.TCPServer(("0.0.0.0", PORT), Handler) as httpd:
        print(f"[live_view_server] http://<Pi IP>:{PORT} 에서 접속 가능 (Ctrl+C로 종료)")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n종료함")
