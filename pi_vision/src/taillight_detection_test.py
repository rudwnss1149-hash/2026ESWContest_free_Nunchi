"""
★대폭 변경: "차를 먼저 찾고 그 안에서 브레이크등을 찾는" YOLO 기반 방식을 버리고,
"화면의 정해진 관심영역(ROI) 안에서 빨간 덩어리를 직접 찾아 밝기변화를 감지"하는 방식으로 전환.

왜 바꿨나:
- 실제 도로 거리에서는 앞차가 화면에 너무 작게 잡혀서 YOLO가 차량 자체를 인식 못하는 문제가 계속 있었음
- 근데 우리한테 진짜 필요한 건 "차량 인식"이 아니라 "앞차 후미등 영역에서 밝기가 급격히 늘었는가"뿐임
- 차 전체를 인식할 필요 없이, 카메라가 정면 고정이라 앞차는 대략 화면 중앙 부근에 있을 거라는 전제로
  그 구역(ROI)만 보고 빨간 덩어리 찾으면 훨씬 작은 물체도 잡을 수 있고, YOLO 추론 자체가 없어져서 훨씬 빠름

측정 방식:
- 밝기값은 단순 R채널 평균이 아니라 "redness = R - (G+B)/2" (빨간정도)로 계산
  → 헤드라이트나 전체적인 노출변화(흰 빛)로 화면이 밝아지는 것과, 실제 빨간 브레이크등이 켜지는 걸 더 잘 구분함
- ON/OFF 판정은 "최근 25초간 관찰값 중 최솟값"을 baseline으로 계속 실시간 갱신하면서 비교함(v3/v4 참고)
  (프레임 간 순간 변화량만 보면, 브레이크등이 몇 초씩 계속 켜져있을 때 "꺼졌다"고 착각하는 문제가 있어서
   장기(25초) 기준값 방식을 씀 — 대신 blob "위치를 찾는" 단계에서는 HSV 빨간색 검출을 그대로 씀)

(Pi 등 화면(모니터) 없는 환경용 — cv2.imshow 대신 파일 저장 + 터미널 로그 방식)

===== ★변경(v2): 레이더는 "화면을 좁게 자르는 용도"가 아니라 "여러 후보 중 고르는 힌트"로만 씀 =====
(1차 버전은 레이더 각도로 스캔 범위 자체를 좁혔었는데, 레이더 자체도 100% 안정적이지 않다는 우려가 있어서
 더 보수적인 방식으로 바꿈 — "레이더가 틀려도 최소한 지금 방식(고정 ROI 넓게 스캔)보다는 절대 나빠지지 않게")

동작 방식:
- 카메라 스캔 범위(ROI)는 항상 고정 넓은 영역(ROI_X1~X2, Y1~Y2) 그대로 유지함 — 레이더 상태와 무관하게
  "혹시 놓치는 일"이 없도록 하는 안전판. 레이더가 죽어도 이 스캔 자체는 절대 안 좁아짐
- 레이더가 "신선하고(RADAR_STALE_TIMEOUT_SEC 이내) + 최근 N개 연속 유효 샘플(RADAR_MIN_SAMPLES)"을
  만족할 때만 "신뢰 가능"으로 보고, 그 여러 샘플의 중앙값(median)을 써서 노이즈(한 프레임 튐)를 걸러냄
  → 단순 이동평균이 아니라 median을 쓰는 이유: 평균은 한 번 크게 튄 값에도 끌려가지만 median은 안 끌려감
- 레이더가 신뢰 가능하면: ROI 안에서 찾은 여러 빨간 블롭 중, 레이더가 가리키는 위치 근처(RADAR_GATE_HALF_WIDTH_RATIO)에
  있는 것들만 골라서 밝기 계산에 씀 (손/가로등/옆차선 후미등처럼 레이더 위치와 무관한 블롭은 자동 제외)
- 근데 만약 레이더가 가리키는 위치 근처에 빨간 블롭이 하나도 없으면(레이더와 카메라가 서로 다른 얘기를 하는 상황
  = 레이더가 순간적으로 틀렸거나, 실제로 그 위치에 빨간불이 없거나) → 그 프레임은 레이더를 무시하고
  ROI 안의 모든 블롭을 그대로 씀 (기존 방식과 동일) → 레이더가 틀려도 신호를 완전히 놓치지는 않음
- 레이더 신호 자체가 없거나(거리=-1) 오래됐거나 샘플이 부족하면 애초에 게이팅을 안 하고 항상 전체 블롭 사용

===== STM32 통신 프로토콜 =====
- STM32가 200ms마다 USART2로 보내는 "R,거리,상대속도,각도\r\n"을 백그라운드 스레드로 계속 읽어서
  latest_radar_distance / latest_radar_speed / latest_radar_angle 전역변수 + radar_angle_history(최근 N개)에 저장해둠
- 베이스라인이 설정된 이후, 밝기 델타를 계산할 때마다 "B,밝기델타,이상여부\r\n"을 STM32로 전송함
  (이상여부: delta가 임계값보다 크면 1=밝아짐 감지됨(정상적으로 브레이크등 켜짐), 아니면 0=변화없음)
- 매 프레임(200ms 속도제한 적용) "C,검출여부\r\n"도 전송함 (LCD 실시간 표시 전용, 판정 로직과 무관)

사용법: python src/taillight_detection_test.py
   Ctrl+C: 종료
"""
import cv2
import numpy as np
import time
import math                                          # 레이더-카메라 위치 차이를 보정하는 삼각함수 계산용 (atan2, sin, cos)
import statistics                                    # 레이더 각도의 median(중앙값) 계산용 — 한 프레임 튀는 값에 안 끌려가게
from collections import deque
import serial                                        # STM32와 시리얼(UART) 통신을 하기 위한 라이브러리
import threading                                     # 시리얼 읽기를 메인 영상처리 루프와 분리된 스레드로 돌리기 위함

SMOOTHING_FRAMES = 5
# ★변경(v4): 절대값 차이(delta) 대신 baseline 대비 "비율(%)"로 판정 기준을 바꿈
# → 앞차와의 거리에 따라 redness/면적의 절대 크기 자체가 달라지므로(가까울수록 크게 잡힘),
#   절대 차이 기준(예: +15)으로는 가까이/멀리 있을 때 기준이 안 맞음. 비율로 보면 거리 무관하게 일관됨
REDNESS_RATIO_THRESHOLD = 0.35   # baseline 대비 redness(색 진하기)가 이 비율(35%) 이상 늘면 "밝아짐" 후보
# ★변경(실차테스트, 2차): 300%로 올렸는데도 또 다른 OFF 프레임에서 +410% 나와서 오탐 재발함
# (지금까지 실측 누적: OFF 노이즈 = +163%, +410% / 진짜 ON = +5230%, +12095%, +14293% — 매번 최소 12배 이상 차이는 유지되고 있음)
# → 임계값을 1500%로 크게 올려서 OFF 노이즈 쪽에 훨씬 넉넉한 여유(3.6배)를 둠 + baseline 자체도
#   percentile=20으로 바꿔서(위 SlidingMinBaseline 참고) 분모(baseline)가 어쩌다 너무 작게 잡히는 것 자체를 완화함
# ⚠ OFF 노이즈가 163%→410%로 계속 커지는 추세라, 이후에도 또 뚫리면 "비율 임계값 계속 올리기"보다
#   redness랑 area를 AND로 묶는 방향으로 설계를 바꾸는 걸 고려해야 함
AREA_RATIO_THRESHOLD = 15.00     # baseline 대비 면적(빛이 번져 보이는 크기)이 이 비율(1500%) 이상 늘면 "밝아짐" 후보

# ★추가(테스트용, 실차): 니로 엔진브레이크(회생제동)가 약해서 "감속인데 브레이크등 안 켜짐" 상황을
# 실제로 재현하기 어려워서, 카메라 판단을 무시하고 강제로 "밝기변화 없음"으로 STM32에 보고하는 테스트 스위치.
# True로 켜면 실제로 브레이크를 밟든 안 밟든 상관없이 계속 anomaly_flag=0(브레이크등 안 보임)으로 전송됨
# → 이 상태에서 그냥 감속(가볍게 브레이크 밟기 등)만 해봐도 STM32의 WARN 로직이 제대로 반응하는지 검증 가능.
# ⚠⚠ STM32쪽 로직만 따로 테스트할 때만 켜두고, 실제 카메라 판단까지 포함한 진짜 검증할 땐 반드시 False로 되돌릴 것!
# ★변경: 이제 실제로 미등/브레이크등을 물리적으로 껐다 켰다 하면서 테스트하니까, 진짜 카메라 판단(area 절대값 기준)이
# 그대로 STM32에 전달되도록 다시 False로 되돌림 (더 이상 강제로 고정할 필요 없음)
FORCE_NO_BRIGHTENING_TEST = False

# ★추가(실차테스트, 핵심 변경): 실측 데이터 보니 area의 "절대값" 자체가 ON/OFF를 훨씬 확실하게 갈라줌
# (확인된 ON area: 53290, 34314, 86234 / 확인된 OFF area(블롭 잡힌 경우): 2073, 7304 — 최소 4.7배 차이)
# redness는 절대값으론 못 씀(ON=68.6이 OFF=69.8보다 낮게 나온 적도 있어서 겹침)
# 그리고 평소엔 블롭 자체가 거의 안 잡혀서 baseline(비율 방식)이 제대로 못 쌓이는 문제도 있었는데,
# 이 절대값 기준은 baseline/warmup이랑 완전히 무관하게 항상 바로 적용 가능 → 훨씬 안정적
# 확인된 OFF 최대(7304)랑 ON 최소(34314) 사이에서 여유 있게 15000으로 설정
ABS_AREA_ON_THRESHOLD = 10000   # ★변경: 줌 3.5배로 줄이면서 블롭 크기 자체가 작아져서 임계값도 낮춤 (35000→10000, 실차 확인값)
# ★변경: 색 진하기(redness) OR 면적(area) 둘 중 하나라도 크게 튀면 "밝아짐"으로 판정
# (브레이크등은 색만 진해지는 게 아니라 빛이 번지면서 눈에 보이는 면적도 커지는 경우가 많아서,
#  두 신호를 같이 보면 한쪽만 볼 때보다 더 안정적으로 잡을 수 있음)
MIN_RED_PIXELS = 8              # ★변경: 멀리 있는 작은 빛도 잡아야 하니 기존(15)보다 낮춤
PADDING = 5
SAVE_EVERY_N_FRAMES = 2   # ★변경: 5→2 (live_view_server 화면 갱신 체감 속도를 위해 더 자주 저장, Pi CPU 여유 있으면 문제없음)
# ★변경(v3): "시작할 때 딱 한 번" 잡고 계속 쓰는 baseline은 실제 주행이랑 안 맞음 — 앞차가 계속
# 바뀌는데(추월당함/앞차가 빠짐/신호에서 다른 차) 세션 시작 시점 값 하나로 주행 내내 버티는 건 무리임
# → "최근 BASELINE_WINDOW_SEC초 동안 관찰한 값 중 최솟값"을 baseline으로 계속 실시간 갱신함
#   (미등 상태가 항상 제일 어두우니, 최근 몇십 초 안에 브레이크 안 밟은 순간이 한 번은 있었을 것이라는 전제)
# 트레이드오프: 이 시간(윈도우)보다 더 오래 브레이크를 계속 밟고 있으면 baseline이 그 밝은 값 쪽으로
# 끌려 올라가서 오판 위험 있음 — 근데 도로에서 30초씩 계속 브레이크 밟는 경우는 거의 없다고 보고 이 값으로 설정
BASELINE_WINDOW_SEC = 25.0     # 최근 몇 초 데이터로 baseline(최솟값)을 계산할지
BASELINE_WARMUP_SEC = 3.0      # 최소 이 정도는 데이터가 쌓여야 baseline을 신뢰함 (콜드스타트에 값 1개짜리가 baseline 되는 거 방지)

# ===== ★추가: 관심영역(ROI) 설정 — 화면 전체가 아니라 이 구역 안에서만 빨간 덩어리를 찾음 =====
# 카메라가 차량 정면 고정이라, 내 차선 앞차는 대략 이 범위 안에 있을 거라는 전제
# (실제로 카메라 붙여놓은 각도 보면서 미세조정 필요 — 조정하고 싶으면 이 4개 비율만 바꾸면 됨)
ROI_X1_RATIO = 0.20   # 왼쪽 경계 (화면 가로의 20% 지점부터)
ROI_X2_RATIO = 0.80   # 오른쪽 경계 (화면 가로의 80% 지점까지)
ROI_Y1_RATIO = 0.30   # 위쪽 경계 (하늘/윗부분 제외)
ROI_Y2_RATIO = 0.90   # 아래쪽 경계 (내 차 보닛/대시보드 제외)
MAX_BLOBS_IN_ROI = 3    # ROI 안에서 최대 몇 개의 빨간 덩어리까지 고려할지 (그중 밝기 평균 큰 것들 위주로 씀)

# ===== ★변경(v2): 레이더는 ROI를 좁히는 게 아니라, 찾은 블롭 중 "이게 진짜다" 골라주는 힌트로만 씀 =====
CAMERA_HFOV_DEG = 60.0             # 웹캠의 대략적인 수평 화각(도) — 실측값 아니면 일반 웹캠 평균치로 가정, 나중에 실측해서 조정 필요
RADAR_GATE_HALF_WIDTH_RATIO = 0.20   # 레이더가 가리키는 위치 기준, 화면 가로폭의 ±몇 % 안에 있는 블롭까지 "레이더와 일치"로 볼지
RADAR_STALE_TIMEOUT_SEC = 1.0      # 이 시간 이상 레이더 갱신이 없으면 "레이더 없음"으로 보고 게이팅 안 함
RADAR_ANGLE_HISTORY_LEN = 5        # 각도 median 계산에 쓸 최근 샘플 개수
RADAR_MIN_SAMPLES = 3              # 이 개수 이상 연속 유효 샘플이 쌓여야 레이더를 신뢰함 (막 재연결된 직후 튀는 값 방지용 디바운스)
RADAR_ANGLE_SIGN = 1.0              # ★실측 전 임시값: 레이더 각도의 +/-가 실제 왼쪽/오른쪽 중 어느 쪽인지 아직 확인 안 됨
                                     # (실차에서 확인해서 반대면 -1.0으로 바꾸면 됨)

# ===== ★추가: 레이더-카메라 실측 장착 위치 차이 보정용 (2026-08-22 실측값) =====
# 레이더 = 쏘렌토 앞 그릴 중앙(기아 엠블럼 부근), 카메라 = 실내 룸미러(정중앙에서 조수석 방향 30cm),
# 카메라가 레이더보다 차량 뒤쪽으로 약 1.8~1.9m 떨어져 있음 (앞뒤 거리 차이가 좌우 매핑에 미치는 영향 보정)
# (높이 차이는 무시함 — 지금은 좌우 픽셀 위치만 계산하므로 상하 위치 차이는 이 계산에 영향 없음)
RADAR_TO_CAM_FORWARD_OFFSET_M = 1.85   # 카메라가 레이더보다 뒤쪽에 있는 거리(m) — 1.8~1.9m의 평균값
RADAR_TO_CAM_LATERAL_OFFSET_M = 0.30   # 카메라가 레이더 중심선보다 조수석 방향으로 치우친 거리(m)

# 캡처 해상도를 미리 높여서 요청함 (웹캠이 지원하면). 원본 해상도가 높아야 작은 빛도 디테일이 살아있음
CAPTURE_WIDTH = 1280
CAPTURE_HEIGHT = 720
WORK_WIDTH = 640    # 실제 처리(빨간블롭 찾기 등)에 쓸 작업용 해상도 (너무 크면 Pi CPU 부담)
WORK_HEIGHT = 480

# ===== ★추가: 디지털 줌 — 원본 캡처 프레임의 중앙 부분만 잘라서 WORK 해상도로 확대함 =====
# 멀리 있는 브레이크등이 화면에서 너무 작게(적은 픽셀 수로) 잡혀서 블롭 검출이 어려운 문제 완화용
# 반드시 "자르기(crop) 먼저 → 그 다음에 확대(resize)" 순서로 해야 함 (반대로 하면 이미 저해상도로
# 줄어든 걸 자르는 거라 확대 효과가 없음)
DIGITAL_ZOOM_CROP_RATIO = 0.2857   # ★변경: 5배(0.20)가 너무 크다고 해서 3.5배로 다시 줄임 (1/0.2857 ≈ 3.5배)
                                   # (빨간 블롭 색상/면적 변화만 보는 용도라 화질 저하는 크게 문제 안 됨)
                                   # 단, 앞차가 많이 가까워질 땐(거의 근접) 오히려 화면 밖으로 벗어날 수 있으니
                                   # 그 상황만 테스트 때 유의 (이건 화질과 무관한 별개 리스크)
                                  # 값을 낮출수록(예: 0.4) 더 확대됨, 대신 화면에 보이는 범위(FOV)는 더 좁아짐

# ===== ★STM32 시리얼 통신 설정 =====
STM32_PORT = '/dev/serial0'                         # 라즈베리파이의 GPIO UART 포트 (STM32 USART2와 연결된 핀)
                                                     # 안 열리면 '/dev/ttyAMA0'으로도 시도해볼 것 (Pi 설정에 따라 이름 다를 수 있음)
STM32_BAUD = 115200                                 # STM32의 huart2 baudrate와 반드시 동일해야 함 (main.c MX_USART2_UART_Init 참고)
STM32_SEND_INTERVAL_SEC = 0.2   # B,/C, 메시지를 STM32로 보내는 최소 간격 (STM32 쪽 200ms 루프주기와 맞춤, 수신버퍼 보호용)

latest_radar_distance = None                        # STM32가 보내준 최신 레이더 거리값(m) 저장, 아직 안 받았으면 None
latest_radar_speed = None                           # STM32가 보내준 최신 상대속도값(km/h) 저장
latest_radar_angle = None                           # STM32가 보내준 최신 레이더 각도값(도, 정면=0) 저장 (화면 표시/로그용, 단일값이라 노이즈 있음)
latest_radar_update_time = 0.0                      # 마지막으로 "유효한"(distance>=0) 레이더 데이터를 받은 시각
radar_angle_history = deque(maxlen=RADAR_ANGLE_HISTORY_LEN)  # ★추가: 최근 유효 각도값들 — median 계산해서 노이즈 억제하는 데 씀
serial_data_lock = threading.Lock()                 # 백그라운드 스레드랑 메인루프가 동시에 위 변수들을 건드리지 않도록 보호하는 락


class SlidingMinBaseline:
    # ★추가(v4): redness랑 area 둘 다 "최근 N초 관찰값 중 최솟값" 방식으로 baseline을 잡아야 해서,
    # 중복 코드를 피하려고 공통 로직을 클래스로 뽑음. 두 개(redness용, area용)를 각각 만들어서 씀.
    # ★변경(실차테스트): area는 순수 min(최솟값)으로 baseline을 잡으면, 어쩌다 한 프레임 area가
    # 유독 작게 잡히는 순간(예: 순간적으로 블롭이 작게 인식됨) 그 값이 그대로 baseline이 돼버려서,
    # 그 다음부터는 조금만 커져도 비율이 수백~수만 %로 폭발함(실측: OFF인데도 +163%, +410% 오탐 발생).
    # → percentile 옵션 추가: area처럼 노이즈에 민감한 지표는 "완전 최솟값"이 아니라 "하위 20% 지점" 정도로
    #   잡아서, 극단적으로 작은 프레임 1~2개가 baseline을 통째로 끌어내리지 못하게 함 (redness는 기존처럼 min 유지)
    def __init__(self, window_sec, warmup_sec, percentile=0):
        self.window_sec = window_sec
        self.warmup_sec = warmup_sec
        self.percentile = percentile   # 0=최솟값(min) 그대로, 그 외(예:20)=하위 N퍼센타일 값 사용
        self.buffer = deque()          # (시각, 값) 튜플들의 슬라이딩 윈도우
        self.first_sample_time = None  # 워밍업 시간 계산용
        self.value = None              # 현재 확정된 baseline (아직 워밍업 안 끝났으면 None)

    def update(self, now_t, sample):
        self.buffer.append((now_t, sample))
        if self.first_sample_time is None:
            self.first_sample_time = now_t
        while self.buffer and self.buffer[0][0] < now_t - self.window_sec:   # 오래된 값 버림
            self.buffer.popleft()
        if (self.first_sample_time is not None
                and (now_t - self.first_sample_time) >= self.warmup_sec
                and self.buffer):
            values = sorted(v for _, v in self.buffer)
            if self.percentile <= 0 or len(values) < 5:   # 데이터 적을 땐 percentile 계산이 불안정하니 그냥 min 사용
                self.value = values[0]
            else:
                idx = min(len(values) - 1, max(0, int(len(values) * self.percentile / 100.0)))
                self.value = values[idx]
        return self.value


def stm32_serial_reader(ser):                       # STM32에서 오는 "R,거리,속도,각도" 줄을 계속 읽어서 전역변수에 저장하는 함수 (별도 스레드에서 실행됨)
    global latest_radar_distance, latest_radar_speed, latest_radar_angle, latest_radar_update_time   # 함수 안에서 전역변수를 수정하겠다고 선언
    while True:                                          # 프로그램이 끝날 때까지 무한 반복
        try:                                               # 시리얼 읽기 중 에러가 나도 스레드가 죽지 않게 감싸줌
            raw_line = ser.readline()                        # 한 줄(개행까지) 읽기 시도, timeout 설정에 따라 없으면 빈 값 반환하고 넘어감
            if not raw_line:                                  # 아무것도 못 읽었으면(타임아웃)
                continue                                        # 그냥 다음 반복으로 (에러 아님, 정상적인 대기)
            line = raw_line.decode('utf-8', errors='ignore').strip()  # 바이트를 문자열로 변환, 앞뒤 공백/개행 제거
            if line.startswith('R,'):                          # STM32가 보내는 레이더 데이터 형식("R,거리,속도,각도")이면
                parts = line.split(',')                          # 쉼표 기준으로 쪼갬 → ["R", "거리", "속도", "각도"]
                # ★변경: 각도값이 추가돼서 이제 4개로 쪼개짐. 구버전 펌웨어(3개)도 혹시 몰라 하위호환으로 처리
                if len(parts) >= 3:                               # 최소 거리/속도까지는 있어야 함
                    try:                                            # 숫자 변환 실패(깨진 데이터 등) 대비
                        dist = float(parts[1])                        # 두 번째 조각을 거리값(실수)으로 변환
                        spd = float(parts[2])                         # 세 번째 조각을 속도값(실수)으로 변환
                        ang = float(parts[3]) if len(parts) >= 4 else None  # 네 번째 조각(각도)이 있으면 변환, 없으면 None
                        with serial_data_lock:                        # 락을 잡은 상태에서만 전역변수 수정(동시접근 방지)
                            # ★추가: 이전 유효 데이터로부터 시간이 너무 많이 지났으면(끊겼다 재연결 등)
                            # 과거 각도 이력을 그대로 이어붙이면 median이 옛날 값에 끌려갈 수 있으니 이력을 비우고 새로 시작
                            if (time.time() - latest_radar_update_time) > RADAR_STALE_TIMEOUT_SEC:
                                radar_angle_history.clear()
                            latest_radar_distance = dist                # 최신 거리값 갱신
                            latest_radar_speed = spd                    # 최신 속도값 갱신
                            latest_radar_angle = ang                    # 최신 각도값 갱신 (표시용)
                            if dist >= 0 and ang is not None:             # 진짜 유효한 타겟일 때만(거리=-1은 "타겟 없음") 이력에 반영
                                radar_angle_history.append(ang)
                                latest_radar_update_time = time.time()     # 갱신 시각 기록 (stale/게이팅 판단용)
                    except ValueError:                               # 숫자로 변환이 안 되면(깨진 줄)
                        pass                                            # 그냥 무시하고 다음 줄 기다림
        except Exception as e:                                    # 시리얼 포트 자체에 문제가 생기면(케이블 빠짐 등)
            print(f"[STM32] 시리얼 읽기 에러: {e}")
            time.sleep(1)                                            # 1초 쉬고 다시 시도


def find_red_blobs(frame, region, max_blobs):
    # HSV 색공간에서 "빨간색"에 해당하는 픽셀들을 찾아서, 덩어리(블롭)로 묶어 위치를 찾는 함수
    # (0~10도, 160~180도 두 구간을 OR로 합치는 이유: HSV에서 빨간색은 색상환의 양 끝에 걸쳐있기 때문)
    x1, y1, x2, y2 = region
    x1, y1 = max(0, int(x1)), max(0, int(y1))
    x2, y2 = int(x2), int(y2)
    crop = frame[y1:y2, x1:x2]
    if crop.size == 0:
        return []
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    # ★변경: 채도(S) 하한을 100→130으로 올림 — 손/피부색처럼 채도가 낮은 "옅은 빨강/핑크"는 걸러내고,
    # 브레이크등처럼 채도가 높은(진짜 진한 빨강) 빛만 남기려는 목적
    lower_red1 = np.array([0, 130, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 130, 100])
    upper_red2 = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)
    kernel = np.ones((3, 3), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(red_mask, connectivity=8)
    blob_areas = stats[1:, cv2.CC_STAT_AREA]
    if len(blob_areas) == 0:
        return []
    top_indices = np.argsort(blob_areas)[::-1][:max_blobs]
    results = []
    for idx in top_indices:
        label_id = idx + 1
        area = stats[label_id, cv2.CC_STAT_AREA]
        if area < MIN_RED_PIXELS:
            continue
        bx = stats[label_id, cv2.CC_STAT_LEFT]
        by = stats[label_id, cv2.CC_STAT_TOP]
        bw = stats[label_id, cv2.CC_STAT_WIDTH]
        bh = stats[label_id, cv2.CC_STAT_HEIGHT]
        final_x1 = x1 + max(0, bx - PADDING)
        final_y1 = y1 + max(0, by - PADDING)
        final_x2 = x1 + min(crop.shape[1], bx + bw + PADDING)
        final_y2 = y1 + min(crop.shape[0], by + bh + PADDING)
        results.append((int(final_x1), int(final_y1), int(final_x2), int(final_y2)))
    return results


def apply_digital_zoom(frame):
    # ★추가: 원본 프레임의 중앙 DIGITAL_ZOOM_CROP_RATIO 비율만 잘라낸 뒤, WORK 해상도로 다시 키움
    # (자르기 → 확대 순서가 중요함. 이미 축소된 걸 자르면 확대해도 디테일이 못 살아남)
    h, w = frame.shape[:2]
    crop_w = int(w * DIGITAL_ZOOM_CROP_RATIO)
    crop_h = int(h * DIGITAL_ZOOM_CROP_RATIO)
    x1 = (w - crop_w) // 2
    y1 = (h - crop_h) // 2
    cropped = frame[y1:y1 + crop_h, x1:x1 + crop_w]
    return cv2.resize(cropped, (WORK_WIDTH, WORK_HEIGHT))


def get_redness(frame, roi):
    # ★변경: 단순 R채널 평균 대신 "redness = R - (G+B)/2"를 씀
    # → 전체 화면이 밝아지는 것(헤드라이트, 노출변화)과 실제 빨간 브레이크등이 켜지는 걸 더 잘 구분함
    x1, y1, x2, y2 = roi
    x1, y1 = max(0, x1), max(0, y1)
    crop = frame[y1:y2, x1:x2]
    if crop.size == 0:
        return None
    b, g, r = cv2.split(crop.astype(np.int16))   # int16으로 변환 (뺄셈 중 음수/오버플로 방지)
    redness = r - (g + b) / 2.0
    redness = np.clip(redness, 0, None)          # 음수(빨간색이 아닌 부분)는 0으로 클리핑
    return float(np.mean(redness))


def get_radar_target_x(w):
    # ★추가(v2): 레이더가 "지금 신뢰할 만한지" 판단하고, 신뢰할 만하면 각도를 화면 픽셀 x좌표로 변환해서 반환.
    # 신뢰 못 하면 None을 반환 → 호출하는 쪽(scan_roi_for_brake_light)에서 게이팅을 아예 안 하고 안전하게 넘어감
    with serial_data_lock:
        dist = latest_radar_distance
        upd_time = latest_radar_update_time
        hist = list(radar_angle_history)   # 리스트로 복사해서 락 밖에서 계산 (락 잡는 시간 최소화)

    if dist is None or dist < 0:                                # 애초에 유효한 타겟이 없으면
        return None
    if (time.time() - upd_time) > RADAR_STALE_TIMEOUT_SEC:        # 마지막 유효 갱신이 너무 오래 전이면(연결 끊김 등)
        return None
    if len(hist) < RADAR_MIN_SAMPLES:                            # 아직 신뢰할 만큼 샘플이 안 쌓였으면(막 재연결된 직후 등)
        return None

    smoothed_angle = statistics.median(hist)                     # 최근 N개 중 중앙값 — 한 프레임 튀는 값에 안 끌려감

    # ===== ★추가: 레이더-카메라 장착 위치 차이 보정 =====
    # 레이더가 "각도 smoothed_angle, 거리 dist"라고 알려준 타겟 위치를, 레이더 기준 좌표계(전방=X, 우측=Y)로 먼저 바꾸고,
    # 카메라가 레이더보다 뒤쪽/조수석쪽으로 떨어진 만큼 좌표를 옮겨서 "카메라 입장에서의 각도"로 다시 계산함
    ang_rad = math.radians(smoothed_angle * RADAR_ANGLE_SIGN)       # 도(degree) → 라디안, 부호는 실측 전이라 RADAR_ANGLE_SIGN으로 임시 처리
    target_x_from_radar = dist * math.cos(ang_rad)                   # 레이더 기준, 타겟까지의 전방 거리
    target_y_from_radar = dist * math.sin(ang_rad)                   # 레이더 기준, 타겟까지의 좌우(+=조수석쪽) 거리
    # 카메라는 레이더보다 뒤쪽에 있으므로(전방좌표 -offset), 카메라 기준 전방거리 = 타겟전방거리 + 레이더-카메라 간격
    rel_x = target_x_from_radar + RADAR_TO_CAM_FORWARD_OFFSET_M
    # 카메라는 레이더 중심선보다 조수석쪽으로 치우쳐 있으므로, 그만큼 빼줌
    rel_y = target_y_from_radar - RADAR_TO_CAM_LATERAL_OFFSET_M
    camera_angle_deg = math.degrees(math.atan2(rel_y, rel_x))        # 카메라 입장에서 본 타겟의 각도(도)

    half_fov = CAMERA_HFOV_DEG / 2.0
    offset_ratio = max(-1.0, min(1.0, camera_angle_deg / half_fov))  # -1~1로 clamp (화각 밖 각도 방지)
    return (w / 2.0) + offset_ratio * (w / 2.0)                   # 화면 중심 기준으로 각도만큼 이동한 픽셀 x좌표
    # ★참고: camera_angle_deg의 부호가 화면 왼쪽/오른쪽 중 어느 쪽인지, 그리고 RADAR_ANGLE_SIGN이 맞는지는
    #        실측 전엔 확신 못 함 — 실측해서 반대로 움직이면 RADAR_ANGLE_SIGN을 -1.0으로 바꾸면 됨


def scan_roi_for_brake_light(frame):
    # ★변경(v2): 스캔 범위(ROI)는 항상 고정 넓은 영역 그대로 유지(레이더 죽어도 절대 안 좁아지는 안전판).
    # 레이더가 신뢰 가능하면, 찾은 블롭들 중 레이더 위치 근처에 있는 것만 "진짜"로 골라서 씀.
    # 레이더 위치 근처에 블롭이 하나도 없으면(레이더-카메라 불일치) → 안전하게 전체 블롭으로 폴백.
    # (예전 process_taillights()를 대체함 — YOLO 차량박스 없이 ROI 기준으로 동작)
    h, w = frame.shape[:2]
    rx1 = int(w * ROI_X1_RATIO)
    ry1 = int(h * ROI_Y1_RATIO)
    rx2 = int(w * ROI_X2_RATIO)
    ry2 = int(h * ROI_Y2_RATIO)
    cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), (200, 200, 0), 1)  # 고정 ROI 테두리 (항상 이 넓은 범위를 실제로 스캔함)

    blobs = find_red_blobs(frame, (rx1, ry1, rx2, ry2), max_blobs=MAX_BLOBS_IN_ROI)
    if not blobs:
        return None, 0, "NO_BLOB", 0.0

    radar_x = get_radar_target_x(w)          # None이면 레이더 신뢰 불가 → 게이팅 안 함
    used_blobs = blobs
    mode = "NO_RADAR"                         # 로그/화면 표시용 현재 모드
    if radar_x is not None:
        cv2.line(frame, (int(radar_x), ry1), (int(radar_x), ry2), (0, 140, 255), 2)  # 레이더가 가리키는 위치를 주황 세로선으로 표시
        gate_half = w * RADAR_GATE_HALF_WIDTH_RATIO
        gated = [b for b in blobs
                 if (radar_x - gate_half) <= ((b[0] + b[2]) / 2.0) <= (radar_x + gate_half)]  # 블롭 중심 x가 게이트 안에 있는지
        if gated:
            used_blobs = gated
            mode = "RADAR_GATED"              # 레이더-카메라 일치 → 게이팅된 블롭만 사용
        else:
            mode = "RADAR_DISAGREE"           # 레이더 위치엔 빨간 블롭이 없음 → 안전하게 전체 블롭 사용(신호 놓치지 않기 위해)

    redness_values = []
    total_area = 0.0   # ★추가(v4): 사용된 블롭들의 면적 합 — redness랑 별개로 "빛이 번져 보이는 크기" 신호로 씀
    for (bx1, by1, bx2, by2) in used_blobs:
        box_color = (0, 255, 0) if mode == "RADAR_GATED" else (0, 200, 200)  # 레이더로 골라진 블롭=초록, 아니면 청록으로 구분
        cv2.rectangle(frame, (bx1, by1), (bx2, by2), box_color, 2)
        rv = get_redness(frame, (bx1, by1, bx2, by2))
        if rv is not None:
            redness_values.append(rv)
        total_area += max(0, bx2 - bx1) * max(0, by2 - by1)   # 박스 면적(패딩 포함, 상대적 크기 비교용이라 이 정도면 충분)

    if redness_values:
        return float(np.mean(redness_values)), len(used_blobs), mode, total_area
    return None, 0, mode, total_area


def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return
    # 웹캠에게 최대한 높은 해상도로 찍어달라고 요청 (지원 안 하면 웹캠이 알아서 가장 가까운 값으로 맞춤)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[카메라] 요청 해상도 {CAPTURE_WIDTH}x{CAPTURE_HEIGHT} → 실제 적용된 해상도 {actual_w}x{actual_h}")

    # ===== ★STM32 시리얼 포트 연결 =====
    try:                                                  # 시리얼 포트 연결 시도, 실패해도 영상처리 자체는 계속 되게(레이더 없이) 처리
        stm32_ser = serial.Serial(STM32_PORT, STM32_BAUD, timeout=1)  # STM32 USART2와 연결된 포트를 baudrate 115200으로 열기
        reader_thread = threading.Thread(target=stm32_serial_reader, args=(stm32_ser,), daemon=True)
        # daemon=True: 메인 프로그램이 종료되면 이 스레드도 같이 종료되게 함(좀비 스레드 방지)
        reader_thread.start()                              # 백그라운드 스레드 시작 (이때부터 R, 데이터 계속 수신됨)
        print(f"[STM32] 시리얼 연결 성공: {STM32_PORT} @ {STM32_BAUD}bps")
    except Exception as e:                                # 포트가 없거나 권한 문제 등으로 연결 실패시
        stm32_ser = None                                    # STM32 연결 안 된 상태로 표시
        print(f"[STM32] 시리얼 연결 실패 (레이더 데이터 없이 영상처리만 진행): {e}")

    history = deque(maxlen=SMOOTHING_FRAMES)        # redness 스무딩용
    area_history = deque(maxlen=SMOOTHING_FRAMES)    # ★추가(v4): area 스무딩용 (redness랑 같은 방식)
    redness_baseline = SlidingMinBaseline(BASELINE_WINDOW_SEC, BASELINE_WARMUP_SEC)             # redness는 기존처럼 min 유지
    area_baseline = SlidingMinBaseline(BASELINE_WINDOW_SEC, BASELINE_WARMUP_SEC, percentile=20)  # ★변경: area는 하위 20%로 완화
    last_redness = None
    frame_count = 0
    last_b_send_time = 0.0
    last_c_send_time = 0.0
    baseline_announced = False   # ★추가: baseline이 처음 잡히는 순간 로그로 한 번 알려주기 위한 플래그
    print(f"Ctrl+C로 종료 | 2프레임마다 latest_frame.jpg 저장 | 베이스라인(redness/area 각각)은 최근 {BASELINE_WINDOW_SEC:.0f}초 최솟값으로 실시간 갱신")
    print(f"판정: baseline 대비 redness {REDNESS_RATIO_THRESHOLD*100:.0f}% 또는 area {AREA_RATIO_THRESHOLD*100:.0f}% 이상 증가 시 '밝아짐'(거리 무관 비율 기준)")
    print(f"스캔 ROI(항상 고정, 레이더 상태와 무관): 가로 {ROI_X1_RATIO*100:.0f}~{ROI_X2_RATIO*100:.0f}%, 세로 {ROI_Y1_RATIO*100:.0f}~{ROI_Y2_RATIO*100:.0f}%")
    print(f"레이더 게이팅: 신뢰 가능(연속 {RADAR_MIN_SAMPLES}샘플+최근 {RADAR_STALE_TIMEOUT_SEC:.0f}초 이내)할 때만, 위치 ±{RADAR_GATE_HALF_WIDTH_RATIO*100:.0f}% 안 블롭만 우선 사용 (불일치시 자동 폴백)")
    print(f"디지털 줌: 중앙 {DIGITAL_ZOOM_CROP_RATIO*100:.0f}% 크롭 → {WORK_WIDTH}x{WORK_HEIGHT}로 확대 (약 {1/DIGITAL_ZOOM_CROP_RATIO:.2f}배)")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("프레임을 읽을 수 없습니다.")
                break
            frame = apply_digital_zoom(frame)   # ★변경: 단순 리사이즈 대신 중앙 크롭+확대로 디지털 줌 적용
            frame_count += 1

            # ===== ★변경: YOLO 없이 매 프레임 바로 ROI에서 빨간 덩어리 스캔 (더 이상 N프레임마다 건너뛸 필요 없음, 훨씬 가벼움) =====
            redness, blob_count, radar_mode, area = scan_roi_for_brake_light(frame)
            blob_found = redness is not None

            if blob_found:
                last_redness = redness
                history.append(redness)
                area_history.append(area)
                smoothed = float(np.mean(history))
                smoothed_area = float(np.mean(area_history))
                now_t = time.time()

                # ===== ★변경(v4): redness/area 각각 슬라이딩 윈도우 baseline 갱신 =====
                r_base = redness_baseline.update(now_t, redness)
                a_base = area_baseline.update(now_t, area)
                if r_base is not None and not baseline_announced:
                    print(f"[자동] 베이스라인 첫 설정(frame={frame_count}): redness={r_base:.1f} (이후 계속 실시간 갱신됨)")
                    baseline_announced = True

                # ===== ★변경(v5): 판정은 절대 area 값 하나로만 함 (redness_ratio/area_ratio는 표시/전송용으로만 계산해서 남겨둠) =====
                redness_ratio = ((smoothed - r_base) / r_base) if (r_base is not None and r_base > 1e-6) else None
                area_ratio = ((smoothed_area - a_base) / a_base) if (a_base is not None and a_base > 1e-6) else None
                abs_area_hit = (area is not None and area >= ABS_AREA_ON_THRESHOLD)
                # ★변경: OR 없이 area 절대값 하나로만 판정 (redness_ratio/area_ratio는 표시/전송용으로만 계산해서 남겨둠)
                brightening = abs_area_hit

                if redness_ratio is not None:
                    # ★변경(용어 정정): "ANOMALY"는 오해 소지가 큼 — 여기서 밝아짐 판정은 "브레이크등이
                    # 정상적으로 켜짐(좋은 상황)"을 뜻하는데 화면엔 마치 문제 생긴 것처럼 보였음
                    # (진짜 "이상"은 STM32 쪽에서 감속중인데 이게 하나도 안 뜨는 상황을 말함)
                    status = "BRAKE-ON?" if brightening else "steady"
                    area_pct_str = f"/{area_ratio*100:+.0f}%" if area_ratio is not None else ""
                    # ★참고: cv2.putText는 한글(유니코드) 렌더링을 지원하지 않아 화면에 "????"로 깨져 나옴
                    #        → 화면 표시용 텍스트는 영어로, 터미널 로그(print)는 한글 그대로 유지
                    cv2.putText(frame, f"r{redness_ratio*100:+.0f}%{area_pct_str} {status}", (10, 55),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    # ===== ★STM32로 밝기 비율/이상여부 전송 (200ms 속도 제한) =====
                    if stm32_ser is not None and (now_t - last_b_send_time) >= STM32_SEND_INTERVAL_SEC:
                        try:
                            anomaly_flag = 1 if brightening else 0   # 밝아짐 감지되면 1, 아니면 0
                            # ★추가(테스트용): 니로가 엔진브레이크(회생제동)가 약해서 "감속인데 브레이크등 안 켜짐" 상황을
                            # 실차로 재현하기 어려움 → 카메라 판단을 강제로 "밝기변화 없음(0)"으로 고정해서
                            # STM32쪽 로직(레이더 감속판정+AND조건+WARN+부저)만 따로 검증. 실제 검증 끝나면 이 줄 지우거나 False로!
                            if FORCE_NO_BRIGHTENING_TEST:
                                anomaly_flag = 0
                            # ★변경: 두 번째 필드가 이제 "절대 밝기차"가 아니라 "redness 증가율(%)"임 (STM32는 그냥 표시만 하므로 파싱 영향 없음)
                            msg = f"B,{redness_ratio*100:.1f},{anomaly_flag}\r\n"   # "B,증가율(%),이상여부\r\n" 형식으로 조립
                            stm32_ser.write(msg.encode('utf-8'))                  # STM32로 전송
                            last_b_send_time = now_t                              # 마지막 전송 시각 갱신
                        except Exception as e:
                            print(f"[STM32] 전송 에러: {e}")
                cv2.putText(frame, f"redness={smoothed:.1f} area={smoothed_area:.0f} (blobs={blob_count})", (10, 75),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

            if not blob_found:
                cv2.putText(frame, "waiting for red light...", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                # ★추가(진짜 원인 발견): 빨간 덩어리가 아예 안 잡히면(blob_found=False) 위의 "B," 전송
                # 블록 자체가 통째로 안 도는 구조였음 → 미등까지 완전히 꺼놓고 주변에 빨간불도 없는
                # 환경(깨끗한 야간 테스트)에서는 "안 켜짐(0)"을 계속 보내는 게 아니라 아예 메시지가
                # 하나도 안 나갔을 수 있음 → STM32의 pi_anomaly_flag가 갱신 안 되고 예전 값에 멈춰있거나,
                # 한 번도 유효 데이터를 못 받았으면(pi_data_valid=0) WARN 판정 자체가 절대 안 걸림.
                # → 빨간 게 하나도 안 잡혀도 "안 켜짐"을 명시적으로 계속 STM32에 보내도록 여기서 처리.
                now_t3 = time.time()
                if stm32_ser is not None and (now_t3 - last_b_send_time) >= STM32_SEND_INTERVAL_SEC:
                    try:
                        stm32_ser.write(f"B,0.0,0\r\n".encode('utf-8'))   # 빨간 것 자체가 없으니 당연히 "밝아짐 아님"
                        last_b_send_time = now_t3
                    except Exception as e:
                        print(f"[STM32] 전송 에러: {e}")

            # ===== 지금 이 순간 검출 여부를 눈에 확 띄게 표시 (화면 테두리 색 + 큰 글씨) =====
            h_disp, w_disp = frame.shape[:2]
            border_color = (0, 200, 0) if blob_found else (0, 0, 220)   # 초록=검출됨, 빨강=미검출 (BGR 순서)
            cv2.rectangle(frame, (0, 0), (w_disp - 1, h_disp - 1), border_color, 6)  # 화면 테두리 굵게
            status_text = "RED LIGHT DETECTED" if blob_found else "NOT DETECTED"
            cv2.putText(frame, status_text, (10, h_disp - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, border_color, 2)

            # ===== 지금 이 순간의 카메라 검출 상태를 STM32로도 실시간 전송 (LCD에 표시하기 위함, 200ms 속도제한) =====
            now_t2 = time.time()
            if stm32_ser is not None and (now_t2 - last_c_send_time) >= STM32_SEND_INTERVAL_SEC:
                try:
                    stm32_ser.write(f"C,{1 if blob_found else 0}\r\n".encode('utf-8'))
                    last_c_send_time = now_t2
                except Exception as e:
                    print(f"[STM32] 카메라상태 전송 에러: {e}")

            # (baseline은 이제 위쪽 blob_found 블록 안에서 슬라이딩 윈도우 방식으로 계속 실시간 갱신됨)

            if frame_count % SAVE_EVERY_N_FRAMES == 0:
                cv2.imwrite("latest_frame.jpg", frame)
                radar_info = ""                                            # 로그에 레이더 값도 같이 찍어줌 (참고용)
                with serial_data_lock:
                    if latest_radar_distance is not None:
                        angle_str = f"/A{latest_radar_angle:.0f}" if latest_radar_angle is not None else ""
                        radar_info = f", radar=D{latest_radar_distance:.1f}/S{latest_radar_speed:.1f}{angle_str}"
                if last_redness is not None:
                    ratio_str = ""
                    if redness_baseline.value is not None:
                        r_pct = (last_redness - redness_baseline.value) / redness_baseline.value * 100 if redness_baseline.value > 1e-6 else 0
                        a_pct_str = ""
                        if area_baseline.value is not None and area_baseline.value > 1e-6:
                            a_pct = (area - area_baseline.value) / area_baseline.value * 100
                            a_pct_str = f", area{a_pct:+.0f}%"
                        ratio_str = f", redness{r_pct:+.0f}%{a_pct_str}"
                    print(f"frame={frame_count}, redness={last_redness:.1f}{ratio_str}{radar_info}, mode={radar_mode}")
                else:
                    print(f"frame={frame_count}, 빨간불빛 미검출{radar_info}, mode={radar_mode}")
    except KeyboardInterrupt:
        print("\n종료합니다.")
    cap.release()


if __name__ == "__main__":
    main()