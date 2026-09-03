"""
측정 방식:
- 밝기값은 단순 R채널 평균이 아니라 "redness = R - (G+B)/2" (빨간정도)로 계산
  → 헤드라이트나 전체적인 노출변화(흰 빛)와 실제 빨간 브레이크등을 더 잘 구분함
- ON/OFF 판정은 "최근 25초간 관찰값 중 최솟값"을 baseline으로 실시간 갱신하며 비교함
  (프레임 간 순간 변화량만 보면 브레이크등이 몇 초씩 켜져있을 때 "꺼졌다"고 착각하는 문제가 있어서
   장기 기준값 방식을 씀 — blob 위치를 찾는 단계는 HSV 빨간색 검출 그대로 사용)


동작 방식:
- 카메라 스캔 범위(ROI)는 항상 고정 넓은 영역(ROI_X1~X2, Y1~Y2) 그대로 유지 — 레이더 상태와 무관하게
  놓치는 일이 없도록 하는 안전판
- 레이더가 신뢰 가능하면: ROI 안에서 찾은 빨간 블롭 중, 레이더가 가리키는 위치 근처(RADAR_GATE_HALF_WIDTH_RATIO)에
  있는 것만 골라서 밝기 계산에 씀 (손/가로등/옆차선 후미등처럼 무관한 블롭은 자동 제외)
- 레이더 위치 근처에 빨간 블롭이 하나도 없으면(레이더-카메라 불일치) → 그 프레임은 레이더를 무시하고
  ROI 안의 모든 블롭을 그대로 씀 → 레이더가 틀려도 신호를 완전히 놓치지는 않음
- 레이더 신호가 없거나(거리=-1) 오래됐거나 샘플이 부족하면 게이팅을 안 하고 항상 전체 블롭 사용

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
import math                                          # 레이더-카메라 위치 차이를 보정하는 삼각함수 계산용
import statistics                                    # 레이더 각도의 median(중앙값) 계산용 — 한 프레임 튀는 값에 안 끌려가게
from collections import deque
import serial                                        # STM32와 시리얼(UART) 통신을 하기 위한 라이브러리
import threading                                     # 시리얼 읽기를 메인 영상처리 루프와 분리된 스레드로 돌리기 위함

SMOOTHING_FRAMES = 5
# baseline 대비 "비율(%)"
REDNESS_RATIO_THRESHOLD = 0.35   # baseline 대비 redness(색 진하기)가 이 비율(35%) 이상 늘면 "밝아짐" 후보 (참고용)
AREA_RATIO_THRESHOLD = 15.00     # baseline 대비 면적(빛이 번져 보이는 크기)이 이 비율(1500%) 이상 늘면 "밝아짐" 후보 (참고용)

FORCE_NO_BRIGHTENING_TEST = False

ABS_AREA_ON_THRESHOLD = 10000   # Area 절대값 임계값 설정
MIN_RED_PIXELS = 8              # 멀리 있는 작은 빛도 잡기 위해 낮춘 값
PADDING = 5
SAVE_EVERY_N_FRAMES = 2   # live_view_server 화면 갱신 체감 속도를 위해 2프레임마다 저장
# 시작할 때 잡는 baseline은 실제 주행과 안 맞음 — 앞차가 계속 바뀌므로
# → 최근 BASELINE_WINDOW_SEC초 동안 관찰한 값 중 최솟값을 baseline으로 실시간 갱신함
BASELINE_WINDOW_SEC = 25.0     # 최근 몇 초 데이터로 baseline(최솟값)을 계산할지
BASELINE_WARMUP_SEC = 3.0      # 최소 이 정도는 데이터가 쌓여야 baseline을 신뢰

# ===== 관심영역(ROI) 설정 — 화면 전체가 아니라 이 구역 안에서만 빨간 덩어리를 찾음 =====
# 카메라가 차량 정면 고정, 내 차선 앞차는 대략 이 범위 안에 있을 거라는 전제
# 카메라 장착 각도에 맞춰 이 4개 비율만 조정
ROI_X1_RATIO = 0.20   # 왼쪽 경계 (화면 가로의 20% 지점부터)
ROI_X2_RATIO = 0.80   # 오른쪽 경계 (화면 가로의 80% 지점까지)
ROI_Y1_RATIO = 0.30   # 위쪽 경계 (하늘/윗부분 제외)
ROI_Y2_RATIO = 0.90   # 아래쪽 경계 (내 차 보닛/대시보드 제외)
MAX_BLOBS_IN_ROI = 3    # ROI 안에서 최대 몇 개의 빨간 덩어리까지 고려할지 (그중 밝기 평균 큰 것들 위주로 씀)

# ===== 레이더는 ROI를 좁히는 게 아니라, 찾은 블롭 중 신뢰성이 높은 블롭에 관한 힌트 =====
CAMERA_HFOV_DEG = 60.0             # 웹캠의 대략적인 수평 화각(도) — 실측값 아니면 일반 웹캠 평균치로 가정, 나중에 실측해서 조정 필요
RADAR_GATE_HALF_WIDTH_RATIO = 0.20   # 레이더가 가리키는 위치 기준, 화면 가로폭의 ±몇 % 안에 있는 블롭까지 볼지
RADAR_STALE_TIMEOUT_SEC = 1.0      # 이 시간 이상 레이더 갱신이 없으면 "레이더 없음"으로 보고 게이팅 안 함
RADAR_ANGLE_HISTORY_LEN = 5        # 각도 median 계산에 쓸 최근 샘플 개수
RADAR_MIN_SAMPLES = 3              # 이 개수 이상 연속 유효 샘플이 쌓여야 레이더를 신뢰
RADAR_ANGLE_SIGN = 1.0              # 실측 전 임시값: 레이더 각도의 +/-가 실제 왼쪽/오른쪽 중 어느 쪽인지 아직 미확인
                                    

# ===== 레이더-카메라 실측 장착 위치 차이 보정용 (2026-08-22 실측값) =====
# 레이더 = 앞 그릴 중앙, 카메라 = 실내 룸미러(정중앙에서 조수석 방향 30cm),
# 카메라가 레이더보다 차량 뒤쪽으로 약 1.8~1.9m 떨어져 있음 (앞뒤 거리 차이가 좌우 매핑에 미치는 영향 보정)
RADAR_TO_CAM_FORWARD_OFFSET_M = 1.85   # 카메라가 레이더보다 뒤쪽에 있는 거리(m) — 1.8~1.9m의 평균값
RADAR_TO_CAM_LATERAL_OFFSET_M = 0.30   # 카메라가 레이더 중심선보다 조수석 방향으로 치우친 거리(m)

# 캡처 해상도를 미리 높여서 요청함 (웹캠이 지원하면). 원본 해상도가 높아야 작은 빛도 디테일이 살아있음
CAPTURE_WIDTH = 1280
CAPTURE_HEIGHT = 720
WORK_WIDTH = 640    # 실제 처리(빨간블롭 찾기 등)에 쓸 작업용 해상도 (너무 크면 Pi CPU 부담)
WORK_HEIGHT = 480

# ===== 디지털 줌 — 원본 캡처 프레임의 중앙 부분만 잘라서 WORK 해상도로 확대함 =====
# 멀리 있는 브레이크등이 화면에서 너무 작게 잡혀서 블롭 검출이 어려운 문제 완화용
# 반드시 "자르기(crop) 먼저 → 확대(resize) 나중" 순서로 해야 함 (반대면 이미 저해상도로 줄어든 걸 잘라서 확대 효과 없음)
DIGITAL_ZOOM_CROP_RATIO = 0.2857   # 약 3.5배 줌

# ===== STM32 시리얼 통신 설정 =====
STM32_PORT = '/dev/serial0'                         # 라즈베리파이의 GPIO UART 포트 (STM32 USART2와 연결된 핀)
STM32_BAUD = 115200                                 # STM32의 huart2 baudrate와 반드시 동일해야 함 (main.c MX_USART2_UART_Init 참고)
STM32_SEND_INTERVAL_SEC = 0.2   # B,/C, 메시지를 STM32로 보내는 최소 간격 (STM32 쪽 200ms 루프주기와 맞춤, 수신버퍼 보호용)

latest_radar_distance = None                        # STM32가 보내준 최신 레이더 거리값(m) 저장, 아직 안 받았으면 None
latest_radar_speed = None                           # STM32가 보내준 최신 상대속도값(km/h) 저장
latest_radar_angle = None                           # STM32가 보내준 최신 레이더 각도값(도, 정면=0) 저장 (화면 표시/로그용, 단일값이라 노이즈 있음)
latest_radar_update_time = 0.0                      # 마지막으로 "유효한"(distance>=0) 레이더 데이터를 받은 시각
radar_angle_history = deque(maxlen=RADAR_ANGLE_HISTORY_LEN)  # 최근 유효 각도값들 — median 계산해서 노이즈 억제
serial_data_lock = threading.Lock()                 # 백그라운드 스레드랑 메인루프가 동시에 위 변수들을 건드리지 않도록 보호하는 락


class SlidingMinBaseline:
    # redness/area 공통으로 쓰는 "최근 N초 관찰값 중 최솟값" baseline 로직 (두 개를 각각 인스턴스로 만들어서 씀)
    def __init__(self, window_sec, warmup_sec, percentile=0):
        self.window_sec = window_sec
        self.warmup_sec = warmup_sec
        self.percentile = percentile   
        self.buffer = deque()          
        self.first_sample_time = None  
        self.value = None              # 확정된 baseline

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
            if not raw_line:                                  
                continue                                        
            line = raw_line.decode('utf-8', errors='ignore').strip()  # 바이트를 문자열로 변환, 앞뒤 공백 제거
            if line.startswith('R,'):                          # STM32가 보내는 레이더 데이터 형식("R,거리,속도,각도")이면
                parts = line.split(',')                          # 쉼표 기준으로 쪼갬 → ["R", "거리", "속도", "각도"]
                if len(parts) >= 3:                               
                    try:                                            # 숫자 변환 실패(깨진 데이터 등) 대비
                        dist = float(parts[1])                        # 두 번째 조각을 거리값(실수)으로 변환
                        spd = float(parts[2])                         # 세 번째 조각을 속도값(실수)으로 변환
                        ang = float(parts[3]) if len(parts) >= 4 else None  # 네 번째 조각(각도)이 있으면 변환, 없으면 None
                        with serial_data_lock:                        
                            if (time.time() - latest_radar_update_time) > RADAR_STALE_TIMEOUT_SEC:
                                radar_angle_history.clear()
                            latest_radar_distance = dist                # 최신 거리값 갱신
                            latest_radar_speed = spd                    # 최신 속도값 갱신
                            latest_radar_angle = ang                    # 최신 각도값 갱신 (표시용)
                            if dist >= 0 and ang is not None:             # 진짜 유효한 타겟일 때만(거리=-1은 "타겟 없음") 이력에 반영
                                radar_angle_history.append(ang)
                                latest_radar_update_time = time.time()     # 갱신 시각 기록 (참고용)
                    except ValueError:                               # 숫자로 변환이 안 되면(깨진 줄)
                        pass                                            # 그냥 무시하고 다음 줄 기다림
        except Exception as e:                                    # 시리얼 포트 자체에 문제가 생기면(케이블 빠짐 등)
            print(f"[STM32] 시리얼 읽기 에러: {e}")
            time.sleep(1)                                            # 1초 쉬고 다시 시도


def find_red_blobs(frame, region, max_blobs):
    # HSV 색공간에서 빨간색에 해당하는 픽셀들을 찾아서, 덩어리(블롭)로 묶어 위치를 찾는 함수
    x1, y1, x2, y2 = region
    x1, y1 = max(0, int(x1)), max(0, int(y1))
    x2, y2 = int(x2), int(y2)
    crop = frame[y1:y2, x1:x2]
    if crop.size == 0:
        return []
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    # 채도(S) 하한을 130으로 잡아서 손/피부색 같은 옅은 빨강/핑크는 걸러내고 진한 빨강만 남김
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
    # 원본 프레임의 중앙 설정한 비율만 잘라낸 뒤, WORK 해상도로 다시 키움
    h, w = frame.shape[:2]
    crop_w = int(w * DIGITAL_ZOOM_CROP_RATIO)
    crop_h = int(h * DIGITAL_ZOOM_CROP_RATIO)
    x1 = (w - crop_w) // 2
    y1 = (h - crop_h) // 2
    cropped = frame[y1:y1 + crop_h, x1:x1 + crop_w]
    return cv2.resize(cropped, (WORK_WIDTH, WORK_HEIGHT))


def get_redness(frame, roi):
    # 단순 R채널 평균 대신 "redness = R - (G+B)/2"를 씀
    # → 전체 화면이 밝아지는 것(헤드라이트, 노출변화)과 실제 빨간 브레이크등을 더 잘 구분
    x1, y1, x2, y2 = roi
    x1, y1 = max(0, x1), max(0, y1)
    crop = frame[y1:y2, x1:x2]
    if crop.size == 0:
        return None
    b, g, r = cv2.split(crop.astype(np.int16))   # int16으로 변환 overflow 방지
    redness = r - (g + b) / 2.0
    redness = np.clip(redness, 0, None)          # 빨간색이 아닌 부분 0으로 클리핑
    return float(np.mean(redness))


def get_radar_target_x(w):
    # 레이더가 지금 신뢰할 만한지 판단하고, 신뢰할 만하면 각도를 화면 픽셀 x좌표로 변환해서 반환
    with serial_data_lock:
        dist = latest_radar_distance
        upd_time = latest_radar_update_time
        hist = list(radar_angle_history)  

    if dist is None or dist < 0:                                # 애초에 유효한 타겟이 없으면
        return None
    if (time.time() - upd_time) > RADAR_STALE_TIMEOUT_SEC:        # 마지막 유효 갱신이 너무 오래 전이면(연결 끊김 등)
        return None
    if len(hist) < RADAR_MIN_SAMPLES:                            # 아직 신뢰할 만큼 샘플이 안 쌓였으면(막 재연결된 직후 등)
        return None

    smoothed_angle = statistics.median(hist)                     # 최근 N개 중 중앙값 — 한 프레임 튀는 값에 안 끌려감

    # ===== 레이더-카메라 장착 위치 차이 보정 =====
    ang_rad = math.radians(smoothed_angle * RADAR_ANGLE_SIGN)       # 라디안
    target_x_from_radar = dist * math.cos(ang_rad)                   # 레이더 기준, 타겟까지의 전방 거리
    target_y_from_radar = dist * math.sin(ang_rad)                   # 레이더 기준, 타겟까지의 좌우(+=조수석쪽) 거리
    # 카메라는 레이더보다 뒤쪽에 있으므로(전방좌표 -offset), 카메라 기준 전방거리 = 타겟전방거리 + 레이더-카메라 간격
    rel_x = target_x_from_radar + RADAR_TO_CAM_FORWARD_OFFSET_M
    # 카메라는 레이더 중심선보다 조수석쪽으로 치우쳐 있으므로, 그만큼 빼줌
    rel_y = target_y_from_radar - RADAR_TO_CAM_LATERAL_OFFSET_M
    camera_angle_deg = math.degrees(math.atan2(rel_y, rel_x))        # 카메라 입장에서 본 타겟의 각도(도)

    half_fov = CAMERA_HFOV_DEG / 2.0
    offset_ratio = max(-1.0, min(1.0, camera_angle_deg / half_fov))  # -1~1로 화각 밖 각도 방지
    return (w / 2.0) + offset_ratio * (w / 2.0)                   # 화면 중심 기준으로 각도만큼 이동한 픽셀 x좌표


def scan_roi_for_brake_light(frame):
    # 스캔 범위(ROI)는 항상 고정 넓은 영역 그대로 유지
    # 레이더가 신뢰 가능하면, 찾은 블롭들 중 레이더 위치 근처에 있는 것만 골라서 씀
    # 레이더 위치 근처에 블롭이 하나도 없으면(레이더-카메라 불일치) -> 안전하게 전체 블롭으로 폴백
    h, w = frame.shape[:2]
    rx1 = int(w * ROI_X1_RATIO)
    ry1 = int(h * ROI_Y1_RATIO)
    rx2 = int(w * ROI_X2_RATIO)
    ry2 = int(h * ROI_Y2_RATIO)
    cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), (200, 200, 0), 1)  # 고정 ROI 테두리 (항상 이 넓은 범위를 실제로 스캔함)

    blobs = find_red_blobs(frame, (rx1, ry1, rx2, ry2), max_blobs=MAX_BLOBS_IN_ROI)
    if not blobs:
        return None, 0, "NO_BLOB", 0.0

    radar_x = get_radar_target_x(w)          # 레이더 신뢰 불가 -> 차단
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
    total_area = 0.0   # 사용된 블롭들의 면적 합 — redness와 별개 신호
    for (bx1, by1, bx2, by2) in used_blobs:
        box_color = (0, 255, 0) if mode == "RADAR_GATED" else (0, 200, 200)  # 레이더로 골라진 블롭=초록, 아니면 청록으로 구분
        cv2.rectangle(frame, (bx1, by1), (bx2, by2), box_color, 2)
        rv = get_redness(frame, (bx1, by1, bx2, by2))
        if rv is not None:
            redness_values.append(rv)
        total_area += max(0, bx2 - bx1) * max(0, by2 - by1)   # 박스 면적

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

    # ===== STM32 시리얼 포트 연결 =====
    try:                                                  # 시리얼 포트 연결 시도, 실패해도 영상처리 자체는 계속 되게(레이더 없이) 처리
        stm32_ser = serial.Serial(STM32_PORT, STM32_BAUD, timeout=1)  
        reader_thread = threading.Thread(target=stm32_serial_reader, args=(stm32_ser,), daemon=True)
        reader_thread.start()                              # 백그라운드 스레드 시작 (이때부터 R, 데이터 계속 수신됨)
        print(f"[STM32] 시리얼 연결 성공: {STM32_PORT} @ {STM32_BAUD}bps")
    except Exception as e:                                # 포트가 없거나 권한 문제 등으로 연결 실패시
        stm32_ser = None                                    # STM32 연결 안 된 상태로 표시
        print(f"[STM32] 시리얼 연결 실패 (레이더 데이터 없이 영상처리만 진행): {e}")

    history = deque(maxlen=SMOOTHING_FRAMES)        # redness 스무딩용
    area_history = deque(maxlen=SMOOTHING_FRAMES)    # area 스무딩용 (redness와 같은 방식)
    redness_baseline = SlidingMinBaseline(BASELINE_WINDOW_SEC, BASELINE_WARMUP_SEC)             # redness는 min 유지
    area_baseline = SlidingMinBaseline(BASELINE_WINDOW_SEC, BASELINE_WARMUP_SEC, percentile=20)  # area는 하위 20%로 완화
    last_redness = None
    frame_count = 0
    last_b_send_time = 0.0
    last_c_send_time = 0.0
    baseline_announced = False   # baseline이 처음 잡히는 순간 로그로 한 번 알려주기 위한 플래그
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
            frame = apply_digital_zoom(frame)   # 중앙 크롭+확대로 디지털 줌 적용
            frame_count += 1

            redness, blob_count, radar_mode, area = scan_roi_for_brake_light(frame)
            blob_found = redness is not None

            if blob_found:
                last_redness = redness
                history.append(redness)
                area_history.append(area)
                smoothed = float(np.mean(history))
                smoothed_area = float(np.mean(area_history))
                now_t = time.time()

                # redness/area 각각 슬라이딩 윈도우 baseline 갱신
                r_base = redness_baseline.update(now_t, redness)
                a_base = area_baseline.update(now_t, area)
                if r_base is not None and not baseline_announced:
                    print(f"[자동] 베이스라인 첫 설정(frame={frame_count}): redness={r_base:.1f} (이후 계속 실시간 갱신됨)")
                    baseline_announced = True

                # 판정은 area 절대값 하나로 함 (redness_ratio/area_ratio는 표시/전송용으로만 계산)
                redness_ratio = ((smoothed - r_base) / r_base) if (r_base is not None and r_base > 1e-6) else None
                area_ratio = ((smoothed_area - a_base) / a_base) if (a_base is not None and a_base > 1e-6) else None
                abs_area_hit = (area is not None and area >= ABS_AREA_ON_THRESHOLD)
                brightening = abs_area_hit

                if redness_ratio is not None:
                    # 브레이크등이 정상적으로 켜졌다는 뜻 
                    status = "BRAKE-ON?" if brightening else "steady"
                    area_pct_str = f"/{area_ratio*100:+.0f}%" if area_ratio is not None else ""
                    cv2.putText(frame, f"r{redness_ratio*100:+.0f}%{area_pct_str} {status}", (10, 55),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    # ===== STM32로 밝기 비율/이상여부 전송 (200ms 속도 제한) =====
                    if stm32_ser is not None and (now_t - last_b_send_time) >= STM32_SEND_INTERVAL_SEC:
                        try:
                            anomaly_flag = 1 if brightening else 0   # 밝아짐 감지되면 1, 아니면 0
                            if FORCE_NO_BRIGHTENING_TEST:
                                anomaly_flag = 0
                            # 두 번째 필드는 "redness 증가율(%)" (STM32는 표시만 하므로 파싱엔 영향 없음)
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
                # 빨간 덩어리가 아예 안 잡히면 위 블록의 "B," 전송이 안 도는 구조라, STM32의
                # pi_anomaly_flag가 예전 값에 멈추거나 pi_data_valid=0 -> WARN 판정이 안 걸릴 수 있음
                # → 여기서 "안 켜짐"을 명시적으로 STM32에 보내도록 처리
                now_t3 = time.time()
                if stm32_ser is not None and (now_t3 - last_b_send_time) >= STM32_SEND_INTERVAL_SEC:
                    try:
                        stm32_ser.write(f"B,0.0,0\r\n".encode('utf-8'))   # 빨간 것 없으니 당연히 "밝아짐 X"
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

            # baseline은  위쪽 blob_found 블록 안에서 계속 실시간 갱신

            if frame_count % SAVE_EVERY_N_FRAMES == 0:
                cv2.imwrite("latest_frame.jpg", frame)
                radar_info = ""                                           
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