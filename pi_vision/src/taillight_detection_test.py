"""
YOLO로 화면에서 "가장 크게(=가장 가깝게) 보이는 차"만 선택하고,
차량 박스를 상단 비율 기준 두 구간(하이마운트 브레이크등 구간 / 좌우 후미등 구간)으로 나눈 뒤,
각 구간 안에서만 빨간 덩어리를 찾아 밝기 변화를 감지하는 코드
(Pi 등 화면(모니터) 없는 환경용 — cv2.imshow 대신 파일 저장 + 터미널 로그 방식)
(속도 최적화: YOLO는 N프레임마다 한 번만 실행, 나머지는 마지막 박스 위치 재사용)

===== ★STM32 통신 프로토콜 추가됨 =====
- STM32가 200ms마다 USART2로 보내는 "R,거리,상대속도\r\n"을 백그라운드 스레드로 계속 읽어서
  latest_radar_distance / latest_radar_speed 전역변수에 저장해둠 (지금 당장 이 스크립트에서 안 써도,
  나중에 "차가 감속중인데 카메라 프레임엔 아직 없다" 같은 조합판단 하려면 필요해서 미리 받아둠)
- 베이스라인이 설정된 이후, 밝기 델타를 계산할 때마다 "B,밝기델타,이상여부\r\n"을 STM32로 전송함
  (이상여부: delta가 임계값보다 크면 1=밝아짐 감지됨(정상적으로 브레이크등 켜짐), 아니면 0=변화없음)

사용법: python src/taillight_detection_test.py
   Ctrl+C: 종료
"""
import cv2
import numpy as np
import time
from collections import deque
from ultralytics import YOLO
import serial                                        # ★추가: STM32와 시리얼(UART) 통신을 하기 위한 라이브러리
import threading                                     # ★추가: 시리얼 읽기를 메인 영상처리 루프와 분리된 스레드로 돌리기 위함

SMOOTHING_FRAMES = 5
DELTA_THRESHOLD = 15.0
MIN_RED_PIXELS = 15
PADDING = 5
MIN_CAR_AREA_RATIO = 0.02
HIGHMOUNT_TOP = 0.00
HIGHMOUNT_BOTTOM = 0.15
TAILLIGHT_TOP = 0.35
TAILLIGHT_BOTTOM = 0.55
SAVE_EVERY_N_FRAMES = 5
BASELINE_AT_FRAME = 30
YOLO_INTERVAL = 8
YOLO_IMGSZ = 320

# ===== ★STM32 시리얼 통신 설정 =====
STM32_PORT = '/dev/serial0'                         # 라즈베리파이의 GPIO UART 포트 (STM32 USART2와 연결된 핀)
                                                     # 안 열리면 '/dev/ttyAMA0'으로도 시도해볼 것 (Pi 설정에 따라 이름 다를 수 있음)
STM32_BAUD = 115200                                 # STM32의 huart2 baudrate와 반드시 동일해야 함 (main.c MX_USART2_UART_Init 참고)

latest_radar_distance = None                        # ★STM32가 보내준 최신 레이더 거리값(m) 저장, 아직 안 받았으면 None
latest_radar_speed = None                           # ★STM32가 보내준 최신 상대속도값(km/h) 저장
serial_data_lock = threading.Lock()                 # ★백그라운드 스레드랑 메인루프가 동시에 위 두 변수를 건드리지 않도록 보호하는 락

print("[YOLO] 모델 로딩 중... (최초 실행시 다운로드 때문에 몇 분 걸릴 수 있음)")
model = YOLO('yolov8n.pt')
print("[YOLO] 모델 로딩 완료")


def stm32_serial_reader(ser):                       # ★STM32에서 오는 "R,거리,속도" 줄을 계속 읽어서 전역변수에 저장하는 함수 (별도 스레드에서 실행됨)
    global latest_radar_distance, latest_radar_speed   # 함수 안에서 전역변수를 수정하겠다고 선언
    while True:                                          # 프로그램이 끝날 때까지 무한 반복
        try:                                               # 시리얼 읽기 중 에러가 나도 스레드가 죽지 않게 감싸줌
            raw_line = ser.readline()                        # 한 줄(개행까지) 읽기 시도, timeout 설정에 따라 없으면 빈 값 반환하고 넘어감
            if not raw_line:                                  # 아무것도 못 읽었으면(타임아웃)
                continue                                        # 그냥 다음 반복으로 (에러 아님, 정상적인 대기)
            line = raw_line.decode('utf-8', errors='ignore').strip()  # 바이트를 문자열로 변환, 앞뒤 공백/개행 제거
            if line.startswith('R,'):                          # STM32가 보내는 레이더 데이터 형식("R,거리,속도")이면
                parts = line.split(',')                          # 쉼표 기준으로 쪼갬 → ["R", "거리", "속도"]
                if len(parts) == 3:                               # 정확히 3개로 쪼개졌으면(형식이 맞으면)
                    try:                                            # 숫자 변환 실패(깨진 데이터 등) 대비
                        dist = float(parts[1])                        # 두 번째 조각을 거리값(실수)으로 변환
                        spd = float(parts[2])                         # 세 번째 조각을 속도값(실수)으로 변환
                        with serial_data_lock:                        # 락을 잡은 상태에서만 전역변수 수정(동시접근 방지)
                            latest_radar_distance = dist                # 최신 거리값 갱신
                            latest_radar_speed = spd                    # 최신 속도값 갱신
                    except ValueError:                               # 숫자로 변환이 안 되면(깨진 줄)
                        pass                                            # 그냥 무시하고 다음 줄 기다림
        except Exception as e:                                    # 시리얼 포트 자체에 문제가 생기면(케이블 빠짐 등)
            print(f"[STM32] 시리얼 읽기 에러: {e}")
            time.sleep(1)                                            # 1초 쉬고 다시 시도


def find_red_blobs(frame, region, max_blobs):
    x1, y1, x2, y2 = region
    x1, y1 = max(0, int(x1)), max(0, int(y1))
    x2, y2 = int(x2), int(y2)
    crop = frame[y1:y2, x1:x2]
    if crop.size == 0:
        return []
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
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


def get_search_regions(car_box):
    x1, y1, x2, y2 = car_box
    box_height = y2 - y1
    highmount_region = (
        x1, y1 + box_height * HIGHMOUNT_TOP,
        x2, y1 + box_height * HIGHMOUNT_BOTTOM
    )
    taillight_region = (
        x1, y1 + box_height * TAILLIGHT_TOP,
        x2, y1 + box_height * TAILLIGHT_BOTTOM
    )
    return highmount_region, taillight_region


def get_red_brightness(frame, roi):
    x1, y1, x2, y2 = roi
    x1, y1 = max(0, x1), max(0, y1)
    crop = frame[y1:y2, x1:x2]
    if crop.size == 0:
        return None
    b, g, r = cv2.split(crop)
    return float(np.mean(r))


def process_taillights(frame, car_box):
    x1, y1, x2, y2 = car_box
    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 1)
    highmount_region, taillight_region = get_search_regions((x1, y1, x2, y2))
    hx1, hy1, hx2, hy2 = [int(v) for v in highmount_region]
    cv2.rectangle(frame, (hx1, hy1), (hx2, hy2), (0, 200, 200), 1)
    tx1, ty1, tx2, ty2 = [int(v) for v in taillight_region]
    cv2.rectangle(frame, (tx1, ty1), (tx2, ty2), (200, 200, 0), 1)
    highmount_blobs = find_red_blobs(frame, highmount_region, max_blobs=1)
    taillight_blobs = find_red_blobs(frame, taillight_region, max_blobs=2)
    all_blobs = highmount_blobs + taillight_blobs
    brightness_values = []
    for (bx1, by1, bx2, by2) in all_blobs:
        cv2.rectangle(frame, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
        b = get_red_brightness(frame, (bx1, by1, bx2, by2))
        if b is not None:
            brightness_values.append(b)
    if brightness_values:
        return float(np.mean(brightness_values)), len(all_blobs)
    return None, 0


def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

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

    history = deque(maxlen=SMOOTHING_FRAMES)
    baseline = None
    last_brightness = None
    frame_count = 0
    last_car_box = None
    last_yolo_time = 0.0
    print("Ctrl+C로 종료 | 5프레임마다 latest_frame.jpg 저장 | 30프레임째 자동 베이스라인 설정")
    print(f"YOLO는 {YOLO_INTERVAL}프레임마다 1회만 실행 (속도 최적화)")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("프레임을 읽을 수 없습니다.")
                break
            frame = cv2.resize(frame, (640, 480))
            frame_count += 1
            run_yolo_this_frame = (frame_count % YOLO_INTERVAL == 1) or (last_car_box is None)
            car_found = False
            if run_yolo_this_frame:
                t0 = time.time()
                results = model(frame, classes=[2], verbose=False, imgsz=YOLO_IMGSZ)
                last_yolo_time = time.time() - t0
                boxes = results[0].boxes
                if len(boxes) > 0:
                    areas = []
                    for box in boxes:
                        bx1, by1, bx2, by2 = box.xyxy[0].tolist()
                        areas.append((bx2 - bx1) * (by2 - by1))
                    biggest_idx = int(np.argmax(areas))
                    best_box = boxes[biggest_idx]
                    x1, y1, x2, y2 = best_box.xyxy[0].tolist()
                    box_area = (x2 - x1) * (y2 - y1)
                    frame_area = frame.shape[0] * frame.shape[1]
                    if box_area >= frame_area * MIN_CAR_AREA_RATIO:
                        last_car_box = (x1, y1, x2, y2)
                    else:
                        last_car_box = None
                else:
                    last_car_box = None
            if last_car_box is not None:
                brightness, blob_count = process_taillights(frame, last_car_box)
                if brightness is not None:
                    last_brightness = brightness
                    history.append(brightness)
                    smoothed = float(np.mean(history))
                    x1, y1, x2, y2 = last_car_box
                    delta = None
                    if baseline is not None:
                        delta = smoothed - baseline
                        status = "이상(밝아짐)" if delta > DELTA_THRESHOLD else "정상"
                        cv2.putText(frame, f"delta={delta:.1f} {status}", (int(x1), max(0, int(y1) - 10)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        # ===== ★STM32로 밝기 델타/이상여부 전송 =====
                        if stm32_ser is not None:                            # 시리얼 연결이 살아있으면
                            try:                                             # 전송 중 케이블 문제 등으로 에러 나도 영상처리는 안 멈추게
                                anomaly_flag = 1 if delta > DELTA_THRESHOLD else 0   # 밝아짐 감지되면 1, 아니면 0
                                msg = f"B,{delta:.1f},{anomaly_flag}\r\n"            # "B,밝기델타,이상여부\r\n" 형식으로 조립
                                stm32_ser.write(msg.encode('utf-8'))                  # STM32로 전송
                            except Exception as e:
                                print(f"[STM32] 전송 에러: {e}")
                    cv2.putText(frame, f"R={smoothed:.1f} (blobs={blob_count})", (int(x1), int(y2) + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                car_found = True
            if not car_found:
                cv2.putText(frame, "차량 인식 대기 중...", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if frame_count == BASELINE_AT_FRAME and last_brightness is not None:
                baseline = last_brightness
                print(f"[자동] 베이스라인 설정: {baseline:.1f}")
            if frame_count % SAVE_EVERY_N_FRAMES == 0:
                cv2.imwrite("latest_frame.jpg", frame)
                yolo_info = f", yolo={last_yolo_time*1000:.0f}ms" if run_yolo_this_frame else ""
                radar_info = ""                                            # ★추가: 로그에 레이더 값도 같이 찍어줌 (참고용)
                with serial_data_lock:
                    if latest_radar_distance is not None:
                        radar_info = f", radar=D{latest_radar_distance:.1f}/S{latest_radar_speed:.1f}"
                if last_brightness is not None:
                    delta_str = ""
                    if baseline is not None:
                        d = last_brightness - baseline
                        delta_str = f", delta={d:.1f}"
                    print(f"frame={frame_count}, brightness={last_brightness:.1f}{delta_str}{yolo_info}{radar_info}")
                else:
                    print(f"frame={frame_count}, 차량 미검출{yolo_info}{radar_info}")
    except KeyboardInterrupt:
        print("\n종료합니다.")
    cap.release()


if __name__ == "__main__":
    main()
