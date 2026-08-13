"""
YOLO로 차량을 찾고, 차량 박스를 상단 비율 기준으로 두 구간(하이마운트 브레이크등 구간 / 좌우 후미등 구간)으로
나눈 뒤, 각 구간 안에서만 빨간 덩어리를 찾아 밝기 변화를 감지하는 테스트 코드
사용법: python src/taillight_detection_test.py
   b: 베이스라인(현재 밝기) 저장
   q: 종료

===== ★STM32 통신 프로토콜 추가됨 =====
- STM32가 200ms마다 USART2로 보내는 "R,거리,상대속도\r\n"을 백그라운드 스레드로 계속 읽어서
  latest_radar_distance / latest_radar_speed 전역변수에 저장해둠
- 베이스라인이 설정된 이후, 밝기 델타를 계산할 때마다 "B,밝기델타,이상여부\r\n"을 STM32로 전송함
  (이상여부: delta가 임계값보다 크면 1=밝아짐 감지됨, 아니면 0=변화없음)
"""
import cv2                                          # OpenCV, 카메라 영상 처리용 라이브러리를 cv2라는 이름으로 불러옴
import numpy as np                                  # 숫자 배열 계산을 위한 numpy 라이브러리를 np라는 이름으로 불러옴
from collections import deque                        # 최근 N개 값만 유지하는 큐 자료구조를 쓰기 위해 불러옴
from ultralytics import YOLO                         # YOLO 모델을 쓰기 위한 라이브러리에서 YOLO 클래스를 불러옴
import serial                                        # ★추가: STM32와 시리얼(UART) 통신을 하기 위한 라이브러리
import threading                                     # ★추가: 시리얼 읽기를 메인 영상처리 루프와 분리된 스레드로 돌리기 위함
import time                                          # ★추가: 에러 발생시 잠깐 대기(sleep)하는 용도

SMOOTHING_FRAMES = 5                                # 밝기값 노이즈를 줄이기 위해 평균낼 프레임 개수
DELTA_THRESHOLD = 15.0                              # 이 값보다 밝기 변화가 크면 "이상(밝아짐)"으로 판정하는 기준치
MIN_RED_PIXELS = 15                                 # 이 개수(면적)보다 작은 빨간 덩어리는 노이즈로 판단해 무시
PADDING = 5                                         # 빨간 영역 찾은 뒤 여유로 둘 픽셀 수
MIN_CAR_AREA_RATIO = 0.02                           # 차량 박스가 전체 화면의 이 비율보다 작으면 신뢰 안 하고 무시 (2%)
HIGHMOUNT_TOP = 0.00                                # 하이마운트(ㅡ자) 브레이크등 후보 구간: 차량 박스 상단 0%부터
HIGHMOUNT_BOTTOM = 0.15                             # ~15%까지
TAILLIGHT_TOP = 0.35                                # 좌우 후미등 후보 구간: 차량 박스 상단 35%부터
TAILLIGHT_BOTTOM = 0.55                             # ~55%까지

# ===== ★STM32 시리얼 통신 설정 =====
STM32_PORT = '/dev/serial0'                         # 라즈베리파이의 GPIO UART 포트 (STM32 USART2와 연결된 핀)
                                                     # 안 열리면 '/dev/ttyAMA0'으로도 시도해볼 것 (Pi 설정에 따라 이름 다를 수 있음)
STM32_BAUD = 115200                                 # STM32의 huart2 baudrate와 반드시 동일해야 함 (main.c MX_USART2_UART_Init 참고)

latest_radar_distance = None                        # ★STM32가 보내준 최신 레이더 거리값(m) 저장, 아직 안 받았으면 None
latest_radar_speed = None                           # ★STM32가 보내준 최신 상대속도값(km/h) 저장
serial_data_lock = threading.Lock()                 # ★백그라운드 스레드랑 메인루프가 동시에 위 두 변수를 건드리지 않도록 보호하는 락


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
            print(f"[STM32 시리얼 읽기 에러] {e}")                    # 에러 내용 출력
            time.sleep(0.5)                                         # 0.5초 쉬고 재시도 (에러 반복으로 CPU 낭비하는 것 방지)


def find_red_blobs(frame, region, max_blobs):        # 지정된 영역(region) 안에서 빨간 덩어리를 최대 max_blobs개 찾는 함수
    x1, y1, x2, y2 = region                           # 영역 좌표를 각각의 변수로 분리
    x1, y1 = max(0, int(x1)), max(0, int(y1))          # 좌표가 화면 밖(음수)으로 안 나가도록 보정
    x2, y2 = int(x2), int(y2)                          # 나머지 좌표도 정수로 변환
    crop = frame[y1:y2, x1:x2]                         # 전체 화면에서 이 영역만 잘라냄
    if crop.size == 0:                                 # 잘라낸 영역이 비어있으면
        return []                                       # 빈 리스트 반환 (찾은 게 없음)
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)         # 빨간색을 조명 변화에 안정적으로 걸러내기 위해 HSV로 변환
    lower_red1 = np.array([0, 100, 100])                # 빨간색 범위1의 하한값
    upper_red1 = np.array([10, 255, 255])               # 빨간색 범위1의 상한값
    lower_red2 = np.array([160, 100, 100])              # 빨간색 범위2의 하한값
    upper_red2 = np.array([180, 255, 255])              # 빨간색 범위2의 상한값
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)    # 범위1 해당 픽셀 마스크
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)    # 범위2 해당 픽셀 마스크
    red_mask = cv2.bitwise_or(mask1, mask2)             # 두 마스크 합쳐서 최종 빨간 마스크 완성
    kernel = np.ones((3, 3), np.uint8)                  # 노이즈 제거용 3x3 커널 (구간 자체가 좁아져서 커널도 작게 조정)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    # 작은 노이즈 점들 제거
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(red_mask, connectivity=8)
    # 빨간 영역을 덩어리(블롭) 단위로 구분
    blob_areas = stats[1:, cv2.CC_STAT_AREA]            # 배경(stats[0]) 제외한 나머지 덩어리들의 면적
    if len(blob_areas) == 0:                            # 빨간 덩어리가 하나도 없으면
        return []                                        # 빈 리스트 반환
    top_indices = np.argsort(blob_areas)[::-1][:max_blobs]  # 면적 큰 순서로 정렬해서 상위 max_blobs개 인덱스 선택
    results = []                                         # 최종 결과를 담을 빈 리스트
    for idx in top_indices:                              # 선택된 상위 덩어리들을 하나씩 순회
        label_id = idx + 1                               # stats[0]이 배경이라 실제 라벨은 +1
        area = stats[label_id, cv2.CC_STAT_AREA]         # 이 덩어리의 면적
        if area < MIN_RED_PIXELS:                        # 너무 작은 덩어리(노이즈)면
            continue                                      # 건너뜀
        bx = stats[label_id, cv2.CC_STAT_LEFT]           # 덩어리의 왼쪽 x좌표(crop 기준)
        by = stats[label_id, cv2.CC_STAT_TOP]            # 덩어리의 위쪽 y좌표
        bw = stats[label_id, cv2.CC_STAT_WIDTH]          # 덩어리의 너비
        bh = stats[label_id, cv2.CC_STAT_HEIGHT]         # 덩어리의 높이
        final_x1 = x1 + max(0, bx - PADDING)             # crop 기준 좌표를 전체 화면 기준으로 변환
        final_y1 = y1 + max(0, by - PADDING)
        final_x2 = x1 + min(crop.shape[1], bx + bw + PADDING)
        final_y2 = y1 + min(crop.shape[0], by + bh + PADDING)
        results.append((int(final_x1), int(final_y1), int(final_x2), int(final_y2)))
        # 계산된 박스를 결과 리스트에 추가
    return results                                        # 찾은 덩어리들의 박스 좌표 리스트 반환


def get_search_regions(car_box):                      # 차량 박스를 받아 "하이마운트 후보 구간"과 "좌우후미등 후보 구간" 좌표를 계산하는 함수
    x1, y1, x2, y2 = car_box                            # 차량 박스 좌표 분리
    box_height = y2 - y1                                # 차량 박스의 세로 길이
    highmount_region = (
        x1,
        y1 + box_height * HIGHMOUNT_TOP,                # 상단 0% 지점
        x2,
        y1 + box_height * HIGHMOUNT_BOTTOM              # 상단 15% 지점
    )
    taillight_region = (
        x1,
        y1 + box_height * TAILLIGHT_TOP,                # 상단 35% 지점
        x2,
        y1 + box_height * TAILLIGHT_BOTTOM              # 상단 55% 지점
    )
    return highmount_region, taillight_region           # 두 구간 좌표를 튜플로 반환


def get_red_brightness(frame, roi):                  # 지정된 영역(roi)의 빨간색 채널 평균 밝기를 계산하는 함수
    x1, y1, x2, y2 = roi                              # ROI 좌표 분리
    x1, y1 = max(0, x1), max(0, y1)                    # 좌표 보정
    crop = frame[y1:y2, x1:x2]                        # 영역 잘라내기
    if crop.size == 0:                                 # 비어있으면
        return None                                    # None 반환
    b, g, r = cv2.split(crop)                          # 채널 분리
    return float(np.mean(r))                           # 빨간 채널 평균 반환


def main():                                            # 메인 실행 함수
    cap = cv2.VideoCapture(0)                          # 웹캠 열기
    if not cap.isOpened():                             # 카메라 못 열면
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

    history = deque(maxlen=SMOOTHING_FRAMES)           # 밝기값 이력 큐
    baseline = None                                    # 베이스라인 값
    last_brightness = None                             # 최근 밝기값
    print("b: 베이스라인 설정 | q: 종료")
    while True:                                        # 무한 반복
        ok, frame = cap.read()                         # 프레임 읽기
        if not ok:                                      # 못 읽으면
            break
        frame = cv2.resize(frame, (640, 480))          # 프레임 크기 조정
        results = model(frame, classes=[2], verbose=False)  # YOLO로 "차" 클래스만 탐지
        boxes = results[0].boxes                        # 찾은 박스들
        car_found = False                                # 신뢰할 만한 차를 찾았는지 여부
        if len(boxes) > 0:                               # 박스가 있으면
            areas = []                                    # 면적 리스트
            for box in boxes:
                bx1, by1, bx2, by2 = box.xyxy[0].tolist()
                areas.append((bx2 - bx1) * (by2 - by1))    # 각 박스 면적 계산해서 추가
            biggest_idx = int(np.argmax(areas))            # 제일 큰 박스(=제일 가까운 차) 선택
            best_box = boxes[biggest_idx]
            x1, y1, x2, y2 = best_box.xyxy[0].tolist()      # 좌표 꺼내기
            box_area = (x2 - x1) * (y2 - y1)                # 선택된 박스 면적
            frame_area = frame.shape[0] * frame.shape[1]    # 전체 화면 면적
            if box_area >= frame_area * MIN_CAR_AREA_RATIO:  # 충분히 큰 박스면(신뢰할 만하면)
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 1)
                # 차량 전체 박스를 파란색 얇은 테두리로 표시
                highmount_region, taillight_region = get_search_regions((x1, y1, x2, y2))
                # 하이마운트 구간, 좌우후미등 구간 좌표 계산
                hx1, hy1, hx2, hy2 = [int(v) for v in highmount_region]
                cv2.rectangle(frame, (hx1, hy1), (hx2, hy2), (0, 200, 200), 1)
                # 하이마운트 후보 구간을 노란 얇은 테두리로 표시 (탐색 범위 시각화용)
                tx1, ty1, tx2, ty2 = [int(v) for v in taillight_region]
                cv2.rectangle(frame, (tx1, ty1), (tx2, ty2), (200, 200, 0), 1)
                # 좌우후미등 후보 구간을 하늘색 얇은 테두리로 표시 (탐색 범위 시각화용)
                highmount_blobs = find_red_blobs(frame, highmount_region, max_blobs=1)
                # 하이마운트 구간 안에서 빨간 덩어리 최대 1개 찾기
                taillight_blobs = find_red_blobs(frame, taillight_region, max_blobs=2)
                # 좌우후미등 구간 안에서 빨간 덩어리 최대 2개 찾기
                all_blobs = highmount_blobs + taillight_blobs  # 찾은 모든 덩어리(브레이크등 후보들) 합치기
                brightness_values = []                       # 밝기값들을 모을 리스트
                for (bx1, by1, bx2, by2) in all_blobs:        # 찾은 각 덩어리에 대해
                    cv2.rectangle(frame, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
                    # 실제로 찾은 빨간 덩어리는 초록 굵은 테두리로 표시 (탐색범위와 구분되게)
                    b = get_red_brightness(frame, (bx1, by1, bx2, by2))
                    if b is not None:
                        brightness_values.append(b)
                if brightness_values:                        # 밝기값이 하나라도 있으면
                    brightness = float(np.mean(brightness_values))  # 평균 내서 최종 밝기 산출
                    last_brightness = brightness
                    history.append(brightness)
                    smoothed = float(np.mean(history))
                    delta = None
                    if baseline is not None:
                        delta = smoothed - baseline
                        status = "이상(밝아짐)" if delta > DELTA_THRESHOLD else "정상"
                        cv2.putText(frame, f"delta={delta:.1f} {status}", (int(x1), max(0, int(y1) - 10)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                        # ===== ★STM32로 밝기 판정 결과 전송 =====
                        if stm32_ser is not None:                       # STM32 시리얼이 연결되어 있으면(연결 실패시엔 그냥 건너뜀)
                            anomaly_flag = 1 if delta > DELTA_THRESHOLD else 0
                            # STM32 프로토콜 기준: 1=밝기변화 감지됨(제동등 켜짐, 정상), 0=변화없음(제동등 꺼진 것으로 추정)
                            b_msg = f"B,{delta:.1f},{anomaly_flag}\r\n"     # "B,밝기델타,이상여부\r\n" 형식으로 메시지 조립
                            try:                                             # 전송 중 케이블 문제 등으로 에러 나도 영상처리는 안 멈추게
                                stm32_ser.write(b_msg.encode('utf-8'))         # STM32로 전송
                            except Exception as e:
                                print(f"[STM32 전송 에러] {e}")

                    cv2.putText(frame, f"R={smoothed:.1f} (blobs={len(all_blobs)})", (int(x1), int(y2) + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                    # 밝기값과 함께 몇 개의 덩어리(브레이크등 후보)를 찾았는지도 같이 표시
                car_found = True                            # 신뢰할 만한 차량을 찾아 처리했다고 표시
        if not car_found:                                    # 차를 못 찾았거나 너무 작은 경우
            cv2.putText(frame, "차량 인식 대기 중...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # ===== ★STM32에서 받은 레이더 값도 화면에 같이 표시 (디버깅용) =====
        with serial_data_lock:                               # 락 잡고 최신값 읽기(백그라운드 스레드와 동시접근 방지)
            radar_dist_display = latest_radar_distance          # 화면표시용으로 복사
            radar_speed_display = latest_radar_speed
        if radar_dist_display is not None:                   # 한 번이라도 레이더 값을 받은 적 있으면
            cv2.putText(frame, f"Radar: {radar_dist_display:.1f}m {radar_speed_display:.1f}km/h",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        else:                                                  # 아직 한 번도 못 받았으면
            cv2.putText(frame, "Radar: 대기중...", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 2)

        cv2.imshow("BrakeCoach - YOLO Taillight ROI Test", frame)  # 화면 표시
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('b') and last_brightness is not None:
            baseline = last_brightness
            print(f"베이스라인 설정: {baseline:.1f}")
    cap.release()
    cv2.destroyAllWindows()
    if stm32_ser is not None:                              # ★프로그램 종료시 시리얼 포트도 깔끔하게 닫아줌
        stm32_ser.close()


if __name__ == "__main__":
    main()