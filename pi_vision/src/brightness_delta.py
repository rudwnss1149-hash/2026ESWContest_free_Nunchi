"""
ROI 밝기 델타 분석 — 노트북 웹캠으로 선행 개발/테스트 가능
실제 Pi + 최종 카메라로 교체해도 이 로직은 그대로 재사용됨

사용법 (선행 테스트):
    python brightness_delta.py
    스페이스바를 누르면 현재 ROI 평균 밝기를 출력.
    스마트폰 손전등을 ROI 앞에서 켰다/껐다 하면서 델타가 잡히는지 확인.
"""

import cv2
import numpy as np
from collections import deque

# ROI는 지금은 화면 중앙 고정값. 실제 시스템에서는 레이더 좌표 투영 결과로 매 프레임 갱신됨.
ROI = (280, 200, 80, 80)  # x, y, w, h — 320x240 기준 화면 중앙 부근

# 노이즈(순간 흔들림) 방지를 위한 이동평균 프레임 수
SMOOTHING_FRAMES = 5

# 이상 판단 임계값 (실측하면서 튜닝 필요)
DELTA_THRESHOLD = 15.0


def get_roi_brightness(frame, roi):
    x, y, w, h = roi
    crop = frame[y:y + h, x:x + w]
    # 적색 채널 평균 (제동등은 붉은 계열이므로 R채널 위주로 판단)
    b, g, r = cv2.split(crop)
    return float(np.mean(r))


def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    history = deque(maxlen=SMOOTHING_FRAMES)
    baseline = None

    print("스페이스바: 현재 밝기 기록 | b: 베이스라인 설정 | q: 종료")

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        frame = cv2.resize(frame, (320, 240))
        x, y, w, h = ROI
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        brightness = get_roi_brightness(frame, ROI)
        history.append(brightness)
        smoothed = float(np.mean(history))

        delta = None
        if baseline is not None:
            delta = smoothed - baseline
            status = "이상(밝아짐)" if delta > DELTA_THRESHOLD else "정상"
            cv2.putText(frame, f"delta={delta:.1f} {status}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(frame, f"brightness={smoothed:.1f}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        cv2.imshow("BrakeCoach - Brightness Delta Test", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('b'):
            baseline = smoothed
            print(f"베이스라인 설정: {baseline:.1f}")
        elif key == ord(' '):
            print(f"현재 밝기: {smoothed:.1f}" + (f", delta: {delta:.1f}" if delta is not None else ""))

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
