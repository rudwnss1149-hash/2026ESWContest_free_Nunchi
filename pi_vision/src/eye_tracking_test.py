"""
얼굴/눈 인식으로 ROI가 눈을 따라오게 하고, 그 영역의 밝기 델타를 재는 테스트 코드
실제 시스템에서 "레이더 좌표 → ROI 위치 갱신"이 될 부분을, 지금은 "눈 위치 → ROI 위치 갱신"으로 대체해서
동적으로 움직이는 ROI 로직 자체를 미리 검증하는 용도

사용법: python src/eye_tracking_test.py
   b: 베이스라인(현재 밝기)을 기준값으로 저장
   q: 종료
"""

import cv2                                          # OpenCV 라이브러리를 cv2라는 이름으로 불러옴 (영상처리 담당)
import numpy as np                                  # 숫자 배열 계산을 위한 numpy 라이브러리를 np라는 이름으로 불러옴
from collections import deque                        # 최근 N개 값만 유지하는 큐 자료구조를 쓰기 위해 불러옴

FACE_CASCADE_PATH = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
                                                     # OpenCV에 기본 내장된 얼굴 인식용 학습 데이터 파일 경로
EYE_CASCADE_PATH = cv2.data.haarcascades + 'haarcascade_eye.xml'
                                                     # OpenCV에 기본 내장된 눈 인식용 학습 데이터 파일 경로

SMOOTHING_FRAMES = 5                                # 밝기값 노이즈를 줄이기 위해 평균낼 프레임 개수
DELTA_THRESHOLD = 15.0                              # 이 값보다 밝기 변화가 크면 "이상(밝아짐)"으로 판정하는 기준치

ROI_PADDING = 15                                    # 눈 주변으로 여유 있게 잡을 픽셀 수 (눈 크기보다 살짝 넉넉하게)


def get_roi_brightness(frame, roi):                 # 특정 영역(roi)의 평균 밝기를 계산하는 함수 정의
    x, y, w, h = roi                                # roi 튜플을 x좌표, y좌표, 너비, 높이로 각각 분리해서 저장

    x = max(0, x)                                   # x가 화면 밖(음수)으로 나가지 않도록 0 이상으로 보정
    y = max(0, y)                                   # y가 화면 밖(음수)으로 나가지 않도록 0 이상으로 보정

    crop = frame[y:y + h, x:x + w]                  # 전체 화면(frame)에서 해당 영역만 잘라냄

    if crop.size == 0:                              # 잘라낸 영역이 비어있는 경우(좌표 오류 등)를 확인
        return None                                 # 계산할 수 없으므로 None 반환

    b, g, r = cv2.split(crop)                       # 잘라낸 영역을 파랑(b), 초록(g), 빨강(r) 채널로 분리
    return float(np.mean(r))                        # 빨강 채널의 평균값을 계산해서 반환 (제동등이 붉은 계열이므로)


def main():                                          # 메인 실행 함수 정의
    face_cascade = cv2.CascadeClassifier(FACE_CASCADE_PATH)
                                                     # 얼굴 인식기 객체를 학습 데이터 파일로부터 생성
    eye_cascade = cv2.CascadeClassifier(EYE_CASCADE_PATH)
                                                     # 눈 인식기 객체를 학습 데이터 파일로부터 생성

    cap = cv2.VideoCapture(0)                       # 0번 카메라(보통 노트북 기본 웹캠)를 열어서 cap이라는 이름으로 저장

    if not cap.isOpened():                          # 카메라가 정상적으로 열렸는지 확인
        print("카메라를 열 수 없습니다.")              # 안 열렸으면 에러 메시지 출력
        return                                      # 함수 종료

    history = deque(maxlen=SMOOTHING_FRAMES)        # 최근 밝기값들을 저장할 큐, 최대 5개까지만 유지(오래된 건 자동 삭제)
    baseline = None                                 # 기준 밝기값을 저장할 변수, 처음엔 아직 없으므로 None
    current_roi = None                              # 현재 추적 중인 ROI 좌표를 저장할 변수, 처음엔 아직 없으므로 None

    print("b: 베이스라인 설정 | q: 종료")              # 사용자에게 조작 방법 안내 출력

    while True:                                     # 무한 반복 시작 (매 프레임마다 처리)
        ok, frame = cap.read()                      # 카메라에서 한 프레임을 읽어옴, ok는 성공여부, frame은 영상 데이터

        if not ok:                                  # 프레임을 못 읽었으면
            break                                   # 반복문 종료

        frame = cv2.resize(frame, (320, 240))       # 처리 속도를 위해 프레임 크기를 320x240으로 축소

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                                                     # 얼굴/눈 인식은 흑백에서 더 안정적이므로 흑백 영상으로 변환

        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)
                                                     # 흑백 영상에서 얼굴들을 찾아 (x,y,너비,높이) 목록으로 반환

        eye_found = False                           # 이번 프레임에서 눈을 찾았는지 표시하는 변수, 기본값 False

        for (fx, fy, fw, fh) in faces:               # 찾은 얼굴들을 하나씩 순회 (보통 얼굴 1개만 잡힘)
            cv2.rectangle(frame, (fx, fy), (fx + fw, fy + fh), (255, 0, 0), 1)
                                                     # 화면에 얼굴 영역을 파란색 얇은 테두리로 표시 (참고용)

            face_gray = gray[fy:fy + fh, fx:fx + fw]  # 얼굴 영역만 잘라낸 흑백 영상 (눈 찾기를 이 안에서만 하기 위함)

            eyes = eye_cascade.detectMultiScale(face_gray, scaleFactor=1.1, minNeighbors=10)
                                                     # 얼굴 영역 안에서 눈들을 찾아 (x,y,너비,높이) 목록으로 반환

            for (ex, ey, ew, eh) in eyes:             # 찾은 눈들을 하나씩 순회 (보통 2개, 양쪽 눈)
                abs_x = fx + ex                       # 눈 좌표는 얼굴 영역 기준 상대좌표이므로, 전체 화면 기준 절대좌표로 변환(x)
                abs_y = fy + ey                       # 마찬가지로 y좌표도 전체 화면 기준으로 변환

                current_roi = (abs_x - ROI_PADDING, abs_y - ROI_PADDING,
                               ew + ROI_PADDING * 2, eh + ROI_PADDING * 2)
                                                     # 눈 위치를 중심으로 여유 패딩을 더한 ROI 좌표를 계산해서 저장
                                                     # (이 부분이 실제 시스템에서는 "레이더 좌표 투영 계산"으로 대체될 자리)

                eye_found = True                      # 눈을 찾았다고 표시
                break                                # 첫 번째 눈만 쓰기로 하고 반복 중단 (양쪽 다 쓰려면 이 줄 제거)

            break                                    # 첫 번째 얼굴만 쓰기로 하고 반복 중단

        if current_roi is not None:                 # 지금까지 한 번이라도 ROI가 설정된 적 있으면 (이번 프레임에 눈 못 찾아도 이전 위치 유지)
            x, y, w, h = current_roi                # ROI 좌표를 각각의 변수로 분리
            color = (0, 255, 0) if eye_found else (0, 165, 255)
                                                     # 이번 프레임에 눈을 찾았으면 초록색, 못 찾아 이전 위치 유지 중이면 주황색으로 표시
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                                                     # 화면에 ROI 사각형을 해당 색으로 그림

            brightness = get_roi_brightness(frame, current_roi)
                                                     # 현재 ROI 영역의 밝기를 계산

            if brightness is not None:              # 밝기 계산이 정상적으로 됐으면
                history.append(brightness)          # 이번 밝기값을 이력(history)에 추가
                smoothed = float(np.mean(history))  # 최근 몇 프레임의 평균을 내서 노이즈를 줄인 값 계산

                delta = None                         # 델타값을 저장할 변수, 기본값 None
                if baseline is not None:            # 베이스라인이 설정되어 있으면
                    delta = smoothed - baseline     # 현재 밝기에서 베이스라인을 뺀 차이(델타)를 계산
                    status = "이상(밝아짐)" if delta > DELTA_THRESHOLD else "정상"
                                                     # 델타가 임계값보다 크면 이상, 아니면 정상으로 판정
                    cv2.putText(frame, f"delta={delta:.1f} {status}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                                                     # 화면에 델타값과 판정 결과를 빨간 글씨로 표시

                cv2.putText(frame, f"brightness={smoothed:.1f}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                                                     # 화면에 현재 밝기값을 노란 글씨로 표시

        else:                                        # 아직 한 번도 눈을 못 찾은 경우
            cv2.putText(frame, "눈 인식 대기 중...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                                                     # 화면에 안내 문구 표시

        cv2.imshow("BrakeCoach - Eye Tracking ROI Test", frame)
                                                     # 지금까지 그려진 화면을 창에 띄워서 보여줌

        key = cv2.waitKey(1) & 0xFF                 # 키보드 입력을 1밀리초 동안 대기하며 확인
        if key == ord('q'):                          # 'q' 키가 눌렸으면
            break                                    # 반복문 종료
        elif key == ord('b') and current_roi is not None:
                                                     # 'b' 키가 눌렸고, ROI가 이미 설정되어 있으면
            baseline = smoothed                       # 현재 밝기를 베이스라인으로 저장
            print(f"베이스라인 설정: {baseline:.1f}")   # 콘솔에 설정된 베이스라인 값을 출력

    cap.release()                                    # 카메라 장치를 해제(반납)
    cv2.destroyAllWindows()                          # 열려있던 모든 화면 창을 닫음


if __name__ == "__main__":                           # 이 파일이 직접 실행됐을 때만 아래 코드를 실행 (다른 파일에서 import될 땐 실행 안 됨)
    main()                                            # 메인 함수 호출