# Pi Vision (경록 메인 담당)

Raspberry Pi Zero 2WH에서 돌아가는 Python 코드. 카메라 밝기 델타 분석 + ELM327 통신 담당.

## 환경 세팅 (Pi 없이도 노트북에서 먼저 가능)

```bash
python3 -m venv venv
source venv/bin/activate   # Windows는 venv\Scripts\activate
pip install opencv-python pyserial numpy
```

## 폴더 구조

```
pi_vision/
├── src/
│   ├── brightness_delta.py   # ROI 밝기 델타 분석 (노트북 웹캠으로 선행 개발 가능)
│   ├── elm327_client.py      # ELM327 AT명령 파싱 (실기기 없이도 가짜 응답으로 테스트 가능)
│   ├── radar_parser.py       # HLK-LD2451 UART 프레임 파싱
│   └── uart_link.py          # STM32와의 UART 통신
└── tests/
    └── test_parsers.py       # 가짜 데이터로 파서 검증 (하드웨어 없이 실행 가능)
```

## 지금 당장 할 수 있는 것 (부품 도착 전)

1. `src/brightness_delta.py`: 노트북 내장 웹캠으로 ROI 밝기 평균/델타 계산 로직 작성 및 테스트
2. `src/elm327_client.py`: AT 명령 문자열 파싱 로직만 먼저 작성 (`tests/test_parsers.py`에서 가짜 응답으로 검증)
3. `src/radar_parser.py`: HLK-LD2451 데이터시트의 UART 프레임 포맷대로 파서 작성, 가짜 바이트 배열로 검증
