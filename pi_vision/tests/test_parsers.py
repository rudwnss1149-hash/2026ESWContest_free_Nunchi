"""
하드웨어 없이 실행 가능한 파서 유닛테스트
실행: python -m pytest tests/ (프로젝트 루트에 pytest 설치 필요: pip install pytest)
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from elm327_client import parse_speed_response, build_speed_request
from radar_parser import parse_frame


def test_elm327_normal_response():
    assert parse_speed_response("41 0D 32") == 0x32  # 50

def test_elm327_response_with_noise():
    assert parse_speed_response("41 0D 00\r>") == 0

def test_elm327_no_data():
    assert parse_speed_response("NO DATA") is None

def test_elm327_empty():
    assert parse_speed_response("") is None

def test_elm327_request_format():
    assert build_speed_request() == b"010D\r"


def test_radar_valid_frame():
    frame = bytes([0xAA, 0x55, 0x0B, 0xB8, 0xFB, 0x02, 0x00])
    result = parse_frame(frame)
    assert result is not None
    assert result.distance_m == 30.0
    assert result.speed_kmh == -5.0
    assert result.angle_deg == 2.0

def test_radar_invalid_header():
    frame = bytes([0x00, 0x00, 0x0B, 0xB8, 0xFB, 0x02, 0x00])
    assert parse_frame(frame) is None

def test_radar_too_short():
    assert parse_frame(bytes([0xAA, 0x55])) is None


if __name__ == "__main__":
    # pytest 없이도 그냥 실행해서 확인 가능하도록
    import traceback
    tests = [v for k, v in list(globals().items()) if k.startswith("test_")]
    passed, failed = 0, 0
    for t in tests:
        try:
            t()
            print(f"PASS: {t.__name__}")
            passed += 1
        except AssertionError:
            print(f"FAIL: {t.__name__}")
            traceback.print_exc()
            failed += 1
    print(f"\n{passed} passed, {failed} failed")
