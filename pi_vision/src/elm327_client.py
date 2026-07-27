"""
ELM327 AT명령 통신 클라이언트
표준 OBD-II PID 프로토콜(SAE J1979) 기반 — 실기기 없이도 응답 파싱 로직은 미리 검증 가능

실제 통신은 블루투스(HC-05 경유 시 STM32 UART, 또는 Pi 직결 시 PySerial+블루투스 시리얼)로 이뤄짐.
이 파일은 "명령 생성"과 "응답 파싱"만 다루고, 실제 전송은 uart_link.py 등에서 처리.
"""

import re


def build_speed_request() -> bytes:
    """차량 속도 요청 명령 (Mode 01, PID 0D)"""
    return b"010D\r"


def parse_speed_response(raw: str) -> int | None:
    """
    ELM327 응답 예시: "41 0D 32" -> 0x32 = 50 (km/h)
    응답에 노이즈(개행, 프롬프트 '>' 등)가 섞여 올 수 있어 정규식으로 필요한 부분만 추출.
    """
    match = re.search(r"41\s*0D\s*([0-9A-Fa-f]{2})", raw)
    if not match:
        return None
    return int(match.group(1), 16)


if __name__ == "__main__":
    # 하드웨어 없이 파서 자체를 바로 확인해볼 수 있는 간단 데모
    fake_responses = [
        "41 0D 32",       # 정상: 50km/h
        "41 0D 00\r>",    # 정상: 0km/h (개행/프롬프트 포함)
        "NO DATA",        # 비정상 응답
        "",               # 빈 응답
    ]
    for r in fake_responses:
        print(f"raw={r!r:20} -> speed={parse_speed_response(r)}")
