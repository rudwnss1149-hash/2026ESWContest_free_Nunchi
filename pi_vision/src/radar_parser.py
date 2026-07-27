"""
HLK-LD2451 UART 프레임 파서

⚠️ 중요: 아래 프레임 포맷은 확정된 게 아니라 "일반적인 Hi-Link 레이더 모듈들의 전형적인 구조"를
참고한 임시 스켈레톤입니다. 실제 HLK-LD2451 데이터시트(또는 구매처 제공 자료)를 받으면
헤더 바이트, 필드 순서, 길이를 반드시 재확인하고 이 파일을 수정해야 합니다.

지금 할 수 있는 것: 이 구조로 "파서 함수의 인터페이스(입력: bytes, 출력: dict)"를 미리 정하고,
STM32/Pi 쪽 코드에서 이 인터페이스에 맞춰 통신 파이프라인을 먼저 구성해두는 것.
실제 프레임 포맷만 나중에 맞추면 나머지 파이프라인은 그대로 재사용 가능.

TODO(부품 도착 후):
- [ ] 실제 헤더 바이트 확인
- [ ] 거리/속도/각도 필드의 바이트 순서(리틀엔디안/빅엔디안), 스케일(단위) 확인
- [ ] 체크섬 유무 확인
- [ ] 다중 타겟 지원 여부 확인 (여러 대 감지 시 프레임 구조가 어떻게 바뀌는지)
"""

from dataclasses import dataclass


@dataclass
class RadarTarget:
    distance_m: float
    speed_kmh: float
    angle_deg: float


def parse_frame(raw: bytes) -> RadarTarget | None:
    """
    임시 가정 포맷 (실제 데이터시트 확인 후 반드시 수정):
    [0xAA][0x55][distance_hi][distance_lo][speed][angle][checksum]
    - distance: cm 단위, 2바이트
    - speed: km/h, signed 1바이트 (음수면 멀어지는 중)
    - angle: degree, signed 1바이트
    """
    if len(raw) < 7:
        return None
    if raw[0] != 0xAA or raw[1] != 0x55:
        return None

    distance_cm = (raw[2] << 8) | raw[3]
    speed_raw = raw[4]
    speed_kmh = speed_raw - 256 if speed_raw > 127 else speed_raw
    angle_raw = raw[5]
    angle_deg = angle_raw - 256 if angle_raw > 127 else angle_raw

    # TODO: 체크섬(raw[6]) 검증 로직 추가

    return RadarTarget(
        distance_m=distance_cm / 100.0,
        speed_kmh=float(speed_kmh),
        angle_deg=float(angle_deg),
    )


if __name__ == "__main__":
    # 가짜 프레임으로 파서 동작만 먼저 확인 (실제 포맷 아님, 인터페이스 검증용)
    fake_frame = bytes([0xAA, 0x55, 0x0B, 0xB8, 0xFB, 0x02, 0x00])  # distance=3000cm=30m, speed=-5km/h, angle=2deg
    result = parse_frame(fake_frame)
    print(result)
