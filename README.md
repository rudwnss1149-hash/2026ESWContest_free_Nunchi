# 브레이크코치 (BrakeCoach)

전방 차량 제동등 이상 감지 및 실시간 경고 시스템 — 임베디드 소프트웨어 경진대회 자유공모

## 팀
- 팀장 박경준 — STM32 메인 로직, 통합, 실차 테스트 조율
- 정훈 — 하드웨어, 배선, 전원계통
- 경록 — AI/코드, 비전 처리, 알고리즘

## 폴더 구조

```
brakecoach/
├── stm32_firmware/     # STM32CubeIDE 프로젝트 전체 (경준 메인)
├── pi_vision/          # Raspberry Pi Zero 2WH용 Python 코드 (경록 메인)
│   ├── src/            # 실제 소스코드
│   └── tests/          # 가짜 데이터 기반 유닛테스트
├── hardware/           # 배선도, BOM (정훈 메인)
│   └── wiring_diagrams/
├── docs/               # 기획서, 회의록, 데이터시트 모음
│   ├── meeting_notes/
│   └── datasheets/
└── README.md
```

## 개발 현황 (진행하면서 업데이트)

- [ ] 레이더(HLK-LD2451) UART 파싱 검증
- [ ] ELM327 블루투스 페어링 검증
- [ ] Pi 카메라 밝기 델타 알고리즘 (노트북 웹캠으로 선행 개발 중)
- [ ] STM32 핀맵 설정 (CubeMX)
- [ ] STM32-레이더 UART 연동
- [ ] STM32-Pi UART 연동
- [ ] 자기운동보정 로직
- [ ] 불일치 판정 로직 (0.5초 지속 필터)
- [ ] cut-in 필터
- [ ] 실차 1차 테스트
- [ ] 실차 최종 촬영

## 브랜치 전략

- `main`: 항상 동작하는 상태만 머지
- `firmware/*`: STM32 관련 작업 (예: `firmware/uart-radar`)
- `vision/*`: Pi 비전 관련 작업 (예: `vision/brightness-delta`)
- `docs/*`: 문서 작업

각자 자기 브랜치에서 작업 → PR 올리고 다른 팀원 1명 리뷰 후 머지 권장.

## 참고 문서

- `docs/` 폴더에 기획서, 핀맵 계획, 각 부품 데이터시트 정리
- 부품 리스트/구매현황은 `hardware/BOM.md` 참고
