# STM32F407VET6 핀맵 계획

CubeMX 설정 시 이 표 기준으로 페리페럴 배정. 실제 물리 핀 번호는 보드(SZH-DVBS-022) 실크스크린 라벨과 대조해서 확정.

| 페리페럴 | 용도 | 연결 대상 | Baudrate(안) |
|---|---|---|---|
| USART1 | 레이더 통신 | HLK-LD2451 (TX/RX) | 9600 (확인 필요) |
| USART2 | Pi 통신 | Pi Zero 2WH (밝기델타 결과 수신) | 115200 |
| USART3 | 블루투스 중계 | HC-05 (→ELM327) | 9600 (기본값) |
| SPI1 | 디스플레이 | ST7789 (SCK/MOSI + RES/DC/CS GPIO) | - |
| TIM3 CH1 (PWM) | 부저 | Passive 부저 | - |
| SWD | 프로그래밍 | ST-LINK V2 | - |

## CubeMX 설정 순서
1. MCU 선택: STM32F407VET6
2. USART1/2/3 → Mode: Asynchronous
3. SPI1 → Mode: Full-Duplex Master
4. TIM3 → Channel1 → PWM Generation CH1
5. SYS → Debug → Serial Wire
6. Clock Configuration 확인 (기본값으로 우선 진행, 이상 있으면 재조정)
7. 코드 생성 (Project Manager 탭에서 Toolchain: STM32CubeIDE 선택 후 GENERATE CODE)

## 확인 필요 사항 (부품 도착 후)
- [ ] HLK-LD2451 실제 Baudrate 및 UART 프레임 포맷 (데이터시트/구매처 문의로 확인)
- [ ] HC-05 기본 Baudrate 확인 (보통 9600, AT모드 진입 시 다를 수 있음)
- [ ] ST7789 SPI 최대 클럭 속도 확인
