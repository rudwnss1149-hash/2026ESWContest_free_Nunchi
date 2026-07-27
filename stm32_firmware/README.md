# STM32 Firmware (경준 메인 담당)

## 프로젝트 생성 방법

1. STM32CubeIDE 실행 → File → New → STM32 Project
2. MCU: STM32F407VET6 선택
3. 프로젝트 이름: `brakecoach_fw` (또는 원하는 이름)
4. **저장 위치를 이 폴더(`stm32_firmware/`) 안으로 지정** — 그래야 Git으로 같이 관리됨
5. `../docs/pinmap.md` 참고해서 CubeMX에서 페리페럴(USART1/2/3, SPI1, TIM3) 설정
6. 코드 생성 후 커밋

## 폴더가 비어있는 이유

STM32CubeIDE 프로젝트는 실제로 생성해야 폴더 구조(Core/Src, Core/Inc, Drivers 등)가 나옵니다.
지금은 빈 폴더 + 이 README만 있는 상태 — 첫 프로젝트 생성 후 바로 커밋해서 채워주세요.

## 지금 당장 할 수 있는 것 (보드 없이)

- CubeMX에서 핀맵 설정 및 코드 생성까지는 보드 없이 가능
- HAL 초기화 코드가 정상 생성되는지, 빌드가 되는지(업로드는 안 되지만 컴파일은 가능)까지 확인
- 레이더/ELM327 UART 인터럽트 수신 구조를 의사코드로 미리 설계해두기
