#!/usr/bin/env python3
# ============================================================================
# hex_dump_radar.py
# CH340(USB-TTL) 어댑터로 레이더 TX에 직결한 상태에서, 들어오는 raw 바이트를
# 16진수(hex)로 그대로 화면에 찍어주는 스크립트. PC(윈도우)에서 실행.
#
# 사용법:
#   1) pip install pyserial   (파이썬 안 깔려있으면 python.org에서 먼저 설치)
#   2) 장치관리자(Device Manager) 열어서 "포트(COM & LPT)"에서 CH340이 몇 번 COM인지 확인
#      (예: "USB-SERIAL CH340 (COM5)" 이렇게 나와있을 것)
#   3) 아래 PORT 값을 그 번호로 수정 (예: 'COM5')
#   4) python hex_dump_radar.py 실행
#   5) 화면에 찍히는 hex 바이트들을 통째로 복사해서 나한테 붙여넣어주면 됨
# ============================================================================
import serial   # 시리얼 통신 라이브러리 (pip install pyserial 필요)
import time      # 타임스탬프 찍는 용도

PORT = 'COM7'     # ★반드시 장치관리자에서 확인한 실제 CH340 COM포트 번호로 수정할 것
BAUD = 115200      # HLK-LD2451 기본 baudrate

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.5)   # 시리얼 포트 열기, 0.5초 안에 데이터 없으면 그냥 넘어감
    print(f"[열림] {PORT} @ {BAUD}bps — hex 덤프 시작 (Ctrl+C로 종료)")
    print("-" * 70)

    buf = bytearray()   # 받은 바이트들을 계속 쌓아두는 버퍼 (한 줄에 16바이트씩 끊어 출력하기 위함)

    try:
        while True:                                # 무한 반복
            data = ser.read(64)                      # 최대 64바이트까지 한 번에 읽기 시도 (없으면 빈 값)
            if data:                                  # 뭔가 받았으면
                buf.extend(data)                        # 버퍼에 이어붙임
                while len(buf) >= 16:                   # 버퍼에 16바이트 이상 쌓였으면 한 줄씩 출력
                    line = buf[:16]                       # 앞 16바이트만 잘라냄
                    del buf[:16]                          # 버퍼에서 그만큼 제거
                    hex_str = ' '.join(f'{b:02X}' for b in line)  # 각 바이트를 두 자리 16진수 대문자로, 공백으로 구분
                    ts = time.strftime('%H:%M:%S')          # 현재 시각
                    print(f"[{ts}] {hex_str}")               # 출력
    except KeyboardInterrupt:                       # Ctrl+C로 종료시
        if buf:                                        # 마지막에 16바이트 못 채우고 남은 게 있으면
            hex_str = ' '.join(f'{b:02X}' for b in buf)      # 그것도 마저 출력
            print(f"[남은 데이터] {hex_str}")
        print("\n종료함")

if __name__ == '__main__':
    main()
