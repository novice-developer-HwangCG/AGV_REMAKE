import board
import digitalio
import pwmio
import rotaryio
import asyncio
import adafruit_ticks
import math

# ======== 하드웨어 설정 ========
pwm = pwmio.PWMOut(board.GP15, frequency=1000, duty_cycle=0)
# 모터 방향 핀 (False: 정방향)
dir_pin = digitalio.DigitalInOut(board.GP14)
dir_pin.direction = digitalio.Direction.OUTPUT
# 모터 활성화 핀 (0: 활성, 1: 정지)
enable_pin = digitalio.DigitalInOut(board.GP13)
enable_pin.direction = digitalio.Direction.OUTPUT

# 엔코더 (rotaryio 사용)
encoder = rotaryio.IncrementalEncoder(board.GP2, board.GP3)

# ======== 상수 정의 ========
MOTOR_PPR         = 1000             # 엔코더 해상도 (ppr)
GEAR_RATIO        = 10               # 기어비 10:1
PULSES_PER_REV    = MOTOR_PPR * GEAR_RATIO
WHEEL_DIAMETER_MM = 120              # mm
WHEEL_CIRC_MM     = math.pi * WHEEL_DIAMETER_MM
TARGET_MM         = 1000             # 목표 거리 1000 mm (1m)
PWM_LEVEL         = 25               # 0-255 스케일

async def move_distance():
    # 시작 위치 저장
    start = encoder.position

    # 모터 활성화
    pwm.duty_cycle = int(PWM_LEVEL * 65535 // 255)
    dir_pin.value  = False   # 정방향
    enable_pin.value = 0     # 활성화

    while True:
        pos = encoder.position
        # 얼마나 회전했는지 계산 (부호 제거)
        delta = pos - start
        dist_mm = abs(delta) / PULSES_PER_REV * WHEEL_CIRC_MM

        # 매 변화마다 출력
        print(f"pos={pos:6d}  Δ={delta:6d}  dist={dist_mm:6.1f} mm")

        # 목표 도달 시 정지
        if dist_mm >= TARGET_MM:
            pwm.duty_cycle   = 0
            enable_pin.value = 1
            print("[INFO] Target reached. Motor stopped.")
            return

        # 즉시 재스케줄
        await asyncio.sleep(0)

async def main():   
    # 비동기 태스크로 이동 시작
    await move_distance()

# uasyncio 이벤트 루프 실행
asyncio.run(main())