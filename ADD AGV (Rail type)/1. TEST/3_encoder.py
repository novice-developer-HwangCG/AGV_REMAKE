""" 단순 폴링 방식 """
import time
import board
import digitalio
import pwmio

# 하드웨어 설정
pwm = pwmio.PWMOut(board.GP15, frequency=1000, duty_cycle= 240 * 65535 // 255)
dir_pin = digitalio.DigitalInOut(board.GP14)
dir_pin.direction  = digitalio.Direction.OUTPUT
dir_pin.drive_mode = digitalio.DriveMode.OPEN_DRAIN
enable_pin = digitalio.DigitalInOut(board.GP13)
enable_pin.direction = digitalio.Direction.OUTPUT

encoderA = digitalio.DigitalInOut(board.GP2)
encoderA.direction = digitalio.Direction.INPUT
encoderA.pull = digitalio.Pull.UP

encoderB = digitalio.DigitalInOut(board.GP3)
encoderB.direction = digitalio.Direction.INPUT
encoderB.pull = digitalio.Pull.UP

# 초기 상태
dir_pin.value = False
enable_pin.value = 1   # 모터 활성화

# 상태 머신 변수
last_state  = (encoderA.value << 1) | encoderB.value
pulse_count = 0

print("Starting motor at PWM=240, reading encoder via state-machine…")

while True:
    a = encoderA.value
    b = encoderB.value
    state = (a << 1) | b

    if state != last_state:
        # 다음 전진 상태: (last_state + 1) mod 4
        next_state = (last_state + 1) & 0x03
        if state == next_state:
            pulse_count += 1
        else:
            pulse_count -= 1

        print(f"A={a} B={b}  state={state}  Pulse count: {pulse_count}")
        last_state = state

    # 너무 빠르면 놓치니 5ms 정도 폴링
    time.sleep(0.005)



""" asyncio 방식 """
import board
import digitalio
import pwmio
import rotaryio
import asyncio
import adafruit_ticks

# 모터 설정 (예시)
pwm = pwmio.PWMOut(board.GP15, frequency=1000, duty_cycle=25 * 65535 // 255)
dir_pin = digitalio.DigitalInOut(board.GP14)
dir_pin.direction  = digitalio.Direction.OUTPUT
enable_pin = digitalio.DigitalInOut(board.GP13)
enable_pin.direction = digitalio.Direction.OUTPUT

# 엔코더 (rotaryio 사용)
encoder = rotaryio.IncrementalEncoder(board.GP2, board.GP3)

async def encoder_monitor():
    last_pos = encoder.position
    while True:
        pos = encoder.position
        if pos != last_pos:
            # 변화가 생겼을 때만 처리
            delta = pos - last_pos
            print(f"Encoder moved: Δ={delta}  total={pos}")
            last_pos = pos
        # 즉시 다음 태스크로 제어권 넘기기 (사실상 이벤트 드리블)
        await asyncio.sleep(0)

async def main():
    # 모터 활성화
    dir_pin.value = False
    enable_pin.value = 0

    # 인코더 모니터 태스크 시작
    asyncio.create_task(encoder_monitor())

    # 다른 잡업무 (예: 1초마다 상태 출력)
    while True:
        await asyncio.sleep(1)
        #print("Heartbeat")

asyncio.run(main())

