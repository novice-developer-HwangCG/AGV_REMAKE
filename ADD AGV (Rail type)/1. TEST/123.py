from machine import Pin, PWM
import utime

# ————— 설정 —————
PWM_PIN    = 4
ENABLE_PIN = 2
FREQ       = 20000  # 한 주기 T = 1/20 000 s = 50 μs / 켜져 있는 시간(on time)과 꺼져 있는 시간(off time)이 듀티 비율
RUN_TIME   = 3
STOP_TIME  = 5
WRAP_8BIT  = 255
LEVEL_8BIT = 20

def duty8_to_u16(d8):
    return int(d8 * 65535 // WRAP_8BIT)

enable = Pin(ENABLE_PIN, Pin.OUT)
pwm    = PWM(Pin(PWM_PIN))
pwm.freq(FREQ)
pwm.duty_u16(0)

try:
    while True:
        enable.value(1)
        pwm.duty_u16(duty8_to_u16(LEVEL_8BIT))
        utime.sleep(RUN_TIME)

        pwm.duty_u16(0)
        enable.value(0)
        utime.sleep(STOP_TIME)

except KeyboardInterrupt:
    pwm.duty_u16(0)
    enable.value(0)
    print("Stopped by user")
