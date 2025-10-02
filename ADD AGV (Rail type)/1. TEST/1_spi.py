import time
import board
import busio
import digitalio

led = digitalio.DigitalInOut(board.GP25)
led.direction = digitalio.Direction.OUTPUT

def blink(times=1, duration=0.3):
    for _ in range(times):
        led.value = True
        time.sleep(duration)
        led.value = False
        time.sleep(duration)

SPI1_SCK = board.GP10
SPI1_MOSI = board.GP11
SPI1_MISO = board.GP12

blink(1, 3)

try:
    spi = busio.SPI(SPI1_SCK, MOSI=SPI1_MOSI, MISO=SPI1_MISO)
    led.value = True  # 정상 listen까지 오면 켜짐
    print("done")
except Exception as e:
    while True:   # 에러 시 빠른 점멸
        blink(3, 0.1)