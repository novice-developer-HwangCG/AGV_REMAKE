import time
import board
import busio
import digitalio
from adafruit_wiznet5k.adafruit_wiznet5k import WIZNET5K

led = digitalio.DigitalInOut(board.GP25)
led.direction = digitalio.Direction.OUTPUT

def blink(times=1, duration=0.3):
    for _ in range(times):
        led.value = True
        time.sleep(duration)
        led.value = False
        time.sleep(duration)

SPI1_SCK = board.GP14
SPI1_MOSI = board.GP15
SPI1_MISO = board.GP12
SPI1_CSn = board.GP13   # SS

spi = busio.SPI(SPI1_SCK, MOSI=SPI1_MOSI, MISO=SPI1_MISO)

try:
    cs = digitalio.DigitalInOut(SPI1_CSn)
    eth = WIZNET5K(spi, cs, is_dhcp=False, mac=(0,1,2,3,4,5))
    print("done")
    led.value = True  # 정상 listen까지 오면 켜짐
except Exception as e:
    while True:
        blink(10, 0.05)

