import board
import busio
import digitalio
import time
from adafruit_wiznet5k.adafruit_wiznet5k import WIZNET5K

SPI1_SCK = board.GP2
SPI1_TX = board.GP3 # MOSI
SPI1_RX = board.GP12 # MISO
SPI1_CSn = board.GP13   # SS
# W5500_RSTn = board.GP15 # optional

print("Wiznet5k Ping Test (no DHCP)")

# Setup your network configuration below
# random MAC, later should change this value on your vendor ID
MY_MAC = (0x00, 0x01, 0x02, 0x03, 0x04, 0x05)
IP_ADDRESS = (192, 168, 0, 111)
SUBNET_MASK = (255, 255, 0, 0)
GATEWAY_ADDRESS = (192, 168, 0, 1)
DNS_SERVER = (8, 8, 8, 8)

# ethernetRst = digitalio.DigitalInOut(W5500_RSTn)
# ethernetRst.direction = digitalio.Direction.OUTPUT

# For Adafruit Ethernet FeatherWing
cs = digitalio.DigitalInOut(SPI1_CSn)
# For Particle Ethernet FeatherWing
# cs = digitalio.DigitalInOut(board.D5)
spi_bus = busio.SPI(SPI1_SCK, MOSI=SPI1_TX, MISO=SPI1_RX)

# Initialize ethernet interface with DHCP
# eth = WIZNET5K(spi_bus, cs)
# Initialize ethernet interface without DHCP
eth = WIZNET5K(spi_bus, cs, is_dhcp=False, mac=MY_MAC)

# Set network configuration
eth.ifconfig = (IP_ADDRESS, SUBNET_MASK, GATEWAY_ADDRESS, DNS_SERVER)

led = digitalio.DigitalInOut(board.GP25)
led.direction = digitalio.Direction.OUTPUT

print("Chip Version:", eth.chip)
print("MAC Address:", [hex(i) for i in eth.mac_address])
print("My IP address is:", eth.pretty_ip(eth.ip_address))

while True:
    led.value = not led.value
    time.sleep(1)

print("Done!")