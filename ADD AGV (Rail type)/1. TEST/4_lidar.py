import board
import busio
import time

"""해당 코드는 circuitpython 9.2.8 버전에서 사용됨"""

# 1) CRC8 함수는 그대로 사용
def crc8(buf: bytes) -> int:
    crc = 0x00
    for b in buf:
        crc ^= b
        for _ in range(8):
            crc = (crc << 1) ^ 0x31 if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc

# 2) make_frame: Value를 4바이트 big-endian 정수로 바꿔 포장
def make_frame(key: int, value: int) -> bytes:
    # key(1B) + value(4B) 로 payload 구성
    value_bytes = value.to_bytes(4, 'big')        # e.g. 50 -> b'\x00\x00\x00\x32'
    payload     = bytes([key]) + value_bytes      # 총 5바이트
    c           = crc8(payload)                   # CRC8 over payload
    return b'\x55' + payload + bytes([c, 0xAA])   # [헤더][payload][CRC][꼬리]

# 3) 프레임 정의: frequency는 50Hz(0x32)로 변경
frame_set_freq  = make_frame(0x03, 50)     # (max 500Hz까지 일반 모드)
frame_set_baud  = make_frame(0x12, 0x0C)   # 115200bps
frame_set_fmt   = make_frame(0x04, 0x01)   # Byte format
frame_start     = make_frame(0x05, 0x00)   # Start

uart = busio.UART(tx=board.GP0, rx=board.GP1,
                  baudrate=115200, timeout=0.1)

time.sleep(0.1)
print(">> Set Frequency  50Hz:", frame_set_freq.hex())
uart.write(frame_set_freq)
time.sleep(0.05)

print(">> Set Baud   115200:", frame_set_baud.hex())
uart.write(frame_set_baud)
time.sleep(0.1)

print(">> Set Format   byte:", frame_set_fmt.hex())
uart.write(frame_set_fmt)
time.sleep(0.1)

print(">> Start meas.:", frame_start.hex())
uart.write(frame_start)
# 충분히 spin-up 대기
time.sleep(1.0)

print(">> Waiting for Distance frames (key=0x07)")

while True:
    # 1) 헤더 싱크
    b = uart.read(1)
    if not b or b[0] != 0x55:
        continue

    # 2) 나머지 7바이트 읽기
    rest = uart.read(7)
    if not rest or rest[-1] != 0xAA:
        continue

    frame = b + rest  # 8바이트 완전 프레임

    # 3) CRC 체크
    if crc8(frame[1:6]) != frame[6]:
        continue

    # 4) 키가 0x07인지 확인
    if frame[1] == 0x07:
        status = frame[2]
        dist   = (frame[3]<<16)|(frame[4]<<8)|frame[5]
        if status == 0x00 and 50 <= dist <= 40000:
            print(f"Distance: {dist} mm")
        else:
            print(f"LiDAR status {status}, out-of-range/error")


""" -------------------- circuitpython 6.2.0 버전 -------------------- """

import board
import busio
import time

# 1) CRC8 함수는 그대로 사용
def crc8(buf: bytes) -> int:
    crc = 0x00
    for b in buf:
        crc ^= b
        for _ in range(8):
            crc = (crc << 1) ^ 0x31 if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc

# 2) make_frame: Value를 4바이트 big-endian 정수로 바꿔 포장
def make_frame(key: int, value: int) -> bytes:
    # key(1B) + value(4B) 로 payload 구성
    value_bytes = value.to_bytes(4, 'big')        # e.g. 50 -> b'\x00\x00\x00\x32'
    payload     = bytes([key]) + value_bytes      # 총 5바이트
    c           = crc8(payload)                   # CRC8 over payload
    return b'\x55' + payload + bytes([c, 0xAA])   # [헤더][payload][CRC][꼬리]

def to_hexstr(b: bytes) -> str:
    return ''.join(f"{x:02x}" for x in b)

# 3) 프레임 정의: frequency는 50Hz(0x32)로 변경
frame_set_freq  = make_frame(0x03, 50)     # (max 500Hz까지 일반 모드)
frame_set_baud  = make_frame(0x12, 0x0C)   # 115200bps
frame_set_fmt   = make_frame(0x04, 0x01)   # Byte format
frame_start     = make_frame(0x05, 0x00)   # Start

uart = busio.UART(tx=board.GP0, rx=board.GP1,
                  baudrate=115200, timeout=0.1)

time.sleep(0.1)
print(">> Set Frequency  50Hz:", to_hexstr(frame_set_freq))
uart.write(frame_set_freq)
time.sleep(0.05)

print(">> Set Baud   115200:", to_hexstr(frame_set_baud))
uart.write(frame_set_baud)
time.sleep(0.1)

print(">> Set Format   byte:", to_hexstr(frame_set_fmt))
uart.write(frame_set_fmt)
time.sleep(0.1)

print(">> Start meas.:", to_hexstr(frame_start))
uart.write(frame_start)
# 충분히 spin-up 대기
time.sleep(1.0)

print(">> Waiting for Distance frames (key=0x07)")

while True:
    # 1) 헤더 싱크
    b = uart.read(1)
    if not b or b[0] != 0x55:
        continue

    # 2) 나머지 7바이트 읽기
    rest = uart.read(7)
    if not rest or rest[-1] != 0xAA:
        continue

    frame = b + rest  # 8바이트 완전 프레임

    # 3) CRC 체크
    if crc8(frame[1:6]) != frame[6]:
        continue

    # 4) 키가 0x07인지 확인
    if frame[1] == 0x07:
        status = frame[2]
        dist   = (frame[3]<<16)|(frame[4]<<8)|frame[5]
        if status == 0x00 and 50 <= dist <= 40000:
            print(f"Distance: {dist} mm")
        else:
            print(f"LiDAR status {status}, out-of-range/error")


