import serial
import time

# 순환 중복 검사(Cyclic Redundancy Check) 통신 시스템에서 데이터를 전송할 때 오류를 감지하기 위해 사용하는 수학적 기법
def crc8(buf: bytes) -> int:
    """CRC-8 (poly=0x31, init=0x00)"""
    crc = 0x00
    for b in buf:
        crc ^= b
        for _ in range(8):
            crc = (crc << 1) ^ 0x31 if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc

def make_frame(key: int, value_byte: int) -> bytes:
    """
    8-byte 프로토콜 프레임 생성
    [0x55][Key][0x00][0x00][0x00][Value][CRC8][0xAA]
    """
    buf = bytes([key, 0x00, 0x00, 0x00, value_byte])
    c   = crc8(buf)
    return b'\x55' + buf + bytes([c, 0xAA])

PORT = '/dev/ttyUSB0'
BAUD = 115200

# 1) 측정 주파수 설정 (Key=0x03, Value=50Hz 0x32)
frame_set_freq = make_frame(0x03, 50)

# 1) Baud 설정 (Key=0x12, Value=0x0C → 115200bps)
frame_set_baud = make_frame(0x12, 0x0C)

# 2) 데이터 포맷 설정: Byte format (Key=0x04, Value=0x01)
frame_set_fmt  = make_frame(0x04, 0x01)

# 3) 측정 시작 (Key=0x05, Value=0x00
frame_start    = make_frame(0x05, 0x00)

# 시리얼 포트 열기
ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(0.1)

# — 주파수 설정  
print(">> Set Frequency 100Hz:", frame_set_freq.hex())  
ser.write(frame_set_freq)  
time.sleep(0.05)  

# 고정 baud 모드 전환
print(">> 고정 모드(115200bps) 설정:", frame_set_baud.hex())
ser.write(frame_set_baud)
time.sleep(0.1)

# Byte 포맷 설정
print(">> Byte 포맷 설정:", frame_set_fmt.hex())
ser.write(frame_set_fmt)
time.sleep(0.1)

# 측정 시작
print(">> 측정 시작:", frame_start.hex())
ser.write(frame_start)
time.sleep(0.1)

print(">> Measurement Data Return(0x07) 수신 대기")

try:
    while True:
        # 헤더(0x55) 동기화
        if ser.read(1) != b'\x55':
            continue
        frame = ser.read(7)
        if len(frame) != 7 or frame[-1] != 0xAA:
            continue
        #print(frame)

        key   = frame[0]
        value = frame[1:5]
        crc_r = frame[5]
        #print(value)
        # CRC 확인
        if crc8(frame[0:5]) != crc_r:
            continue

        # Key=0x07 → 거리 데이터
        if key == 0x07:
            status   = value[0]
            dist_mm  = int.from_bytes(value[1:4], byteorder='big')
            if status == 0x00:
                if 50 <= dist_mm <= 40000:
                    print(f"Distance: {dist_mm} mm")
                else:
                    print("Out of range or no target")
            else:
                print(f"Error code: {status}")
except KeyboardInterrupt:
    pass
finally:
    ser.close()
    print(">> 종료, 포트 닫힘")
