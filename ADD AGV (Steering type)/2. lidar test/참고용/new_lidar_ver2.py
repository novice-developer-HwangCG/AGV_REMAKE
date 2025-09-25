#!/usr/bin/env python3
import serial
import time

"""
프로토콜 전달 방식
5byte, 6자리, signed 24bit big-endian, (hi, mid, lo)
(hi << 16) | (mid << 8) | lo: 파싱할 때 int.from_bytes([hi, mid, lo], 'big', signed=True)
예시: 123456 → [0x01, 0xE2, 0x40] (hex로)
값 범위 절대 거리 = 0~400,000mm / 상대 거리 = -400,000 ~ 400,00mm
"""

# LPB40B 설정
SERIAL_PORT = '/dev/ttyUSB2'
BAUD_RATE   = 115200

def crc8(ptr):
    crc = 0
    for b in ptr:
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

# 프레임 정의
FRAME_SET_FREQ = make_frame(0x03, 50)    # 측정 주파수 50Hz
FRAME_SET_BAUD = make_frame(0x12, 0x0C)  # 고정 모드 115200bps
FRAME_SET_FMT  = make_frame(0x04, 0x01)  # Byte format
FRAME_START    = make_frame(0x05, 0x00)  # 측정 시작
FRAME_STOP     = make_frame(0x06, 0x00)  # 측정 중지

class LiDAR:
    def __init__(self, port=SERIAL_PORT, baud=BAUD_RATE):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.baseline = None
        self.lidar_mode = 'absolute'  # 'absolute', 'relative', 'pause'

        # 초기화 시퀀스
        for frame in (FRAME_SET_FREQ, FRAME_SET_BAUD, FRAME_SET_FMT, FRAME_START):
            self.ser.write(frame)
            time.sleep(0.05)

    def clamp_signed_24(self, val):
        if val > 400_000:
            return 400_000
        elif val < -400_000:
            return -400_000
        return val

    def read_frame(self):
        # pause 모드일 때는 데이터 반환 안함
        self.ser.reset_input_buffer()
        if self.lidar_mode == 'pause':
            return None

        # 헤더(0x55) 동기화
        while True:
            b = self.ser.read(1)
            if not b:
                return None
            if b == b'\x55':
                break

        pkt = self.ser.read(7)
        if len(pkt) != 7 or pkt[-1] != 0xAA:
            return None

        key, status, D2, D1, D0, crc_r = pkt[0], pkt[1], pkt[2], pkt[3], pkt[4], pkt[5]
        # CRC 확인
        if crc8([key, status, D2, D1, D0]) != crc_r:
            return None

        # 원거리 데이터
        dist_raw = (D2 << 16) | (D1 << 8) | D0

        # relative 모드 보정
        if self.lidar_mode == 'relative' and self.baseline is not None:
            dist_raw = dist_raw - self.baseline

        # signed 24bit 범위 제한 (40m)
        dist_signed = self.clamp_signed_24(dist_raw)
        dist_bytes = dist_signed.to_bytes(3, byteorder='big', signed=True)
        hi, mid, lo = dist_bytes[0], dist_bytes[1], dist_bytes[2]

        # Protocol 반환: [0x5A, status, hi, mid, lo] -> ui 쪽 복원 방법 signed 16-bit 정수로 복원 [예시 dist_signed = int.from_bytes(bytes([hi, mid, lo]), byteorder='big', signed=True)]
        return bytes([0x5A, status, hi, mid, lo])

    def reset_baseline(self):
        # 현재 프레임 읽어서 baseline 설정
        raw = self.read_frame()
        if raw and len(raw) == 5:
            hi, mid, lo = raw[2], raw[3], raw[4]
            baseline_val = int.from_bytes(bytes([hi, mid, lo]), byteorder='big', signed=True)
            if raw[1] == 0:
                self.baseline = baseline_val
                self.lidar_mode = 'relative'
                print(f"[LIDAR] Baseline set → mode=relative, baseline={self.baseline}")

    def set_absolute_mode(self):
        self.lidar_mode = 'absolute'
        self.baseline = None
        self.ser.write(FRAME_START)
        #print("[LIDAR] Switched to absolute mode")

    def pause(self):
        self.lidar_mode = 'pause'
        self.ser.write(FRAME_STOP)
        #print("[LIDAR] Paused data transmission")

    def resume(self):
        self.lidar_mode = 'absolute'
        self.ser.write(FRAME_START)
        #print("[LIDAR] Resumed data transmission (absolute mode)")

    def close(self):
        self.ser.write(FRAME_STOP)
        self.ser.close()

if __name__ == '__main__':
    lidar = LiDAR()

    try:
        print("LiDAR initialized. Reading frames...")
        while True:
            frame = lidar.read_frame()
            if frame:
                print([hex(b) for b in frame])
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        lidar.close()
