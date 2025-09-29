#!/usr/bin/env python3
# lidar.py
# from __future__ import annotations

import serial
import time
from typing import Optional, Tuple

# LPB40B 기본 설정
SERIAL_PORT = "/dev/ttyLiDAR"
BAUD_RATE = 115200

"""
출력(클라이언트/UI로 전송하는) 프레임 형식: 6바이트
[ 0x5A, status, hi, mid, lo, sign ]
  - status: 센서 상태(원 프레임의 status 바이트 그대로 전달)
  - hi,mid,lo: 거리의 6자리 10진수를 2자리씩 분할(예: 123456 → hi=12, mid=34, lo=56)
  - sign: 0(+) / 1(-)  — 부호 해석은 UI에서 수행
"""

# ── 디바이스 명령 프레임 ────────────────────────────────────────────────────
def crc8(ptr):
    crc = 0
    for b in ptr:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if (crc & 0x80) else ((crc << 1) & 0xFF)
    return crc

def make_frame(key: int, value_byte: int) -> bytes:
    # [0x55][Key][0x00][0x00][0x00][Value][CRC8][0xAA]
    buf = bytes([key, 0x00, 0x00, 0x00, value_byte])
    c = crc8(buf)
    return b"\x55" + buf + bytes([c, 0xAA])

FRAME_SET_FREQ = make_frame(0x03, 50)    # 50Hz
FRAME_SET_BAUD = make_frame(0x12, 0x0C)  # 115200bps
FRAME_SET_FMT  = make_frame(0x04, 0x01)  # byte format
FRAME_START    = make_frame(0x05, 0x00)  # start
FRAME_STOP     = make_frame(0x06, 0x00)  # stop

# ── LiDAR 클래스 ─────────────────────────────────────────────────────────────
class LiDAR:
    def __init__(self, port: str = SERIAL_PORT, baud: int = BAUD_RATE, timeout: float = 0.1):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.baseline: Optional[int] = None  # 원시 센서 단위(24비트 정수, mm 등 장치 단위)
        self.lidar_mode = "absolute"         # 'absolute' | 'relative' | 'pause'

        # 초기화 시퀀스
        for frame in (FRAME_SET_FREQ, FRAME_SET_BAUD, FRAME_SET_FMT, FRAME_START):
            self.ser.write(frame)
            time.sleep(0.05)

    # 내부: 원시 센서 프레임 1개 읽기 → (status, dist_raw) 반환
    def _read_sensor_once(self) -> Optional[Tuple[int, int]]:
        # 헤더 동기화(0x55)
        while True:
            b = self.ser.read(1)
            if not b:
                return None
            if b == b"\x55":
                break

        # 이후 7바이트: [key, status, D2, D1, D0, crc, 0xAA]
        pkt = self.ser.read(7)
        if len(pkt) != 7 or pkt[-1] != 0xAA:
            return None

        key, status, D2, D1, D0, crc_r = pkt[0], pkt[1], pkt[2], pkt[3], pkt[4], pkt[5]
        if crc8([key, status, D2, D1, D0]) != crc_r:
            return None

        dist_raw = (D2 << 16) | (D1 << 8) | D0  # 원시 거리값(양수)
        return status, dist_raw

    @staticmethod
    def _to_ui_frame(status: int, value_signed: int) -> bytes:
        """
        내부 정수값(value_signed)을 6자리 10진수(절대값) + sign(0/1)으로 패킹.
        - 절대값은 0..999999로 클램프
        - sign: value_signed >= 0 → 0,  value_signed < 0 → 1
        """
        sign_pm = 0 if value_signed >= 0 else 1
        abs_val = abs(int(value_signed))

        # 6자리로 제한
        abs_val = max(0, min(999_999, abs_val))
        raw_dist = abs(abs_val) * 10
        dist_str = f"{raw_dist:06d}"
        hi  = int(dist_str[0:2])
        mid = int(dist_str[2:4])
        lo  = int(dist_str[4:6])
        return bytes([0x5A, status & 0xFF, hi, mid, lo, sign_pm])

    def read_frame(self) -> Optional[bytes]:
        """
        pause 모드: None 반환(송출 안 함)
        absolute/relative 모드: 6바이트 UI 프레임 반환
        """
        # 최신 프레임을 읽기 위해 입력 버퍼 비우기
        self.ser.reset_input_buffer()

        if self.lidar_mode == "pause":
            return None

        sensor = self._read_sensor_once()
        if sensor is None:
            return None

        status, dist_raw = sensor

        # relative 모드 보정(부호 가능)
        if self.lidar_mode == "relative" and self.baseline is not None:
            value_signed = dist_raw - self.baseline
        else:
            value_signed = dist_raw  # absolute는 항상 +
        value_signed = max(-400_000, min(400_000, value_signed))    # 40m 범위제한

        return self._to_ui_frame(status, value_signed)

    def reset_baseline(self):
        """
        현재 'absolute' 측정값을 원시 센서 프레임에서 직접 읽어 baseline으로 설정.
        이후 모드를 'relative'로 전환.
        """
        sensor = self._read_sensor_once()
        if sensor is None:
            return
        status, dist_raw = sensor
        # status==0 일 때만 유효로 볼지 여부는 장치 스펙에 맞춰 조정
        if status == 0:
            self.baseline = dist_raw
            self.lidar_mode = "relative"
            print(f"[LIDAR] Baseline set → mode=relative, baseline={self.baseline}")

    def set_absolute_mode(self):
        self.lidar_mode = "absolute"
        self.baseline = None
        try:
            self.ser.write(FRAME_START)
        except Exception:
            pass

    def pause(self):
        self.lidar_mode = "pause"
        try:
            self.ser.write(FRAME_STOP)
        except Exception:
            pass

    def resume(self):
        # 서버 요구사항: 재개 시 'absolute'로
        self.lidar_mode = "absolute"
        try:
            self.ser.write(FRAME_START)
        except Exception:
            pass

    def close(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(FRAME_STOP)
                self.ser.close()
        except Exception as e:
            print(f"[LIDAR] Error closing serial: {e}")


if __name__ == "__main__":
    lidar = LiDAR()
    try:
        print("LiDAR initialized. Reading frames...")
        while True:
            frame = lidar.read_frame()
            # if frame:
            #     # 예시 출력
            #     print([hex(b) if i < 2 else b for i, b in enumerate(frame)])
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        lidar.close()