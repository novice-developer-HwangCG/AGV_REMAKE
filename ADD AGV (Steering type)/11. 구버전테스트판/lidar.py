#!/usr/bin/env python3
import serial
import time
from typing import Optional, Tuple

"""
UI로 보내는 프레임: 6바이트
[0] 0x5A (헤더)
[1] status (센서 status)
[2] hi  (6자리 10진수의 앞 두 자리)
[3] mid (가운데 두 자리)
[4] lo  (뒤 두 자리)
[5] sign (0=양수, 1=음수)
"""

SERIAL_PORT = "/dev/ttyLiDAR"
BAUD_RATE   = 115200

def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if (crc & 0x80) else ((crc << 1) & 0xFF)
    return crc

def make_frame(key: int, value_byte: int) -> bytes:
    """
    센서 제어용 8바이트 명령:
    [0x55][Key][0x00][0x00][0x00][Value][CRC8][0xAA]
    """
    body = bytes([key, 0x00, 0x00, 0x00, value_byte])
    c = crc8(body)
    return b"\x55" + body + bytes([c, 0xAA])

# 센서 초기화 명령
FRAME_SET_FREQ = make_frame(0x03, 50)    # 50Hz
FRAME_SET_BAUD = make_frame(0x12, 0x0C)  # 115200bps
FRAME_SET_FMT  = make_frame(0x04, 0x01)  # Byte format
FRAME_START    = make_frame(0x05, 0x00)  # 측정 시작
FRAME_STOP     = make_frame(0x06, 0x00)  # 측정 중지

class LiDAR:
    def __init__(self, port: str = SERIAL_PORT, baud: int = BAUD_RATE):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.baseline: Optional[int] = None     # mm 단위 절대값
        self.lidar_mode: str = "absolute"       # 'absolute' / 'relative' / 'pause'

        # 초기화 시퀀스
        for frame in (FRAME_SET_FREQ, FRAME_SET_BAUD, FRAME_SET_FMT, FRAME_START):
            self.ser.write(frame)
            time.sleep(0.05)

    # ───────────────────────── 내부 헬퍼 ─────────────────────────
    def _read_sensor_once(self) -> Optional[Tuple[int, int]]:
        """
        센서 원시 프레임을 1개 읽어 (status, dist_raw_mm) 반환.
        dist_raw_mm: 0..400_000 (절대)
        """
        # 헤더 동기화 (0x55)
        while True:
            b = self.ser.read(1)
            if not b:
                return None
            if b == b"\x55":
                break

        # 나머지 7바이트
        pkt = self.ser.read(7)
        if len(pkt) != 7 or pkt[-1] != 0xAA:
            return None

        key, status, D2, D1, D0, crc_r = pkt[0], pkt[1], pkt[2], pkt[3], pkt[4], pkt[5]
        # CRC 체크
        if crc8(bytes([key, status, D2, D1, D0])) != crc_r:
            return None

        dist_raw = (D2 << 16) | (D1 << 8) | D0
        # 40m 클램프
        if dist_raw > 400_000:
            dist_raw = 400_000
        return status, dist_raw

    @staticmethod
    def _to_ui_frame(status: int, value_signed: int) -> bytes:
        """
        내부 정수값(value_signed)을 6자리 10진수(절대값) + sign(0/1)으로 패킹.
        sign: value_signed >= 0 → 0,  value_signed < 0 → 1
        """
        sign_pm = 0 if value_signed >= 0 else 1
        abs_val = abs(int(value_signed))
        abs_val = max(0, min(999_999, abs_val))
        s = f"{abs_val:06d}"
        hi, mid, lo = int(s[0:2]), int(s[2:4]), int(s[4:6])
        return bytes([0x5A, status & 0xFF, hi, mid, lo, sign_pm])

    # ───────────────────────── 공개 API ─────────────────────────
    def read_frame(self) -> Optional[bytes]:
        """pause 모드면 None. absolute/relative 모드면 6바이트 UI 프레임 반환."""
        if self.lidar_mode == "pause":
            return None

        sensor = self._read_sensor_once()
        if sensor is None:
            return None

        status, dist_abs = sensor
        value = dist_abs
        if self.lidar_mode == "relative" and self.baseline is not None:
            value = dist_abs - self.baseline

        # -40m ~ +40m 제한
        value = max(-400_000, min(400_000, value))
        return self._to_ui_frame(status, value)

    def reset_baseline(self, samples: int = 3) -> bool:
        """
        현재 절대 거리의 평균을 기준(baseline)으로 잡고 relative 모드로 전환.
        UI에서 '영점' 요청 시 서버가 이 함수를 호출.
        """
        vals = []
        for _ in range(samples):
            sensor = self._read_sensor_once()
            if sensor is None:
                continue
            _, dist_abs = sensor
            vals.append(dist_abs)
            time.sleep(0.02)
        if not vals:
            print("[LIDAR] Baseline set failed: no data")
            return False

        self.baseline = int(sum(vals) / len(vals))
        self.lidar_mode = "relative"
        # print(f"[LIDAR] Baseline set → mode=relative, baseline={self.baseline} mm")
        return True

    def set_absolute_mode(self):
        self.lidar_mode = "absolute"
        self.baseline = None
        # 스트리밍은 변경하지 않음

    def pause(self):
        """스트리밍만 중지(모드는 유지)."""
        self.ser.write(FRAME_STOP)
        self.lidar_mode = "pause"

    def resume(self):
        """스트리밍 재개(모드는 유지)."""
        # relative 모드로 이미 설정돼 있으면 그대로 유지
        self.lidar_mode = "absolute"
        # if self.lidar_mode == "pause":
        #     self.lidar_mode = "relative" if self.baseline is not None else "absolute"
        self.ser.write(FRAME_START)

    def close(self):
        try:
            if self.ser.is_open:
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
            if frame:
                print([hex(b) for b in frame])
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        lidar.close()


# #!/usr/bin/env python3
# import serial
# import time

# """
# 프로토콜 전달 방식
# 5byte, 6자리, x 엔디방 방식 아님
# 예시: 123456 → [0x01, 0xE2, 0x40] (hex로)
# 값 범위 절대 거리 = 0~400,000mm / 음수값 표현 안됨
# """

# # LPB40B 설정
# SERIAL_PORT = '/dev/ttyLiDAR'
# BAUD_RATE   = 115200

# def crc8(ptr):
#     crc = 0
#     for b in ptr:
#         crc ^= b
#         for _ in range(8):
#             crc = (crc << 1) ^ 0x31 if (crc & 0x80) else (crc << 1)
#             crc &= 0xFF
#     return crc

# def make_frame(key: int, value_byte: int) -> bytes:
#     """
#     8-byte 프로토콜 프레임 생성
#     [0x55][Key][0x00][0x00][0x00][Value][CRC8][0xAA]
#     """
#     buf = bytes([key, 0x00, 0x00, 0x00, value_byte])
#     c   = crc8(buf)
#     return b'\x55' + buf + bytes([c, 0xAA])

# # 프레임 정의
# FRAME_SET_FREQ = make_frame(0x03, 50)    # 측정 주파수 50Hz
# FRAME_SET_BAUD = make_frame(0x12, 0x0C)  # 고정 모드 115200bps
# FRAME_SET_FMT  = make_frame(0x04, 0x01)  # Byte format
# FRAME_START    = make_frame(0x05, 0x00)  # 측정 시작
# FRAME_STOP     = make_frame(0x06, 0x00)  # 측정 중지

# class LiDAR:
#     def __init__(self, port=SERIAL_PORT, baud=BAUD_RATE):
#         self.ser = serial.Serial(port, baud, timeout=0.1)
#         self.baseline = None
#         self.lidar_mode = 'absolute'  # 'absolute', 'relative', 'pause'

#         # 초기화 시퀀스
#         for frame in (FRAME_SET_FREQ, FRAME_SET_BAUD, FRAME_SET_FMT, FRAME_START):
#             self.ser.write(frame)
#             time.sleep(0.05)

#     def clamp_signed_24(self, val):
#         if val > 400_000:
#             return 400_000
#         elif val < -400_000:
#             return -400_000
#         return val

#     def read_frame(self):
#         # pause 모드일 때는 데이터 반환 안함
#         self.ser.reset_input_buffer()
#         if self.lidar_mode == 'pause':
#             return None

#         # 헤더(0x55) 동기화
#         while True:
#             b = self.ser.read(1)
#             if not b:
#                 return None
#             if b == b'\x55':
#                 break

#         pkt = self.ser.read(7)
#         if len(pkt) != 7 or pkt[-1] != 0xAA:
#             return None

#         key, status, D2, D1, D0, crc_r = pkt[0], pkt[1], pkt[2], pkt[3], pkt[4], pkt[5]
#         # CRC 확인
#         if crc8([key, status, D2, D1, D0]) != crc_r:
#             return None

#         # 원거리 데이터
#         dist_raw = (D2 << 16) | (D1 << 8) | D0

#         # relative 모드 보정
#         if self.lidar_mode == 'relative' and self.baseline is not None:
#             dist_raw = dist_raw - self.baseline

#         if dist_raw > 400_000:
#             dist_raw = 400_000
        
#         sign_pm = 0
        
#         if dist_raw > 0:
#             sign_pm = 0
#         else:
#             sign_pm = 1

#         # signed 24bit 범위 제한 (40m)
#         #dist_signed = self.clamp_signed_24(dist_raw)
#         #dist_bytes = dist_signed.to_bytes(3, byteorder='big', signed=True)
#         abs_dist = abs(dist_raw) # * 10
#         dist_str = f"{abs_dist:06d}"
#         # dist_str = f"{dist_raw:+06d}" if dist_raw <0 else f"{dist_raw:+06d}"
#         hi=int(dist_str[0:2])
#         mid=int(dist_str[2:4])
#         lo=int(dist_str[4:6])
#         # print(hi)
#         # print(mid)
#         # print(lo)
#         #hi, mid, lo = dist_bytes[0], dist_bytes[1], dist_bytes[2]
#         return bytes([0x5A, status, hi, mid, lo, sign_pm])

#     def reset_baseline(self):
#         # 현재 프레임 읽어서 baseline 설정
#         raw = self.read_frame()
#         if raw and len(raw) == 5:
#             hi, mid, lo = raw[2], raw[3], raw[4]
#             baseline_val = int.from_bytes(bytes([hi, mid, lo]), byteorder='big', signed=True)
#             if raw[1] == 0:
#                 self.baseline = baseline_val
#                 self.lidar_mode = 'relative'
#                 print(f"[LIDAR] Baseline set → mode=relative, baseline={self.baseline}")

#     def set_absolute_mode(self):
#         self.lidar_mode = 'absolute'
#         self.baseline = None
#         self.ser.write(FRAME_START)
#         #print("[LIDAR] Switched to absolute mode")

#     def pause(self):
#         self.lidar_mode = 'pause'
#         self.ser.write(FRAME_STOP)
#         #print("[LIDAR] Paused data transmission")

#     def resume(self):
#         self.lidar_mode = 'absolute'
#         self.ser.write(FRAME_START)
#         #print("[LIDAR] Resumed data transmission (absolute mode)")

#     def close(self):
#         try:
#             if self.ser.is_open:
#                 self.ser.write(FRAME_STOP)
#                 self.ser.close()
#         except Exception as e:
#             print(f"[LIDAR] Error closing serial: {e}")

# if __name__ == '__main__':
#     lidar = LiDAR()

#     try:
#         print("LiDAR initialized. Reading frames...")
#         while True:
#             frame = lidar.read_frame()
#             if frame:
#                 print([hex(b) for b in frame])
#             time.sleep(1)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         lidar.close()
