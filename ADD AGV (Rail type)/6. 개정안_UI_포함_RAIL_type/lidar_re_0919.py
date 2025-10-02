import time

"""
프로토콜 전달 방식
5byte, 6자리, 2자리씩
hi, mid, lo: 각각 2자리, 파싱할 때 int(f"{hi:02d}{mid:02d}{lo:02d}")
예시 1: 123456 → [12, 34, 56] = 12.3456m
예시 2: 003456 → [00, 34, 56] = 0.3456m
예시 3: 000056 → [00, 00, 56] = 0.0056m
"""

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
FRAME_SET_FREQ  = make_frame(0x03, 50)     # (max 500Hz까지 일반 모드)
FRAME_SET_BAUD  = make_frame(0x12, 0x0C)   # 115200bps
FRAME_SET_FMT   = make_frame(0x04, 0x01)   # Byte format
FRAME_START     = make_frame(0x05, 0x00)   # Start
FRAME_STOP      = make_frame(0x06, 0x00)   # Stop

class LiDAR:
    def __init__(self, uart):
        self.uart = uart
        self.baseline = None
        self.send_frame = None  
        self._last_raw_mm  = None   # 직전 측정된 절대 거리 (mm)
        self.rel_mm = None
        self.lidar_mode = 'absolute'  # 'absolute', 'relative'
        self.rel_mm_last = 0

        # 초기화 시퀀스
        for set_frame in (FRAME_SET_FREQ, FRAME_SET_BAUD, FRAME_SET_FMT, FRAME_START):
            self.uart.write(set_frame)
            time.sleep(0.1)

    def read_frame(self):
        # pause 모드일 때는 데이터 반환 안함
        self.uart.reset_input_buffer()
        if self.lidar_mode == 'pause':
            return None

        # 헤더(0x55) 동기화
        start = time.monotonic()
        while True:
            b = self.uart.read(1)
            if b and b[0] == 0x55:
                break
            if time.monotonic() - start > 0.2:   # 200ms 타임아웃
                return None

        pkt = self.uart.read(7)
        if not pkt or len(pkt) < 7 or pkt[-1] != 0xAA:
            return None

        key, status, D2, D1, D0, crc_r = pkt[0], pkt[1], pkt[2], pkt[3], pkt[4], pkt[5]
        # CRC 확인
        if crc8([key, status, D2, D1, D0]) != crc_r:
            return None

        # 원거리 데이터(24bit)
        dist_raw = (D2 << 16) | (D1 << 8) | D0
        self._last_raw_mm = dist_raw

        # relative 모드 보정
        if self.lidar_mode == 'relative' and self.baseline is not None:
            self.rel_mm = dist_raw - self.baseline
        else:
            self.rel_mm = dist_raw

        self.rel_mm_last = self.rel_mm  # ← 최신 상대값(절대모드면 절대값) 저장
        sign_pm = 0

        # 400,000mm 이상은 max값으로 제한 (6자리)
        if self.rel_mm > 400_000:
            self.rel_mm = 400_000

        if self.rel_mm > 0:
            sign_pm = 0
        else:
            sign_pm = 1

        # 각 두자리씩 분리: [앞2자리, 중간2자리, 뒤2자리]
        # 예: 271,144 → 27, 11, 44
        abs_dist = abs(self.rel_mm)
        raw_units = abs_dist * 10
        dist_str = f"{raw_units:06d}"
        hi = int(dist_str[0:2])
        mid = int(dist_str[2:4])
        lo = int(dist_str[4:6])

        self.send_frame = bytes([0x5A, status, hi, mid, lo, sign_pm])

        # 프로토콜 반환: [0x5A, status, hi, mid, lo]
        return self.send_frame

    def reset_baseline(self):
        """
        반드시 '절대 프레임 1개 확보' → baseline 세팅 → 상대 모드 진입.
        """
        # 1) 절대 모드로 전환
        self.set_absolute_mode()

        # 2) 최대 200ms 동안 절대 프레임 1개 확보 시도
        t0 = time.monotonic()
        while (self._last_raw_mm is None) and (time.monotonic() - t0 < 0.2):
            self.read_frame()  # 한 프레임 읽혀야 _last_raw_mm 갱신됨

        if self._last_raw_mm is not None:
            # 3) baseline 기록 + 상대 모드
            self.baseline = self._last_raw_mm
            self.lidar_mode = 'relative'
            self.rel_mm_last = 0  # 영점 시점이므로 0으로 초기화
        # else: 절대 프레임 확보 실패 → baseline 미설정 (필요시 호출측에서 재시도 처리)

        # # 현재 프레임 읽어서 baseline 설정
        # if self._last_raw_mm is not None:
        #     self.baseline   = self._last_raw_mm
        #     self.lidar_mode = 'relative'
        #     # print(f"[LIDAR] Baseline set → mode=relative, baseline={self.baseline}")

    def set_absolute_mode(self):
        self.lidar_mode = 'absolute'
        self.baseline = None
        self.uart.write(FRAME_START)

    def pause(self):
        self.lidar_mode = 'pause'
        self.uart.write(FRAME_STOP)

    def close(self):
        self.uart.write(FRAME_STOP)
        try:
            self.uart.deinit()
        except AttributeError:
            pass
