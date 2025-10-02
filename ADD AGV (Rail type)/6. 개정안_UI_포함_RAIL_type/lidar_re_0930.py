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
        self.stream_on = False     # START/STOP 상태
        self.buf = bytearray()

        # 초기화 시퀀스
        for set_frame in (FRAME_SET_FREQ, FRAME_SET_BAUD, FRAME_SET_FMT):
            self.uart.write(set_frame)
            time.sleep(0.1)

    def read_frame(self):
        # 누적 읽기
        n = getattr(self.uart, "in_waiting", 0) or 0
        if n:
            chunk = self.uart.read(n)
            if chunk:
                self.buf.extend(chunk)

        latest = None

        # 헤더 동기 + 꼬리 확인(총 8바이트: 0x55 + 6바이트 + 0xAA)
        while True:
            i = self.buf.find(b'\x55')
            if i < 0:
                # 헤더 없으면 버퍼 과도 증가 방지
                if len(self.buf) > 64:
                    self.buf = self.buf[-1:]
                break

            # 헤더 앞의 쓰레기 제거
            if i > 0:
                self.buf = self.buf[i:]

            if len(self.buf) < 8:
                # 프레임이 아직 다 안 들어왔음
                break

            frame = self.buf[i:i+8]
            if frame[-1] != 0xAA:
                # 잘못된 패턴 → 헤더 1바이트만 버리고 재동기
                self.buf[:] = self.buf[i+1:]
                continue

            key, status, D2, D1, D0, crc_r = frame[1], frame[2], frame[3], frame[4], frame[5], frame[6]
            if crc8(bytes([key, status, D2, D1, D0])) != crc_r:
                # CRC 불일치 → 이 헤더 버리고 재시도
                self.buf[:] = self.buf[i+1:]
                continue

            # 유효 프레임 소비
            self.buf[:] = self.buf[i+8:]

            # 원거리 데이터(24bit)
            dist_raw = (D2 << 16) | (D1 << 8) | D0
            self._last_raw_mm = dist_raw

            # relative 모드 보정
            if self.lidar_mode == 'relative' and self.baseline is not None:
                self.rel_mm = dist_raw - self.baseline
            else:
                self.rel_mm = dist_raw

            if self.lidar_mode == 'relative':
                self.rel_mm_last = self.rel_mm  # ← 최신 상대값(절대모드면 절대값) 저장

            # 400,000mm 이상은 max값으로 제한 (6자리)
            # 표준화/포맷
            val = self.rel_mm
            if val > 400_000:
                val = 400_000
            sign_pm = 0 if val >= 0 else 1
            raw_units = abs(val) * 10  # mm -> 1e-4 m 자리수 6자리
            s = f"{raw_units:06d}"
            hi = int(s[0:2])
            mid= int(s[2:4])
            lo = int(s[4:6])

            latest = bytes([0x5A, status, hi, mid, lo, sign_pm])
            # 계속 루프를 돌며 더 새로운 프레임이 있으면 갱신해서 'latest'를 업데이트

        return latest  # 없으면 None, 있으면 '가장 최신' 프레임

    def reset_baseline(self):
        # 1) 절대 모드로 전환
        self.set_absolute_mode()

        # ★ 스트리밍 보장 (일시적으로 켰다가, 이전 상태로 복원)
        was_on = self.stream_on
        if not was_on:
            self.start_stream()

        # 2) 최대 200ms 동안 절대 프레임 1개 확보 시도
        t0 = time.monotonic()
        while (self._last_raw_mm is None) and (time.monotonic() - t0 < 0.2):
            self.read_frame()

        if self._last_raw_mm is not None:
            self.baseline = self._last_raw_mm
            self.lidar_mode = 'relative'
            self.rel_mm_last = 0

        # 스트리밍 원복
        if not was_on:
            self.stop_stream()

        # # 현재 프레임 읽어서 baseline 설정
        # if self._last_raw_mm is not None:
        #     self.baseline   = self._last_raw_mm
        #     self.lidar_mode = 'relative'
        #     # print(f"[LIDAR] Baseline set → mode=relative, baseline={self.baseline}")

    def start_stream(self):
        if not self.stream_on:
            self.uart.write(FRAME_START)
            time.sleep(0.05)
            try:
                self.uart.reset_input_buffer()  # 전이 시점 1회 flush
            except Exception:
                pass
            self.buf = bytearray()
            self.stream_on = True

    def stop_stream(self):
        if self.stream_on:
            self.uart.write(FRAME_STOP)
            time.sleep(0.02)
            self.stream_on = False

    def set_absolute_mode(self):
        self.lidar_mode = 'absolute'
        self.baseline = None
        try:
            self.uart.reset_input_buffer()  # 전이 시점 1회 flush
        except Exception:
            pass
        self.buf = bytearray()

    def close(self):
        self.uart.write(FRAME_STOP)
        try:
            self.uart.deinit()
        except AttributeError:
            pass
