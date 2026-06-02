import time

"""
LiDAR module - CircuitPython/MicroPython compatible UART object version

프로토콜 전달 방식
9byte, 6자리, 2자리씩
[ 0x5A, status, abs_hi, abs_mid, abs_lo, rel_hi, rel_mid, rel_lo, sign_pm ]
  - status: 센서 상태(원 프레임의 status 바이트 그대로 전달)
  - abs_* : 절대 거리값을 1e-4 m 단위 6자리로 두 자리씩 분할
            예: 123456 -> 12,34,56 -> 12.3456 m
  - rel_* : 상대 거리값을 1e-4 m 단위 6자리로 두 자리씩 분할
  - sign_pm: 상대 거리 부호(0=양/0, 1=음) — 부호 해석은 UI에서 수행
  - 동작:
    - lidar on(set_absolute_mode, start_stream): baseline=None -> 상대=0으로 시작
    - lidar reset(reset_baseline): 현재 절대값을 baseline으로 설정 -> 이후 상대=절대-baseline
    - lidar off(stop_stream): 전송 중지

중요:
  - 6자리 프로토콜에서 mm 값을 1e-4 m 단위로 보내기 위해 mm * 10을 사용
  - 따라서 40,000 mm -> 400000 -> 40.0000 m 까지는 6자리로 정상 표현 가능
  - 400,000 mm를 허용하면 4,000,000으로 7자리가 되어 기존 3바이트 포맷이 깨짐
"""

# ======== Constants ========
LIDAR_FRAME_HEADER = 0x55
LIDAR_FRAME_TAIL   = 0xAA
LIDAR_DATA_KEY     = 0x07

TX_FRAME_HEADER    = 0x5A

# 센서/프로토콜 유효 범위
MIN_DISTANCE_MM    = 0
MAX_DISTANCE_MM    = 40_000      # 40 m. 6자리 포맷(mm*10) 유지 목적
MAX_RELATIVE_MM    = 40_000      # +/- 40 m
UART_BUF_LIMIT     = 128
BASELINE_TIMEOUT_MS = 250


# ======== Utility ========
def _sleep_ms(ms: int):
    try:
        time.sleep_ms(ms)
    except AttributeError:
        time.sleep(ms / 1000.0)


def _ticks_ms():
    try:
        return time.ticks_ms()
    except AttributeError:
        return int(time.monotonic() * 1000)


def _ticks_diff(now, start):
    try:
        return time.ticks_diff(now, start)
    except AttributeError:
        return now - start


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
    value_bytes = int(value).to_bytes(4, 'big')   # e.g. 50 -> b'\x00\x00\x00\x32'
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

        self.abs_val = 0          # 절대 거리(mm)
        self.rel_val = 0          # 상대 거리(mm)
        self.rel_mm_last = 0      # 기존 code.py 호환용: 원위치 이동 로직에서 사용 가능

        self._last_raw_val = None # 직전 측정된 절대 거리(mm)
        self._last_raw_mm = None  # 기존 이름 호환용

        self.lidar_mode = 'absolute'  # 'absolute', 'relative'
        self.stream_on = False        # START/STOP 상태
        self.buf = bytearray()

        # 초기화 시퀀스
        for set_frame in (FRAME_SET_FREQ, FRAME_SET_BAUD, FRAME_SET_FMT):
            self.uart.write(set_frame)
            _sleep_ms(100)

    # ---------- UART helpers ----------
    def _available(self) -> int:
        """CircuitPython(in_waiting) / MicroPython(any) 호환"""
        try:
            return int(getattr(self.uart, "in_waiting"))
        except Exception:
            pass

        try:
            return int(self.uart.any())
        except Exception:
            return 0

    def _flush_uart(self):
        """전이 시점의 오래된 UART 데이터를 버림"""
        try:
            self.uart.reset_input_buffer()
        except Exception:
            while True:
                n = self._available()
                if n <= 0:
                    break
                self.uart.read(n)
                _sleep_ms(1)
        self.buf = bytearray()

    # ---------- RX parser ----------
    def _read_latest_sensor(self):
        """
        UART 버퍼에 쌓인 프레임을 가능한 한 모두 처리, 가장 최신의 유효한 key=0x07 거리 프레임만 반환
        반환: (status, dist_raw) 또는 None
        """
        n = self._available()
        if n:
            chunk = self.uart.read(n)
            if chunk:
                self.buf.extend(chunk)

        latest = None

        while True:
            i = self.buf.find(bytes([LIDAR_FRAME_HEADER]))
            if i < 0:
                # 헤더가 없다면 쓰레기 데이터이므로 과도 증가 방지
                if len(self.buf) > UART_BUF_LIMIT:
                    self.buf = bytearray()
                break

            # 헤더 앞의 쓰레기 제거
            if i > 0:
                self.buf = self.buf[i:]

            if len(self.buf) < 8:
                # 프레임이 아직 다 안 들어왔음
                break

            frame = self.buf[:8]

            if frame[7] != LIDAR_FRAME_TAIL:
                # 잘못된 패턴 -> 헤더 1바이트만 버리고 재동기
                self.buf[:] = self.buf[1:]
                continue

            key, status, D2, D1, D0, crc_r = frame[1], frame[2], frame[3], frame[4], frame[5], frame[6]
            if crc8(bytes([key, status, D2, D1, D0])) != crc_r:
                # CRC 불일치 -> 이 헤더 버리고 재시도
                self.buf[:] = self.buf[1:]
                continue

            # 유효 프레임 소비
            self.buf[:] = self.buf[8:]

            # 거리 데이터 프레임만 사용
            if key != LIDAR_DATA_KEY:
                continue

            dist_raw = (D2 << 16) | (D1 << 8) | D0
            latest = (status, dist_raw)
            # 계속 루프를 돌며 버퍼 안의 더 최신 프레임이 있으면 latest를 갱신

        return latest

    # 기존 이름 호환용
    def _read_sensor_once(self):
        return self._read_latest_sensor()

    # ---------- TX frame builder ----------
    def _clamp_abs_mm(self, value: int) -> int:
        return max(MIN_DISTANCE_MM, min(MAX_DISTANCE_MM, int(value)))

    def _clamp_rel_mm(self, value: int) -> int:
        return max(-MAX_RELATIVE_MM, min(MAX_RELATIVE_MM, int(value)))

    def _pack_mm_to_3bytes(self, mm_value: int):
        """
        mm 값을 1e-4 m 단위 6자리로 변환 후 두 자리씩 분할
        예: 1234 mm -> 12340 -> 01,23,40 -> 1.2340 m
        """
        raw_units = abs(int(mm_value)) * 10
        if raw_units > 999_999:
            raw_units = 999_999
        s = f"{raw_units:06d}"
        return int(s[0:2]), int(s[2:4]), int(s[4:6])

    def _make_tx_frame(self, status: int) -> bytes:
        abs_hi, abs_mid, abs_lo = self._pack_mm_to_3bytes(self.abs_val)
        rel_hi, rel_mid, rel_lo = self._pack_mm_to_3bytes(self.rel_val)
        sign_pm = 0 if self.rel_val >= 0 else 1
        return bytes([
            TX_FRAME_HEADER,
            status & 0xFF,
            abs_hi,
            abs_mid,
            abs_lo,
            rel_hi,
            rel_mid,
            rel_lo,
            sign_pm,
        ])

    def read_frame(self):
        # LIDAR 센서 데이터 보내기
        sensor = self._read_latest_sensor()
        if sensor is None:
            return None

        status, dist_raw = sensor

        self.abs_val = self._clamp_abs_mm(dist_raw)
        self._last_raw_val = self.abs_val
        self._last_raw_mm = self.abs_val

        # relative 모드 보정
        if self.lidar_mode == 'relative' and (self.baseline is not None):
            self.rel_val = self._clamp_rel_mm(self.abs_val - self.baseline)
        else:
            # relative 모드가 아닐 시 0 값 보내기
            self.rel_val = 0

        self.rel_mm_last = self.rel_val

        return self._make_tx_frame(status)

    def reset_baseline(self):
        """
        현재 절대 거리값을 baseline으로 설정
        기존 코드와 달리 _last_raw_val이 이미 있어도 새 프레임을 확보해서 baseline으로 사용
        반환: baseline 설정 성공 여부 True/False
        """
        was_on = self.stream_on
        if not was_on:
            self.start_stream()

        # 오래된 버퍼가 baseline으로 잡히는 것을 방지
        self._flush_uart()

        latest = None
        t0 = _ticks_ms()
        while _ticks_diff(_ticks_ms(), t0) < BASELINE_TIMEOUT_MS:
            sensor = self._read_latest_sensor()
            if sensor is not None:
                latest = sensor
                break
            _sleep_ms(2)

        if latest is not None:
            status, dist_raw = latest
            self.abs_val = self._clamp_abs_mm(dist_raw)
            self._last_raw_val = self.abs_val
            self._last_raw_mm = self.abs_val
            self.baseline = int(self.abs_val)
            self.rel_val = 0
            self.rel_mm_last = 0
            self.lidar_mode = 'relative'
            ok = True
        else:
            # 새 프레임 확보 실패 시 이전 값이 있으면 fallback으로 사용
            if self._last_raw_val is not None:
                self.baseline = int(self._last_raw_val)
                self.rel_val = 0
                self.rel_mm_last = 0
                self.lidar_mode = 'relative'
                ok = True
            else:
                ok = False

        # 스트리밍 원복
        if not was_on:
            self.stop_stream()

        return ok

    def start_stream(self):
        if not self.stream_on:
            self.uart.write(FRAME_START)
            _sleep_ms(50)
            self._flush_uart()  # 전이 시점 1회 flush
            self.stream_on = True

    def stop_stream(self):
        if self.stream_on:
            self.uart.write(FRAME_STOP)
            _sleep_ms(20)
            self.stream_on = False
            self.buf = bytearray()

    def set_absolute_mode(self):
        self.lidar_mode = 'absolute'
        self.baseline = None
        self.rel_val = 0
        self.rel_mm_last = 0
        self._flush_uart()  # 전이 시점 1회 flush

    def close(self):
        self.uart.write(FRAME_STOP)
        try:
            self.uart.deinit()
        except AttributeError:
            pass
