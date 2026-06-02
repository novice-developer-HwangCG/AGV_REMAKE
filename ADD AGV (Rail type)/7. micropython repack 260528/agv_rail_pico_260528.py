import time
import math
import gc

try:
    import errno
except ImportError:
    errno = None

try:
    import socket
except ImportError:
    try:
        import usocket as socket
    except ImportError:
        socket = None

try:
    import network
except ImportError:
    network = None

try:
    import micropython
    micropython.alloc_emergency_exception_buf(100)
except Exception:
    pass

from machine import Pin, PWM, ADC, UART, SPI
from lidar import LiDAR

# ======== Constants ========
# Network
MY_MAC = b'\x00\x01\x02\x03\x04\x05'
IP_ADDRESS = '192.168.100.2'     # 변경: 10 → 2
SUBNET_MASK = '255.255.255.0'
GATEWAY_ADDRESS = '192.168.100.1'
DNS_SERVER = '8.8.8.8'
SERVER_PORT = 5024

# W5500 pins - 기존 CircuitPython code.py 기준: SPI0 SCK=GP6, MOSI=GP7, MISO=GP4, CS=GP5
W5500_SPI_ID = 0
W5500_SCK_PIN = 6
W5500_MOSI_PIN = 7
W5500_MISO_PIN = 4
W5500_CS_PIN = 5
# 원본 CircuitPython code.py에는 W5500 RST 핀이 없었음.
# MicroPython WIZNET5K 펌웨어/보드에 따라 reset pin이 필요할 수 있으므로 실제 배선이 있으면 GPIO 번호 입력.
W5500_RST_PIN = 8
W5500_SPI_BAUD = 10_000_000

# Frame headers
FRAME_HEADER_DIST   = 0xD2
FRAME_HEADER_STATUS = 0xA1
FRAME_HEADER_MOTOR  = 0xA2
FRAME_DISCONNECT    = 0xFF

# Motor / encoder
MOTOR_PPR = 1000
GEAR_RATIO = 10
PULSES_PER_REV = MOTOR_PPR * GEAR_RATIO
WHEEL_DIAMETER_MM = 120
WHEEL_CIRC_MM = math.pi * WHEEL_DIAMETER_MM

SPEED_MAP_MANUAL = [0, 25, 50, 75, 100, 125, 0, 0, 0, 0]
SPEED_MAP_AUTO   = [0, 50, 75, 100, 125, 150, 0, 0, 0, 0]

DECEL_THRESHOLD_MM = 500
MIN_DECEL_SCALE    = 0.3
ACCEL_STEP         = 10
FORCE_STOP_THRESH  = 10  # 기존 0.5mm  → 최소 거리를 0.01m(10mm)까지 요청 하였음

ARRIVAL_BAND_MM   = 25   # 목표 근처 deadband (25mm)

ENCODER_MM_THRESHOLD = 0.2   # 움직임으로 인정할 최소 이동(mm) (튜닝)
ENCODER_FORWARD_SIGN = -1    # 앞(전진)일 때 엔코더가 +증가하면 +1, 반대면 -1

# Battery thresholds (V)
BATTERY_V_20PIN = 2.05
BATTERY_V_99PIN = 2.85

# Timing
POLL_INTERVAL_MS = 10
LIDAR_STREAM_INTERVAL_MS = 100

# Motor state enums
MOTOR_STOP = 0
MOTOR_FWD  = 1
MOTOR_REV  = 2

DEBUG = False


def _sleep_ms(ms):
    try:
        time.sleep_ms(ms)
    except AttributeError:
        time.sleep(ms / 1000.0)


def _ticks_ms():
    return time.ticks_ms()


def _ticks_diff(now, then):
    return time.ticks_diff(now, then)


def _err_no(e):
    # MicroPython OSError는 e.args[0]으로 errno가 들어오는 경우가 많음
    try:
        return e.errno
    except AttributeError:
        pass
    if getattr(e, 'args', None):
        return e.args[0]
    return None


def _is_again(e):
    n = _err_no(e)
    eagain = getattr(errno, 'EAGAIN', 11) if errno else 11
    ewouldblock = getattr(errno, 'EWOULDBLOCK', eagain) if errno else eagain
    return n in (eagain, ewouldblock)


# ======== Hardware Interfaces ========
class LEDBlinker:
    def __init__(self, pin):
        self.led = Pin(pin, Pin.OUT)
        self.off()

    def blink(self, times=1, on_time_ms=200, off_time_ms=200):
        for _ in range(times):
            self.led.value(1)
            _sleep_ms(on_time_ms)
            self.led.value(0)
            _sleep_ms(off_time_ms)

    def on(self):
        self.led.value(1)

    def off(self):
        self.led.value(0)


class MotorController:
    def __init__(self, pwm_pin, dir_pin, enable_pin):
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(1000)
        self.pwm.duty_u16(0)
        self.dir = Pin(dir_pin, Pin.OUT)
        self.enable = Pin(enable_pin, Pin.OUT)
        self.current_pwm_val = 0
        self.stop()

    def set_manual(self, speed_idx, direction):
        pwm_val = SPEED_MAP_MANUAL[speed_idx] if 0 <= speed_idx < len(SPEED_MAP_MANUAL) else 0
        self._apply(pwm_val, direction)

    def set_ramp(self, speed_idx, direction, target_dist, traveled_mm):
        base_pwm = SPEED_MAP_AUTO[speed_idx] if 0 <= speed_idx < len(SPEED_MAP_AUTO) else 0
        rem = max(target_dist - traveled_mm, 0)
        if rem <= FORCE_STOP_THRESH:
            self.stop()
            return
        frac = min(rem / DECEL_THRESHOLD_MM, 1.0)
        scale = MIN_DECEL_SCALE + (1.0 - MIN_DECEL_SCALE) * frac
        target_pwm = int(base_pwm * scale)

        diff = target_pwm - self.current_pwm_val
        if diff > ACCEL_STEP:
            self.current_pwm_val += ACCEL_STEP
        elif diff < -ACCEL_STEP:
            self.current_pwm_val -= ACCEL_STEP
        else:
            self.current_pwm_val = target_pwm

        self._apply(self.current_pwm_val, direction)

    def _apply(self, pwm_val, direction):
        self.pwm.duty_u16(int(pwm_val * 65535 // 255))
        self.dir.value(1 if direction else 0)
        self.enable.value(0)  # enable low = active

    def stop(self):
        self.pwm.duty_u16(0)
        self.enable.value(1)  # disable high = stop
        self.current_pwm_val = 0


class EncoderWrapper:
    """
    1차 MicroPython 포팅용 GPIO IRQ 엔코더.
    - 원본 rotaryio.IncrementalEncoder를 대체
    - 고속/고PPR에서 누락 가능성이 있으므로 최종 운용에서는 PIO 방식 권장
    """
    # prev_state(2bit) + cur_state(2bit) 인덱스 기반 quadrature decode
    _TRANSITION = (0, -1, 1, 0,
                   1, 0, 0, -1,
                   -1, 0, 0, 1,
                   0, 1, -1, 0)

    def __init__(self, pin_a, pin_b):
        self.pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self.position = 0
        self.offset = 0
        self.enc = self  # 기존 self.encoder.enc.position 접근 유지용
        self._last_state = (self.pin_a.value() << 1) | self.pin_b.value()
        trig = Pin.IRQ_RISING | Pin.IRQ_FALLING
        self.pin_a.irq(trigger=trig, handler=self._irq)
        self.pin_b.irq(trigger=trig, handler=self._irq)

    def _irq(self, pin):
        state = (self.pin_a.value() << 1) | self.pin_b.value()
        idx = (self._last_state << 2) | state
        self.position += self._TRANSITION[idx]
        self._last_state = state

    def reset(self):
        self.offset = self.position

    @property
    def distance_mm(self):
        delta = self.position - self.offset
        return abs(delta) / PULSES_PER_REV * WHEEL_CIRC_MM


class BatteryMonitor:
    def __init__(self, adc_pin):
        self.adc = ADC(adc_pin)

    def get_bin_0_9(self):
        """
        0..9 등분값 반환:
          v < 2.05V → 0
          2.05~2.85V → 1..8 (균등 8등분)
          v ≥ 2.85V → 9
        """
        v = (self.adc.read_u16() * 3.3) / 65535
        if v <= BATTERY_V_20PIN:
            return 0
        if v >= BATTERY_V_99PIN:
            return 9
        step = (BATTERY_V_99PIN - BATTERY_V_20PIN) / 8.0
        idx = int((v - BATTERY_V_20PIN) // step) + 1  # 1..9
        return max(1, min(8, idx))


class LiDARHandler:
    # 클래스 lidar와 모듈 lidar 헷갈림 방지 클래스 lidar는 self.lidar, 모듈 lidar는 self.lidar_module
    def __init__(self, uart):
        self.lidar_module = LiDAR(uart)
        self.streaming = False

    def start_stream(self):
        self.streaming = True
        self.lidar_module.set_absolute_mode()
        self.lidar_module.start_stream()

    def stop_stream(self):
        self.streaming = False
        self.lidar_module.stop_stream()

    def reset_baseline(self):
        self.lidar_module.reset_baseline()

    def read_frame(self):
        return self.lidar_module.read_frame()


class NetworkInterface:
    def __init__(self):
        if network is None:
            raise RuntimeError('network module 없음: WIZNET5K 지원 MicroPython 펌웨어가 필요합니다.')
        if socket is None:
            raise RuntimeError('socket/usocket module 없음: TCP 서버를 쓰려면 network/socket 포함 MicroPython 펌웨어가 필요합니다.')

        self.spi = SPI(
            W5500_SPI_ID,
            baudrate=W5500_SPI_BAUD,
            polarity=0,
            phase=0,
            sck=Pin(W5500_SCK_PIN),
            mosi=Pin(W5500_MOSI_PIN),
            miso=Pin(W5500_MISO_PIN)
        )

        self.cs = Pin(W5500_CS_PIN, Pin.OUT, value=1)
        self.rst = Pin(W5500_RST_PIN, Pin.OUT, value=1)
        # W5500 하드 리셋
        self.rst.value(0)
        time.sleep_ms(100)
        self.rst.value(1)
        time.sleep_ms(300)

        self.eth = network.WIZNET5K(self.spi, self.cs, self.rst)

        # MicroPython WIZNET5K는 빌드/버전에 따라 reset pin 인자가 필요할 수 있음
        try:
            if self.rst is not None:
                self.eth = network.WIZNET5K(self.spi, self.cs, self.rst)
            else:
                self.eth = network.WIZNET5K(self.spi, self.cs)
        except TypeError:
            if self.rst is None:
                raise RuntimeError('이 펌웨어의 network.WIZNET5K는 reset pin이 필요합니다. W5500_RST_PIN을 실제 배선 GPIO로 설정하세요.')
            raise

        try:
            self.eth.active(True)
        except Exception:
            pass

        try:
            self.eth.config(mac=MY_MAC)
        except Exception:
            pass

        self.eth.ifconfig((IP_ADDRESS, SUBNET_MASK, GATEWAY_ADDRESS, DNS_SERVER))
        print('W5500 ifconfig:', self.eth.ifconfig())

    def socket(self):
        return socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    @property
    def link_ok(self):
        try:
            return bool(self.eth.isconnected())
        except Exception:
            try:
                return self.eth.ifconfig()[0] != '0.0.0.0'
            except Exception:
                return False


# ======== AGV Server ========
class AGVServer:
    def __init__(self):
        # Hardware init
        self.led = LEDBlinker(25)
        self.battery = BatteryMonitor(28)
        self.motor = MotorController(15, 14, 13)
        self.encoder = EncoderWrapper(2, 3)
        self.lidar = LiDARHandler(UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1)))
        self.network = NetworkInterface()

        # State
        self.command_queue = []
        self.rx_buf = bytearray()
        self.target_dist = 0
        self.ui_distance_m = 0
        self.remaining = 0
        self.last_drive = 0
        self.last_speed = 0
        self.drive_mode = 0
        self.last_status_call = None
        self.server = None

        self.motor_state = MOTOR_STOP  # 마지막 보고한 모터 상태 (중복 전송 방지)
        self._prev_pos = self.encoder.enc.position

        self._lidar_last = None

        self.zsd_call = False
        self.get_to_back_mm = None    # 도착 처리용 값 (lidar 거리 제어용)

    # ---------- helpers: TX packets ----------
    def send_motor_state(self, sock, state):
        """[0xA2, state(0/1/2), 0, 0]"""
        pkt = bytes([FRAME_HEADER_MOTOR, state & 0xFF, 0x00, 0x00])
        try:
            sock.send(pkt)
            if DEBUG:
                print(pkt)
        except Exception as e:
            print('send_motor_state err:', e)

    def send_battery_bin(self, sock):
        """[0xA1, 2, 0, adc_bin]"""
        adc_bin = self.battery.get_bin_0_9()
        pkt = bytes([FRAME_HEADER_STATUS, 0x02, 0x00, adc_bin & 0xFF])
        try:
            sock.send(pkt)
            if DEBUG:
                print(pkt)
        except Exception as e:
            print('send_battery_bin err:', e)

    def motor_state_check(self):
        """엔코더 변화로 FWD/REV/STOP 판정 (지터 내성)"""
        cur = self.encoder.enc.position
        delta_ticks = cur - self._prev_pos
        self._prev_pos = cur

        # ticks -> mm
        delta_mm = (delta_ticks / PULSES_PER_REV) * WHEEL_CIRC_MM
        if abs(delta_mm) < ENCODER_MM_THRESHOLD:
            return MOTOR_STOP

        return MOTOR_FWD if (delta_ticks * ENCODER_FORWARD_SIGN) > 0 else MOTOR_REV

    def _report_motor_state_if_changed(self, sock):
        """상태가 바뀔 때만 1회 보고 (A2 → A1 순서)"""
        state = self.motor_state_check()
        if state != self.motor_state:
            self.motor_state = state
            self.send_motor_state(sock, state)  # [0xA2, state, 0, 0]
            self.send_battery_bin(sock)

    # ---------- RX / dispatch ----------
    def receive_frame(self, sock):
        try:
            data = sock.recv(32)
            if not data:
                raise OSError(104)  # connection reset/disconnect 취급
            self.rx_buf.extend(data)

            # TCP 분할/합쳐짐 대응: 8바이트 프레임 단위로 검증
            while len(self.rx_buf) >= 8:
                cmd = bytes(self.rx_buf[:8])
                if (sum(cmd[:7]) & 0xFF) == cmd[7]:
                    self.command_queue.append((sock, cmd))
                    self.rx_buf = self.rx_buf[8:]
                else:
                    # checksum이 맞지 않으면 1바이트씩 밀면서 재동기
                    self.rx_buf = self.rx_buf[1:]
        except OSError as e:
            if _is_again(e):
                return
            raise

    def dispatcher(self):
        sock, cmd = self.command_queue.pop(0)
        mode, drive, speed, _, status_call, _, _, _ = cmd

        # Disconnect
        if mode == FRAME_DISCONNECT:
            print('Client sent disconnect frame — shutting down Pico server.')
            try:
                sock.close()
                self.server.close()
            except Exception:
                pass
            raise SystemExit

        # Distance commands
        if mode == 2:
            hi, lo = cmd[1], cmd[2]
            raw_m = hi * 100 + lo          # 예: 27, 44 → 2744 → 27.44m이니 pico는 27440mm로, 02, 00 → 0200 → 2.00m이니 pico는 2000mm로
            self.target_dist = raw_m * 10  # m → mm
            self.encoder.reset()
            self.remaining = 0
            frame = bytes([FRAME_HEADER_DIST, hi, lo])  # ACK
            sock.send(frame)
            if DEBUG:
                print('Dist_reset')

        if mode == 3:
            self.target_dist = 0
            self.encoder.reset()
            self.remaining = 0
            if DEBUG:
                print('Dist_reset')
            sock.send(b'Dist_reset\n')

        # Manual / Auto drive
        self.drive_mode = drive

        # mode 값 상관 없이 영점 조절 이후 라면 원위치 이동 가능하게
        if self.drive_mode == 3 and self.zsd_call == True:
            self.get_to_back_mm = - self.lidar.lidar_module.rel_mm_last  # +면 전진, -면 후진
            if abs(self.get_to_back_mm) < 25:    # 25mm 이내면 도착처리
                self.motor.stop()
                self.target_dist = 0
                self.drive_mode = 0
                self.zsd_call = False
                self._prev_pos = self.encoder.enc.position
                return
            else:
                self.target_dist = abs(self.get_to_back_mm)
                self.encoder.reset()
                self._prev_pos = self.encoder.enc.position
                self.drive_mode = 1 if self.get_to_back_mm < 0 else 2
                # 속도는 UI측에서 전달하는 속도 값 단계로
                return
        else:
            pass

        if mode == 0:  # Manual
            self.target_dist = 0
            self.remaining   = 0
            if drive == 0:
                self.motor.stop()
                self._prev_pos = self.encoder.enc.position
            elif drive == 1:
                self.motor.set_manual(speed, 0)  # FWD → dir=0 (하드웨어에 맞게 유지)
                self._prev_pos = self.encoder.enc.position
            elif drive == 2:
                self.motor.set_manual(speed, 1)  # REV → dir=1
                self._prev_pos = self.encoder.enc.position
            else:
                # 정의 외 값이면 무시
                pass

        elif mode == 1:  # Auto
            # 새로 전진/후진 시작
            if (self.last_drive != drive) and (drive in (1, 2)) and (self.target_dist > 0):
                self.remaining = self.target_dist
                self.encoder.reset()
                self._prev_pos = self.encoder.enc.position

            # 일시정지/정지
            if drive == 0:
                self.remaining = self.target_dist - self.encoder.distance_mm
                self.motor.stop()
            # 재개 지시
            elif drive == 1 and self.remaining > 0:
                self.target_dist = self.remaining
                self.remaining = 0
                self.encoder.reset()
                self._prev_pos = self.encoder.enc.position
            elif drive == 2 and self.remaining > 0:
                self.target_dist = self.remaining
                self.remaining = 0
                self.encoder.reset()
                self._prev_pos = self.encoder.enc.position
            else:
                # 다른 케이스는 모터 제어 루프에서 처리
                pass

        self.last_drive = drive
        self.last_speed = speed

        # Status / LiDAR / Baseline
        if status_call == 0 and self.last_status_call != 0:
            self.send_status(sock)
        elif status_call == 1 and self.last_status_call != 1:
            self.lidar.start_stream()
        elif status_call == 2 and self.last_status_call != 2:
            self.lidar.stop_stream()
        elif status_call == 3 and self.last_status_call != 3:
            self.lidar.reset_baseline()
            self.zsd_call = True
        self.last_status_call = status_call

    def send_status(self, sock):
        # 단순 링크 상태만 보고 (LiDAR 프레임 소모 방지)
        code = 0 if self.network.link_ok else 1
        resp = bytes([FRAME_HEADER_STATUS, 0 if code == 0 else 1, code, 0])
        try:
            sock.send(resp)
            if DEBUG:
                print(resp)
        except Exception as e:
            print('send_status err:', e)

    def run(self):
        print('AGV Rail TCP server ready...')
        self.led.on()
        self.server = self.network.socket()
        try:
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except Exception:
            pass
        self.server.bind((IP_ADDRESS, SERVER_PORT))

        # 링크 업 대기
        print('Waiting for Ethernet link...')
        while not self.network.link_ok:
            _sleep_ms(POLL_INTERVAL_MS)
        print('Ethernet link established, listening...')

        # 리슨 시작
        while True:
            try:
                self.server.listen(1)
                break
            except Exception:
                _sleep_ms(POLL_INTERVAL_MS)

        while True:
            print('Waiting for client...')
            conn, addr = self.server.accept()
            print('Client connected:', addr)
            try:
                conn.setblocking(False)
            except Exception:
                pass

            self.rx_buf = bytearray()
            state_now = self.motor_state_check()
            self.send_motor_state(conn, state_now)
            self.send_battery_bin(conn)
            # 연결 시점에 기준 재설정 (첫 판정 튐 방지)
            self._prev_pos = self.encoder.enc.position
            last_lidar_time = _ticks_ms()
            last_gc_time = _ticks_ms()

            try:
                while True:
                    self.receive_frame(conn)
                    if self.command_queue:
                        self.dispatcher()

                    now = _ticks_ms()

                    # LiDAR streaming   100ms 단위로 전달
                    if self.lidar.streaming and _ticks_diff(now, last_lidar_time) > LIDAR_STREAM_INTERVAL_MS:
                        frame = self.lidar.read_frame()
                        if frame:
                            try:
                                conn.send(frame)    # 클라이언트 측으로 라이다 거리 값은 계속 보내기
                            except Exception as e:
                                print('lidar send err:', e)
                        last_lidar_time = now

                    # Auto drive update (거리 주행)
                    if self.target_dist > 0 and self.drive_mode in (1, 2):
                        traveled = self.encoder.distance_mm
                        err = self.target_dist - traveled   # 목표 이동 거리 - 현재 이동 중인 거리 = 남은 거리
                        self.motor.set_ramp(
                            self.last_speed,
                            0 if self.drive_mode == 1 else 1,
                            self.target_dist,
                            traveled
                        )
                        if err < ARRIVAL_BAND_MM:
                            # 목표 도달 → 정지 & 상태/배터리 즉시 보고
                            self.motor.stop()
                            self._prev_pos = self.encoder.enc.position  # 진동에 의한 잔여 Δ 제거
                            self.motor_state = MOTOR_STOP               # 상태 머신을 STOP으로 강제 싱크
                            self.send_motor_state(conn, MOTOR_STOP)     # A2
                            self.drive_mode = 0
                            self.remaining = 0
                            self.ui_distance_m = 0
                            self.target_dist = 0
                            self._report_motor_state_if_changed(conn)

                    # 매 tick 엔코더 기반 상태변경 보고 (Manual/AUTO 공통)
                    self._report_motor_state_if_changed(conn)

                    # 장시간 운용 시 GC 시점을 조금 통제
                    if _ticks_diff(now, last_gc_time) > 1000:
                        gc.collect()
                        last_gc_time = now

                    _sleep_ms(POLL_INTERVAL_MS)

            except OSError:
                print('Client disconnected')
            except Exception as e:
                print('[Error]', e)
                self.led.blink(times=5, on_time_ms=100, off_time_ms=100)
            finally:
                try:
                    conn.close()
                except Exception:
                    pass
                self.motor.stop()
                self.lidar.stop_stream()
                self.led.blink(times=5, on_time_ms=100, off_time_ms=100)
                self.led.on()


if __name__ == '__main__':
    server = AGVServer()
    server.run()

