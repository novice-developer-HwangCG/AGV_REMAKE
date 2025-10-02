import time
import board
import busio
import digitalio
import analogio
import rotaryio
import pwmio
import errno
import math
import sys

from adafruit_wiznet5k.adafruit_wiznet5k import WIZNET5K
import adafruit_wiznet5k.adafruit_wiznet5k_socket as wiznet5k_socket

from lidar import LiDAR

# ======== Constants ========
# Network
MY_MAC = (0x00, 0x01, 0x02, 0x03, 0x04, 0x05)
IP_ADDRESS = (192, 168, 100, 2)     # 변경: 10 → 2
SUBNET_MASK = (255, 255, 255, 0)
GATEWAY_ADDRESS = (192, 168, 100, 1)
DNS_SERVER = (8, 8, 8, 8)
SERVER_PORT = 5024

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

# Battery thresholds (V)
BATTERY_V_20PIN = 2.05
BATTERY_V_99PIN = 2.85

# Timing
POLL_INTERVAL_SEC = 0.01
LIDAR_STREAM_INTERVAL = 0.10

# Motor state enums
MOTOR_STOP = 0
MOTOR_FWD  = 1
MOTOR_REV  = 2

DEBUG = False

# ======== Hardware Interfaces ========
class LEDBlinker:
    def __init__(self, pin):
        self.led = digitalio.DigitalInOut(pin)
        self.led.direction = digitalio.Direction.OUTPUT

    def blink(self, times=1, on_time=0.2, off_time=0.2):
        for _ in range(times):
            self.led.value = True
            time.sleep(on_time)
            self.led.value = False
            time.sleep(off_time)

    def on(self):  self.led.value = True
    def off(self): self.led.value = False

class MotorController:
    def __init__(self, pwm_pin, dir_pin, enable_pin):
        self.pwm = pwmio.PWMOut(pwm_pin, frequency=1000, duty_cycle=0)
        self.dir = digitalio.DigitalInOut(dir_pin)
        self.dir.direction = digitalio.Direction.OUTPUT
        self.enable = digitalio.DigitalInOut(enable_pin)
        self.enable.direction = digitalio.Direction.OUTPUT
        self.current_pwm_val = 0

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
        self.pwm.duty_cycle = int(pwm_val * 65535 // 255)
        self.dir.value = direction
        self.enable.value = 0  # enable low = active

    def stop(self):
        self.pwm.duty_cycle = 0
        self.enable.value = 1  # disable high = stop
        self.current_pwm_val = 0

class EncoderWrapper:
    def __init__(self, pin_a, pin_b):
        self.enc = rotaryio.IncrementalEncoder(pin_a, pin_b)
        self.offset = 0

    def reset(self):
        self.offset = self.enc.position

    @property
    def distance_mm(self):
        delta = self.enc.position - self.offset
        return abs(delta) / PULSES_PER_REV * WHEEL_CIRC_MM

class BatteryMonitor:
    def __init__(self, adc_pin):
        self.adc = analogio.AnalogIn(adc_pin)

    def get_bin_0_9(self):
        """
        0..9 등분값 반환:
          v < 2.05V → 0
          2.05~2.85V → 1..8 (균등 8등분)
          v ≥ 2.85V → 9
        """
        v = (self.adc.value * 3.3) / 65535
        if v < BATTERY_V_20PIN:
            return 0
        if v >= BATTERY_V_99PIN:
            return 9
        step = (BATTERY_V_99PIN - BATTERY_V_20PIN) / 8.0
        idx = int((v - BATTERY_V_20PIN) // step) + 1  # 1..8
        if idx < 1: idx = 1
        if idx > 8: idx = 8
        return idx

class LiDARHandler:
    def __init__(self, uart):
        self.lidar = LiDAR(uart)
        self.streaming = False

    def start_stream(self):
        self.streaming = True
        self.lidar.set_absolute_mode()

    def stop_stream(self):
        self.streaming = False
        self.lidar.pause()

    def reset_baseline(self):
        self.lidar.reset_baseline()

    def read_frame(self):
        return self.lidar.read_frame()

class NetworkInterface:
    def __init__(self, spi_pins, cs_pin):
        spi = busio.SPI(*spi_pins)
        cs = digitalio.DigitalInOut(cs_pin)
        self.eth = WIZNET5K(spi, cs, is_dhcp=False, mac=MY_MAC, hostname="AGVRAIL", debug=False)
        self.eth.ifconfig = (IP_ADDRESS, SUBNET_MASK, GATEWAY_ADDRESS, DNS_SERVER)
        wiznet5k_socket.set_interface(self.eth)

    def socket(self):
        return wiznet5k_socket.socket()

    @property
    def link_ok(self):
        return bool(self.eth.link_status) and (self.eth.ifconfig[0] != "0.0.0.0")

# ======== AGV Server ========
class AGVServer:
    def __init__(self):
        # Hardware init
        self.led = LEDBlinker(board.GP25)
        self.battery = BatteryMonitor(board.GP28)
        self.motor = MotorController(board.GP15, board.GP14, board.GP13)
        self.encoder = EncoderWrapper(board.GP2, board.GP3)
        self.lidar = LiDARHandler(busio.UART(board.GP0, board.GP1, baudrate=115200))
        self.network = NetworkInterface((board.GP6, board.GP7, board.GP4), board.GP5)

        # State
        self.command_queue = []
        self.target_dist = 0
        self.ui_distance_m = 0
        self.remaining = 0
        self.last_drive = 0
        self.last_speed = 0
        self.drive_mode = 0
        self.last_status_call = None
        self.server = None
        self.motor_state = MOTOR_STOP  # 마지막 보고한 모터 상태 (중복 전송 방지)

        self.move_mm_thr = 1.0          # 움직임 임계 값
        self._prev_pos = 0

        self.get_to_back_mm = None    # 도착 처리용 값

    # ---------- helpers: TX packets ----------
    def send_motor_state(self, sock, state):
        """[0xA2, state(0/1/2), 0, 0]"""
        pkt = bytes([FRAME_HEADER_MOTOR, state & 0xFF, 0x00, 0x00])
        try:
            sock.send(pkt)
            if DEBUG:
                print(pkt)
        except Exception as e:
            print("send_motor_state err:", e)

    def send_battery_bin(self, sock):
        """[0xA1, 2, 0, adc_bin]"""
        adc_bin = self.battery.get_bin_0_9()
        pkt = bytes([FRAME_HEADER_STATUS, 0x02, 0x00, adc_bin & 0xFF])
        try:
            sock.send(pkt)
            if DEBUG:
                print(pkt)
        except Exception as e:
            print("send_battery_bin err:", e)

    def motor_state_check(self):
        # 단순 엔코더 값 변화로 전진 후진 정지 변화 확인
        cur = self.encoder.enc.position
        delta = cur - self._prev_pos
        self._prev_pos = cur

        if abs(delta) < 1:      # 임계(튜닝)
            return MOTOR_STOP
        return MOTOR_FWD if delta > 0 else MOTOR_REV

    # ---------- RX / dispatch ----------
    def receive_frame(self, sock):
        try:
            # wiznet5k 소켓은 available() 지원 (CircuitPython 포팅)
            if hasattr(sock, "available") and sock.available() < 8:
                return
            cmd = sock.recv(8)
            if not cmd:
                raise ConnectionResetError
            if len(cmd) == 8 and (sum(cmd[:7]) & 0xFF) == cmd[7]:
                self.command_queue.append((sock, cmd))
        except OSError as e:
            if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return
            raise

    def dispatcher(self):
        sock, cmd = self.command_queue.pop(0)
        mode, drive, speed, _, status_call, _, _, _ = cmd

        # Disconnect
        if mode == FRAME_DISCONNECT:
            print("Client sent disconnect frame — shutting down Pico server.")
            try:
                sock.close()
                self.server.close()
            except:
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
                print("Dist_reset")

        if mode == 3:
            self.target_dist = 0
            self.encoder.reset()
            self.remaining = 0
            if DEBUG:
                print("Dist_reset")
            sock.send(b"Dist_reset\n")

        # Manual / Auto drive
        prev_state = self.motor_state
        self.drive_mode = drive

        if mode == 0:  # Manual
            if drive == 0:
                self.motor.stop()
            elif drive == 1:
                self.motor.set_manual(speed, 0)  # FWD → dir=0 (하드웨어에 맞게 유지)
                self.motor_state_check()
            elif drive == 2:
                self.motor.set_manual(speed, 1)  # REV → dir=1
                self.motor_state_check()
            else:
                # 정의 외 값이면 무시
                pass

        elif mode == 1:  # Auto
            # 새로 전진/후진 시작
            if (self.last_drive != drive) and (drive in (1, 2)) and (self.target_dist > 0):
                self.remaining = self.target_dist
                self.encoder.reset()

            # 일시정지/정지
            if drive == 0:
                self.remaining = self.target_dist - self.encoder.distance_mm
                self.motor.stop()
                self.motor_state_check()
            # 재개 지시
            elif drive == 1 and self.remaining > 0:
                self.target_dist = self.remaining
                self.remaining = 0
                self.encoder.reset()
                self.motor_state_check()
            elif drive == 2 and self.remaining > 0:
                self.target_dist = self.remaining
                self.remaining = 0
                self.encoder.reset()
                self.motor_state_check()
            elif drive == 3:
                self.get_to_back_mm = - self.lidar.lidar.rel_mm_last  # +면 전진, -면 후진
                if abs(self.get_to_back_mm) < 25:    # 25mm 이내면 도착처리
                    self.motor.stop()
                    self.target_dist = 0
                    self.drive_mode = 0
                    self.motor_state_check()
                else:
                    self.target_dist = abs(self.get_to_back_mm)
                    self.encoder.reset()
                    self.drive_mode = 1 if self.get_to_back_mm < 0 else 2
                    self.motor_state_check()
                    # 속도는 UI측에서 전달하는 속도 값 단계로
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
            print("send_status err:", e)

    def run(self):
        print("AGV Rail TCP server ready...")
        self.led.on()
        self.server = self.network.socket()
        self.server.bind((f"{IP_ADDRESS[0]}.{IP_ADDRESS[1]}.{IP_ADDRESS[2]}.{IP_ADDRESS[3]}", SERVER_PORT))

        # 링크 업 대기
        print("Waiting for Ethernet link...")
        while not self.network.link_ok:
            time.sleep(POLL_INTERVAL_SEC)
        print("Ethernet link established, listening...")

        # 리슨 시작 (W5500 특성상 재시도)
        while True:
            try:
                self.server.listen(1)
                break
            except AssertionError:
                time.sleep(POLL_INTERVAL_SEC)

        while True:
            conn, addr = self.server.accept()
            print("Client connected:", addr)
            last_lidar_time = time.monotonic()
            try:
                while True:
                    self.receive_frame(conn)
                    if self.command_queue:
                        self.dispatcher()

                    now = time.monotonic()

                    # LiDAR streaming   100ms 단위로 전달
                    if self.lidar.streaming and (now - last_lidar_time) > LIDAR_STREAM_INTERVAL:
                        frame = self.lidar.read_frame()
                        if frame:
                            try:
                                conn.send(frame)    # 클라이언트 측으로 라이다 거리 값은 계속 보내기
                            except Exception as e:
                                print("lidar send err:", e)
                        last_lidar_time = now

                    # Auto drive update (거리 주행)
                    if self.target_dist > 0 and self.drive_mode in (1, 2):
                        traveled = self.encoder.distance_mm
                        self.motor.set_ramp(
                            self.last_speed,
                            0 if self.drive_mode == 1 else 1,
                            self.target_dist,
                            traveled
                        )
                        if traveled >= self.target_dist:
                            # 목표 도달 → 정지 & 상태/배터리 즉시 보고
                            self.motor.stop()
                            self.drive_mode = 0
                            self.remaining = 0
                            self.ui_distance_m = 0

                    time.sleep(POLL_INTERVAL_SEC)

            except OSError:
                print("Client disconnected")
            except Exception as e:
                print("[Error]", e)
                self.led.blink(times=5, on_time=0.1, off_time=0.1)
            finally:
                try:
                    conn.close()
                except:
                    pass
                self.motor.stop()
                self.lidar.stop_stream()
                self.led.blink(times=5, on_time=0.1, off_time=0.1)
                self.led.on()

if __name__ == '__main__':
    server = AGVServer()
    server.run()
