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
IP_ADDRESS = (192, 168, 100, 10)
SUBNET_MASK = (255, 255, 255, 0)
GATEWAY_ADDRESS = (192, 168, 100, 1)
DNS_SERVER = (8, 8, 8, 8)
SERVER_PORT = 5024

# Frame headers
FRAME_HEADER_DIST = 0xD2
FRAME_HEADER_STATUS = 0xA1
FRAME_HEADER_MOTOR = 0xA2
FRAME_DISCONNECT = 0xFF

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
FORCE_STOP_THRESH  = 50  # mm

# Battery
BATTERY_V_20PIN = 2.05
BATTERY_V_99PIN = 2.85

# Timing
POLL_INTERVAL_SEC = 0.01
LIDAR_STREAM_INTERVAL = 0.25

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

    def on(self):
        self.led.value = True

    def off(self):
        self.led.value = False

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

    def get_percent(self):
        Vpin = (self.adc.value * 3.3) / 65535
        if Vpin >= BATTERY_V_99PIN:
            return 99
        if Vpin <= BATTERY_V_20PIN:
            return 20
        soc = 20 + (Vpin - BATTERY_V_20PIN) * (99 - 20) / (BATTERY_V_99PIN - BATTERY_V_20PIN)
        return int(soc)

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
        if not self.eth.link_status:
            return True
        return self.eth.ifconfig[0] != "0.0.0.0"

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
        self.offset = 0
        self.target_dist = 0
        self.ui_distance_m = 0
        self.remaining = 0
        self.last_drive = 0
        self.last_speed = 0
        self.drive_mode = 0
        self.last_status_call = None
        self.last_power_call = None
        self.server = None

    def receive_frame(self, sock):
        try:
            if sock.available() < 8:
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
        mode, drive, speed, _, status_call, power_call, _, _ = cmd

        # Disconnect
        if mode == FRAME_DISCONNECT:
            print("Client sentdisconnect frame — shutting down Pico server.")
            sock.close()
            self.server.close()
            raise SystemExit

        # Distance commands
        if mode == 2:
            hi, md, lo = cmd[1], cmd[2], cmd[3]
            raw = hi * 10000 + md * 100 + lo        # UI에서 보낸 값, 단위: 1/10000 m
            meters = raw / 10000.0                  # e.g. 27.1144 m
            # 실제 pico 주행용은 mm 단위로 변환
            self.target_dist = int(meters * 1000)  # 27114 mm → 27.114 m
            # 디버깅용으로 m 단위도 저장
            # self.ui_distance_m = meters
            self.encoder.reset()
            self.remaining = 0
            # frame = bytes([FRAME_HEADER_DIST, hi, md, lo])
            sock.send(frame)

        if mode == 3:
            self.target_dist = 0
            self.encoder.reset()
            self.remaining = 0
            sock.send(b"Dist_reset\n")

        # Manual / Auto
        self.drive_mode = drive
        if mode == 0:  # Manual
            if drive == 0:
                self.motor.stop()
            else:
                self.motor.set_manual(speed, 0 if drive == 1 else 1)
        elif mode == 1:  # Auto
            if self.last_drive != drive and drive in (1, 2) and self.target_dist > 0:
                self.remaining = self.target_dist
                self.encoder.reset()
            if drive == 0:
                self.remaining = self.target_dist - self.encoder.distance_mm
                self.motor.stop()
            elif drive in (1,2) and self.remaining > 0:
                self.target_dist = self.remaining
                self.remaining = 0
                self.encoder.reset()
            else:
                self.motor.stop()
        self.last_drive = drive
        self.last_speed = speed

        # Status / LiDAR / Battery
        if status_call == 0 and self.last_status_call != 0:
            self.send_status(sock)
        elif status_call == 1 and self.last_status_call != 1:
            self.lidar.start_stream()
        elif status_call == 2 and self.last_status_call != 2:
            self.lidar.stop_stream()
        elif status_call == 3 and self.last_status_call != 3:
            self.lidar.reset_baseline()
        self.last_status_call = status_call

        if power_call == 1 and self.last_power_call != 1:
            bat = self.battery.get_percent()
            resp = bytes([FRAME_HEADER_STATUS, bat, 0x00, 0x00])
            sock.send(resp)
            if bat <= 20:
                sock.send(b"Battery level is at or below 20%\n")
        self.last_power_call = power_call

    def send_status(self, sock):
        code = 0 if self.network.link_ok and (self.lidar.read_frame() or True) else 1
        resp = bytes([FRAME_HEADER_STATUS, 0 if code==0 else 1, code, 0])
        sock.send(resp)

    def motor_status_frame(self, conn):
        """
        drive 값에 따라 모터 제어 상태에 대한 프레임을 전송
        프레임 포맷: [0xA2, motor_status, 0x00, 0x00]
        """
        # 0,1,2 외에는 전송하지 않음
        if self.drive_mode not in (0, 1, 2):
            return
        resp = bytes([0xA2, self.drive_mode, 0x00, 0x00])
        conn.send(resp)

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

        # 연결 유지
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
            last_motor_status_ts = time.monotonic()
            try:
                while True:
                    self.receive_frame(conn)
                    if self.command_queue:
                        self.dispatcher()

                    now = time.monotonic()
                    # Motor streaming
                    # if now - last_motor_status_ts >= 0.5:
                        # self.motor_status_frame(conn)
                        # last_motor_status_ts = now

                    # LiDAR streaming
                    if self.lidar.streaming and now - last_lidar_time > LIDAR_STREAM_INTERVAL:
                        frame = self.lidar.read_frame()
                        if frame:
                            conn.send(frame)
                        last_lidar_time = now

                    # Auto drive update
                    if self.target_dist > 0 and self.drive_mode in (1,2):
                        # 거리 주행 (rotaryio 기반)
                        traveled = self.encoder.distance_mm
                        self.motor.set_ramp(
                            self.last_speed,
                            0 if self.drive_mode==1 else 1,
                            self.target_dist,
                            traveled
                        )
                        if traveled >= self.target_dist:
                            self.drive_mode = 0
                            self.remaining = 0
                            self.ui_distance_m = 0
                            self.motor.stop()

                    time.sleep(POLL_INTERVAL_SEC)
            except OSError:
                print("Client disconnected")
            except Exception as e:
                print("[Error]", e)
                self.led.blink(times=5, on_time=0.1, off_time=0.1)
            finally:
                try: conn.close()
                except: pass
                self.motor.stop()
                self.lidar.stop_stream()
                self.led.blink(times=5, on_time=0.1, off_time=0.1)
                self.led.on()

if __name__ == '__main__':
    server = AGVServer()
    server.run()
