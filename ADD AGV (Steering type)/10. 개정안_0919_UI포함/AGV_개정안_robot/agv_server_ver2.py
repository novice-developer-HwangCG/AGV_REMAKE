#!/usr/bin/env python3
# jetson_server_refactored.py
import os
import sys, traceback
import socket
import threading
import time
# from dataclasses import dataclass
from queue import Queue, Empty
from typing import Optional, List, Set, Tuple

import cv2

from lidar import LiDAR
from rs485_motor import MotorController
from linetracing import LineTracer
from encoder_pico import AGVENCODER

# ──────────────────────────────────────────────────────────────────────────────
# Config & Helpers
# ──────────────────────────────────────────────────────────────────────────────

DEBUG = False

def _hx(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

class Config(object):
    def __init__(self):
        self.host = "0.0.0.0"
        self.port = 5024
        self.param_size = 8
        self.lidar_port = "/dev/ttyLiDAR"
        self.lidar_baud = 115200
        self.pico_port = "/dev/ttyPICO"
        self.pico_baud = 115200
        self.camera_device = 0
        self.ka_idle = 3
        self.ka_intvl = 1
        self.ka_cnt = 2

V_20PIN = 2.05  # ~20%
V_99PIN = 2.85  # ~90%

def pin_voltage_to_soc(v: Optional[float]) -> int:
    """
    2.05V 이하면 1(저전압 경고 구간), 2.05~2.85V를 8등분하여 2..9,
    2.85V 초과는 9로 클램프.
    """
    if v is None:
        return 0
    if v <= V_20PIN:
        return 1
    if v >= V_99PIN:
        return 9
    step = (V_99PIN - V_20PIN) / 8.0
    # 2(>20%)..9(>=90%) 사이 구간화
    idx = int((v - V_20PIN) // step) + 2
    return max(2, min(9, idx))

def recv_exact(conn: socket.socket, n: int) -> Optional[bytes]:
    buf = bytearray()
    while len(buf) < n:
        try:
            chunk = conn.recv(n - len(buf))
        except socket.timeout:
            continue
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)

# ──────────────────────────────────────────────────────────────────────────────
# Devices container (shared singletons)
# ──────────────────────────────────────────────────────────────────────────────
class SystemDevices:
    def __init__(self, cfg: Config):
        # --- Motor ---
        try:
            print("[BOOT] Opening RS485 motor...")
            self.motor = MotorController(port="/dev/ttyRS485", baudrate=115200)
            print("[BOOT] RS485 motor OK")
        except Exception as e:
            print(f"[BOOT] RS485 motor OPEN FAIL: {e}")
            self.motor = self._make_dummy_motor()

        # --- LiDAR ---
        try:
            print(f"[BOOT] Opening LiDAR on {cfg.lidar_port} {cfg.lidar_baud}...")
            try:
                self.lidar = LiDAR(cfg.lidar_port, cfg.lidar_baud)  # (port, baud)
            except TypeError:
                try:
                    self.lidar = LiDAR(cfg.lidar_port)               # (port,)
                except TypeError:
                    self.lidar = LiDAR()                            # ()
            print("[BOOT] LiDAR OK")
        except Exception as e:
            print(f"[BOOT] LiDAR OPEN FAIL: {e}")
            self.lidar = self._make_dummy_lidar()

        # --- LineTracer ---
        # self.tracer = LineTracer(self.motor)
        try:
            print("[BOOT] Creating LineTracer...")
            self.tracer = LineTracer(self.motor)
            print("[BOOT] LineTracer OK")
        except Exception as e:
            import traceback
            print(f"[BOOT] LineTracer FAIL: {e}")
            traceback.print_exc()
            self.tracer = self._make_dummy_tracer()

        # --- Encoder ---
        self._last_adc_lock = threading.Lock()
        self._last_adc: Optional[float] = None
        def _on_adc_value(adc: float):
            with self._last_adc_lock:
                self._last_adc = adc
        try:
            print(f"[BOOT] Opening PICO on {cfg.pico_port} {cfg.pico_baud}...")
            self.encoder = AGVENCODER(self.tracer, self.motor, adc_callback=_on_adc_value,
                                      port=cfg.pico_port, baudrate=cfg.pico_baud)
            print("[BOOT] PICO OK")
        except Exception as e:
            print(f"[BOOT] PICO OPEN FAIL: {e}")
            self.encoder = self._make_dummy_encoder()

    def get_last_adc(self) -> Optional[float]:
        with self._last_adc_lock:
            return self._last_adc

    # ---------- Dummy devices ----------
    def _make_dummy_motor(self):
        class _DummyMotor:
            is_open = True
            def servo_on(self): pass
            def servo_off(self): pass
            def stop(self): pass
            def send_speeds(self, l, r): pass
            def set_manual_mode(self): self.mode='manual'
            def set_auto_mode(self): self.mode='auto'
            @property
            def get_mode(self): return getattr(self, 'mode', 'manual')
            def get_drive(self, d): self.manual_drive = d
            def set_speed_map(self, s): self.manual_speed = s
            def set_rotate_map(self, r): self.manual_rotate = r
            def get_distance(self, mm): self.target_distance_mm = mm
            def set_remaining_distance(self, mm): self.remaining_distance_mm = mm
            def start_distance_tracing(self, speed_sel): pass
            prev_encoder_left = 0
            prev_encoder_right = 0
            traveled_mm = 0
            target_distance_mm = 0
            remaining_distance_mm = 0
            manual_drive = 0
        return _DummyMotor()

    def _make_dummy_tracer(self):
        class _DummyTracer:
            auto_move = 0
            no_line = False
            target_distance_mm = 0
            remaining_distance_mm = 0
            traveled_mm = 0
            prev_encoder_left = 0
            prev_encoder_right = 0
            def __init__(self): 
                class _M: 
                    def send_speeds(self, l, r): pass
                    @property
                    def get_mode(self): return "manual"
                self.motor = _M()
            def set_drive_mode(self, m): self.auto_move = int(m)
            def stop_line_tracing(self): self.auto_move = 0
            def start_basic_tracing(self, s): pass
            def start_distance_tracing(self, s): pass
            def get_distance(self, mm): self.target_distance_mm = int(mm)
            def set_remaining_distance(self, mm): self.remaining_distance_mm = int(mm)
            def update_encoder(self, l, r): pass
            def process_frame(self, frame): return frame, None
        return _DummyTracer()

    def _make_dummy_lidar(self):
        class _DummyLiDAR:
            def read_frame(self): return None
            def reset_baseline(self): pass
            def pause(self): pass
            def resume(self): pass
            def close(self): pass
        return _DummyLiDAR()

    def _make_dummy_encoder(self):
        class _DummyENC:
            def main(self):
                while True: time.sleep(1)
            def read_counts(self): return (0, 0)
            def reset_counter(self): pass
            def stop(self): pass
        return _DummyENC()

# ──────────────────────────────────────────────────────────────────────────────
# Client Session
# ──────────────────────────────────────────────────────────────────────────────
class ClientSession:
    """
    각 클라이언트 연결마다 독립 스레드로:
      - reader_loop: 소켓에서 8바이트 프레임 수신 → queue
      - dispatch_loop: queue를 해석/처리 (모드/주행/거리/상태/전원/라이다 on/off)
      - lidar_loop: 요청 시 라이다 패킷 스트리밍
      - motor_status_loop: 0xA2 상태 프레임 주기 전송
    """
    def __init__(self, cfg: Config, devices: SystemDevices, conn: socket.socket, addr: Tuple[str, int]):
        self.cfg = cfg
        self.devices = devices
        self.conn = conn
        self.addr = addr

        self._send_lock = threading.Lock()
        self._alive = threading.Event()
        self._alive.set()

        self._param_q: "Queue[bytes]" = Queue()
        self._lidar_stream_enabled = threading.Event()
        self._lidar_lock = threading.Lock()

        self._servo_ready = False

        # 세션 지역 상태
        self._set_mode: Optional[int] = None   # 0=수동, 1=자동
        self._last_status_call: Optional[int] = None
        self._last_dist_set: int = 0
        self._last_emitted_state: Optional[int] = 0   # 처음 STOP은 보내지 않음
        self._ever_emitted = False

        # 스레드
        # self._threads: list[threading.Thread] = []
        self._threads: List[threading.Thread] = []

    # ── lifecycle ────────────────────────────────────────────────────────────
    def start(self):
        self.conn.settimeout(10)
        # TCP keepalive
        self.conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, self.cfg.ka_idle)
        self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, self.cfg.ka_intvl)
        self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, self.cfg.ka_cnt)

        t_disp   = threading.Thread(target=self._dispatch_loop, name=f"dispatch-{self.addr}", daemon=True)
        t_reader = threading.Thread(target=self._reader_loop, name=f"reader-{self.addr}", daemon=True)
        t_watch  = threading.Thread(target=self._motion_watch_loop, name=f"motionwatch-{self.addr}", daemon=True)

        t_disp.start()
        t_reader.start()
        t_watch.start()

        self._threads.extend([t_disp, t_reader, t_watch])
        # for t in self._threads:
        #     t.start()

        print(f"[CTRL] Connected: {self.addr}")

    def _ensure_servo_on(self):
        if self._servo_ready:
            return
        try:
            if DEBUG:
                print("[DISP] servo_on ...")
            self.devices.motor.servo_on()
            self._servo_ready = True
            if DEBUG:
                print("[DISP] servo_on ok")
        except Exception as e:
            print(f"[MOTOR] servo_on error: {e}")

    def close(self):
        if not self._alive.is_set():
            return
        self._alive.clear()

        # 세션 종료 시 안전 정지
        try:
            self.devices.tracer.auto_move = 0
            self.devices.motor.stop()
        except Exception:
            pass

        # LiDAR 스트림 중지
        with self._lidar_lock:
            self._lidar_stream_enabled.clear()

        try:
            self.conn.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        try:
            self.conn.close()
        except Exception:
            pass

        print(f"[CTRL] Disconnected: {self.addr}")

    # ── safe send ────────────────────────────────────────────────────────────
    def _send(self, data: bytes):
        if not self._alive.is_set():
            return
        with self._send_lock:
            try:
                if DEBUG: print(f"[TX>] {_hx(data)}")
                # self.conn.settimeout(1.0)           # 1초 가드
                self.conn.sendall(data)
                if DEBUG: print(f"[TX<] OK")
            except socket.timeout:
                print("[TX] timeout (client not reading?)")
                # 세션은 유지
            except Exception as e:
                print(f"[CTRL] send error to {self.addr}: {e}")

    # ── loops ────────────────────────────────────────────────────────────────
    def _reader_loop(self):
        while self._alive.is_set():
            try:
                frame = recv_exact(self.conn, self.cfg.param_size)
                if frame is None:
                    # 연결 종료 신호만 큐에 넣고 루프 종료
                    self._param_q.put(None)         # 센티넬
                    break

                # checksum 검사
                if (sum(frame[:7]) & 0xFF) != frame[7]:
                    print(f"[RECV] bad checksum: {_hx(frame)}")
                    continue

                self._param_q.put(frame)

            except socket.timeout:
                # 세션 유지
                continue
            except Exception as e:
                print(f"[CTRL] Reader error {self.addr}: {e}")
                self._param_q.put(None)             # 에러 끊김 신호
                break

    def _dispatch_loop(self):
        # print(f"[DISP] start {self.addr}")
        try:
            motor = self.devices.motor
            tracer = self.devices.tracer
            encoder = self.devices.encoder
            lidar = self.devices.lidar
            # print(f"[DISP] devices ok for {self.addr}")

            # try:
            #     encoder.reset_counter()
            # except Exception as e:
            #     print(f"[ENCODER] reset_counter error: {e}")

            tracer.traveled_mm = 0
            print("[DISP] init done, entering loop")

            while self._alive.is_set():
                # print("1")
                try:
                    param = self._param_q.get(timeout=0.5)
                except Empty:
                    # 타임아웃 - 정상적인 상황, 계속 진행
                    continue
                except Exception as e:
                    if DEBUG:
                        print(f"[DISP] Queue get error: {e}")
                    continue

                # 연결 종료 신호 확인
                if param is None:
                    if DEBUG:
                        print("[DISP] Received disconnect signal")
                    break

                try:
                    if DEBUG:
                        if not isinstance(param, (bytes, bytearray)) or len(param) < 8:
                            print(f"[DISP] Invalid param: type={type(param)}, len={len(param) if hasattr(param, '__len__') else 'unknown'}")
                            continue
                        
                    b0, b1, b2, b3, status_call, sbc_power, _, _ = param
                except (ValueError, TypeError) as e:
                    if DEBUG:
                        print(f"[DISP] Invalid param format: {e}, data={param}")
                    continue
                # b0, b1, b2, b3, status_call, sbc_power, _, _ = param
                drive     = None
                speed_sel = None
                rotate    = None
                hi=mid=lo = 0

                if b0 == 0:        # 수동
                    drive, speed_sel, rotate = b1, b2, b3
                elif b0 == 1:      # 자동
                    drive, speed_sel, rotate = b1, b2, 0
                elif b0 == 0x02:   # 거리 입력
                    hi, mid, lo = b1, b2, b3

                if DEBUG:
                    print(f"[DISP] b0={b0:#04x} drive/speed/rot={drive}/{speed_sel}/{rotate} "
                        f"status={status_call} power={sbc_power}")

                # ── 모드 전환
                try:
                    if b0 == 0:
                        self._set_mode = 0
                        tracer.prev_encoder_left = tracer.prev_encoder_right = 0
                        motor.get_drive(drive or 0)
                        motor.set_manual_mode()
                        encoder.reset_counter()
                        if DEBUG:
                            print("[DISP] -> manual mode")
                    elif b0 == 1:
                        self._set_mode = 1
                        motor.prev_encoder_left = motor.prev_encoder_right = 0
                        encoder.reset_counter()
                        motor.set_auto_mode()
                        if DEBUG:
                            print("[DISP] -> auto mode")

                    # ── 거리 입력/초기화
                    if self._set_mode == 0:
                        # 수동 모드일 때 최신 인덱스 반드시 반영 (안 하면 0rpm)
                        if speed_sel is not None:
                            motor.set_speed_map(speed_sel)
                        if rotate is not None:
                            motor.set_rotate_map(rotate)
                        # if (drive == 0) and (rotate is not None) and int(rotate) != 0:
                        #     rot_idx = int(rotate)
                        #     state = 1 if 4 <= rot_idx <= 6 else 2  # 좌(4~6)=FWD, 우(1~3)=BWD
                        #     self._emit_motor_and_battery(state)
                        # else:
                        #     pass
                        if b0 == 0x02:
                            meters = hi
                            hundredths = mid
                            thousandths = (lo // 10)
                            target_mm = meters * 1000 + hundredths * 10 + thousandths
                            motor.get_distance(target_mm)
                            motor.set_remaining_distance(target_mm)
                            motor.traveled_mm = 0
                            motor.prev_encoder_left = motor.prev_encoder_right = 0
                            encoder.reset_counter()
                            self._last_dist_set = target_mm
                            if DEBUG:
                                print(f"[DISP] manual set distance {target_mm}mm")
                            continue

                        if b0 == 0x03:
                            motor.get_distance(0)
                            motor.set_remaining_distance(0)
                            motor.traveled_mm = 0
                            motor.prev_encoder_left = motor.prev_encoder_right = 0
                            encoder.reset_counter()
                            self._last_dist_set = 0
                            if DEBUG:
                                print("[DISP] manual distance clear")
                            continue

                        # 수동 주행
                        if drive == 0:
                            motor.get_drive(0)
                            if DEBUG:
                                print("[DISP] manual drive=0 -> EMIT STOP")
                            self._emit_motor_and_battery(0)
                        else:
                            self._ensure_servo_on()
                            # try:
                            #     from rs485_motor import MotorController as _MC
                            #     spmap = getattr(_MC, "SPEED_MAP", {1:250,2:500,3:750,4:1000,5:1250})
                            #     base = spmap.get(int(speed_sel or 1), 250)
                            #     l_test = -base if drive == 1 else base
                            #     r_test =  base if drive == 1 else -base
                            #     if DEBUG:
                            #         print(f"[DISP] manual drive={drive} TEST send_speeds L={l_test} R={r_test}")
                            #     motor.send_speeds(l_test, r_test)    # 강제 1회 패킷
                            # except Exception as e:
                            #     print(f"[DISP] test send_speeds error: {e}")

                            # 정상 경로
                            motor.get_drive(drive)
                            if DEBUG:
                                print(f"[DISP] manual get_drive({drive}) -> EMIT { 'FWD' if drive==1 else 'REV' }")
                            # self._emit_motor_and_battery(1 if drive == 1 else 2)
                            self._emit_motor_and_battery(1 if drive == 1 else 2)
                            if motor.target_distance_mm > 0:
                                remaining = motor.remaining_distance_mm
                                motor.start_distance_tracing(speed_sel=speed_sel)
                                if remaining > 0:
                                    pass
                                elif self._last_dist_set != motor.target_distance_mm:
                                    motor.start_distance_tracing(speed_sel=speed_sel)
                                else:
                                    motor.get_drive(drive)
                            else:
                                motor.get_drive(drive)

                    elif self._set_mode == 1:
                        if b0 == 0x02:
                            meters = hi
                            hundredths = mid
                            thousandths = (lo // 10)
                            target_mm = meters * 1000 + hundredths * 10 + thousandths
                            tracer.get_distance(target_mm)
                            tracer.set_remaining_distance(target_mm)
                            tracer.traveled_mm = 0
                            tracer.prev_encoder_left = tracer.prev_encoder_right = 0
                            encoder.reset_counter()
                            self._last_dist_set = target_mm
                            if DEBUG:
                                print(f"[DISP] auto set distance {target_mm}mm")
                            continue

                        if b0 == 0x03:
                            tracer.get_distance(0)
                            tracer.set_remaining_distance(0)
                            tracer.traveled_mm = 0
                            tracer.prev_encoder_left = tracer.prev_encoder_right = 0
                            encoder.reset_counter()
                            self._last_dist_set = 0
                            if DEBUG:
                                print("[DISP] auto distance clear")
                            continue

                        if drive == 0:
                            tracer.set_drive_mode(0)
                            tracer.stop_line_tracing()
                            tracer.start_distance_tracing(0)
                            tracer.start_basic_tracing(0)
                            self._emit_motor_and_battery(0)
                            if tracer.target_distance_mm > 0:
                                traveled = tracer.traveled_mm
                                remaining = max(0, tracer.target_distance_mm - traveled)
                                tracer.set_remaining_distance(remaining)
                        elif drive == 1:
                            self._ensure_servo_on()
                            tracer.set_drive_mode(1)
                            if tracer.no_line:
                                self._send(bytes([0xA1, 1, 4, 0]))
                                self._emit_motor_and_battery(0)
                            self._emit_motor_and_battery(1)
                            if tracer.target_distance_mm > 0:
                                tracer.start_distance_tracing(speed_sel=speed_sel)
                            else:
                                tracer.start_basic_tracing(speed_sel=speed_sel)

                        elif drive == 2:
                            self._ensure_servo_on()
                            tracer.set_drive_mode(2)
                            if tracer.no_line:
                                self._send(bytes([0xA1, 1, 4, 0]))
                                self._emit_motor_and_battery(0)
                            self._emit_motor_and_battery(2)
                            # 목표 거리 값 있으면 거리 라인트레이싱, 없으면 기본 라인트레이싱
                            if tracer.target_distance_mm > 0:
                                tracer.start_distance_tracing(speed_sel=speed_sel)
                            else:
                                tracer.start_basic_tracing(speed_sel=speed_sel)

                    # ── 상태/라이다/전원
                    if status_call == 0:
                        self._handle_agv_status_request()
                        self._last_status_call = 0
                    elif status_call == 1 and self._last_status_call != 1:
                        if DEBUG:
                            print("[LIDAR] on")
                        with self._lidar_lock:
                            self._lidar_stream_enabled.set()
                        self._ensure_lidar_thread()
                        try: lidar.resume()
                        except Exception: pass
                        self._last_status_call = 1
                    elif status_call == 2 and self._last_status_call != 2:
                        if DEBUG:
                            print("[LIDAR] off")
                        with self._lidar_lock:
                            self._lidar_stream_enabled.clear()
                        try: lidar.pause()
                        except Exception: pass
                        self._last_status_call = 2
                    elif status_call == 3 and self._last_status_call != 3:
                        if DEBUG:
                            print("[LIDAR] reset baseline")
                        try: lidar.reset_baseline()
                        except Exception: pass
                        self._last_status_call = 3

                    if sbc_power == 1:
                        # self._send(b"ACK:AGV down\n")
                        os.system("sudo shutdown now")

                except Exception as e:
                    print(f"[DISP] Error processing command b0={b0:#04x}: {e}")
                    import traceback
                    traceback.print_exc()
                    # 개별 명령 처리 오류가 발생해도 계속 실행

        except Exception as e:
            print(f"[DISP] Fatal error in dispatch loop: {e}")
            import traceback
            traceback.print_exc()
        finally:
            print(f"[DISP] dispatch loop ended for {self.addr}")
            self.close()

    def _emit_motor_and_battery(self, state: int):
        # 처음 들어오는 STOP(0)은 억제: 레거시와 동일하게 "명령 후 변화"만 송신
        if self._ever_emitted is False and state == 0:
            self._last_emitted_state = 0
            return

        if state == self._last_emitted_state:
            return

        # 여기서부터 실제 송신
        self._ever_emitted = True
        self._send(bytes([0xA2, state & 0xFF, 0x00, 0x00]))
        last_adc = self.devices.get_last_adc()
        soc = pin_voltage_to_soc(last_adc)
        soc_byte = 0 if soc is None else int(max(0, min(9, soc)))
        self._send(bytes([0xA1, 0x02, 0x00, soc_byte]))
        self._last_emitted_state = state

    def _ensure_lidar_thread(self):
        # 이미 실행 중이면 생략
        for t in self._threads:
            if t.name.startswith(f"lidar-{self.addr}"):
                if t.is_alive():
                    return
        t_lidar = threading.Thread(target=self._lidar_loop, name=f"lidar-{self.addr}", daemon=True)
        self._threads.append(t_lidar)
        t_lidar.start()

    def _lidar_loop(self):
        lidar = self.devices.lidar
        while self._alive.is_set():
            with self._lidar_lock:
                if not self._lidar_stream_enabled.is_set():
                    break
            try:
                pkt = lidar.read_frame()
                if pkt:
                    self._send(pkt)
            except (BrokenPipeError, ConnectionResetError):
                print("[LIDAR] client disconnected; stop stream")
                break
            except Exception as e:
                print(f"[LIDAR] stream error: {e}")
            time.sleep(0.5)

    def _motion_watch_loop(self):
        """
        인코더 펄스 변화를 100ms 간격으로 관찰하여
        정지<->전진/후진 전환 '시점에만' 1회 전송.
        """
        encoder = self.devices.encoder
        motor   = self.devices.motor
        prev_left, prev_right = 0, 0
        THRESH_PULSES = 10

        while self._alive.is_set():
            try:
                cur_left, cur_right = encoder.read_counts()
            except Exception:
                break

            d_left  = cur_left  - prev_left
            d_right = cur_right - prev_right
            prev_left, prev_right = cur_left, cur_right

            mag = abs(d_left) + abs(d_right)  # 방향 무관한 '움직임' 크기

            # 모드/회전 인덱스 확인
            try:
                mode = motor.get_mode  # @property
            except Exception:
                mode = ""
            rot_idx = int(getattr(motor, "manual_rotate", 0))

            if mode == "manual" and rot_idx != 0:
                # 회전 중: 움직임이면 FWD(1), 아니면 STOP(0)
                state = 1 if mag >= THRESH_PULSES else 0
            else:
                # 일반: 움직임 없으면 STOP, 있으면 좌/우휠 합으로 전/후진 판정
                if mag < THRESH_PULSES:
                    state = 0
                else:
                    avg = (d_left + d_right) / 2
                    state = 1 if avg > 0 else 2

            # 현재 상태 추정
            # if abs(d_left) + abs(d_right) < THRESH_PULSES:
            #     state = 0
            # else:
            #     avg = (d_left + d_right) / 2
            #     state = 1 if avg > 0 else 2

            # 변화가 있을 때만 내보냄
            self._emit_motor_and_battery(state)
            time.sleep(0.10)

    # ── utilities ─────────────────────────────────────────────────────────────
    def _handle_agv_status_request(self):
        motor = self.devices.motor
        lidar = self.devices.lidar
        encoder = self.devices.encoder

        pico_st = 0 if encoder is not None else 1
        motor_st = 0 if getattr(motor, "is_open", False) else 1
        lidar_st = 0 if lidar is not None else 1

        if pico_st == 0 and motor_st == 0 and lidar_st == 0:
            self._send(bytes([0xA1, 0, 0, 0]))          # 정상
        else:
            if pico_st == 1:
                self._send(bytes([0xA1, 1, 1, 0]))      # pico 오류
            elif motor_st == 1:
                self._send(bytes([0xA1, 1, 2, 0]))      # RS485 오류
            elif lidar_st == 1:
                self._send(bytes([0xA1, 1, 3, 0]))      # LiDAR 오류)

# ──────────────────────────────────────────────────────────────────────────────
# Camera worker (글로벌 1개)
# ──────────────────────────────────────────────────────────────────────────────

def camera_worker(cfg: Config, devices: SystemDevices, alive: threading.Event):
    tracer = devices.tracer
    cap = cv2.VideoCapture(cfg.camera_device)
    if not cap.isOpened():
        print("[CAM] Cannot open camera")
        return
    try:
        while alive.is_set():
            ok, frame = cap.read()
            if not ok:
                print("[CAM] Frame grab failed")
                break

            # 자동 모드가 아니면 안전 정지
            if tracer.auto_move != 1 and getattr(tracer.motor, "get_mode", "") == "auto":
                tracer.motor.send_speeds(0, 0)

            # 프레임 처리(표시는 비활성화)
            try:
                tracer.process_frame(frame)
            except Exception:
                pass
    finally:
        cap.release()

# ──────────────────────────────────────────────────────────────────────────────
# Control Server (accept loop)
# ──────────────────────────────────────────────────────────────────────────────

class ControlServer:
    def __init__(self, cfg: Config, devices: SystemDevices):
        self.cfg = cfg
        self.devices = devices
        self._sock: Optional[socket.socket] = None
        self._alive = threading.Event()
        self._alive.set()
        self._sessions_lock = threading.Lock()
        # self._sessions: set[ClientSession] = set()
        self._sessions: Set["ClientSession"] = set()

    def start(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, self.cfg.ka_idle)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, self.cfg.ka_intvl)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, self.cfg.ka_cnt)

        s.bind((self.cfg.host, self.cfg.port))
        s.listen(2)
        self._sock = s
        print(f"[CTRL] Listening on {self.cfg.port}")

        while self._alive.is_set():
            try:
                conn, addr = s.accept()
            except OSError:
                break
            sess = ClientSession(self.cfg, self.devices, conn, addr)
            with self._sessions_lock:
                self._sessions.add(sess)
            sess.start()

    def stop(self):
        self._alive.clear()
        try:
            if self._sock:
                self._sock.close()
        except Exception:
            pass
        with self._sessions_lock:
            for sess in list(self._sessions):
                try:
                    sess.close()
                except Exception:
                    pass
            self._sessions.clear()

# ──────────────────────────────────────────────────────────────────────────────
# main
# ──────────────────────────────────────────────────────────────────────────────

def main():
    print("[BOOT] starting server process"); sys.stdout.flush()
    cfg = Config()
    try:
        devices = SystemDevices(cfg)
    except Exception:
        traceback.print_exc()
        return

    # Encoder thread
    t_enc = threading.Thread(target=devices.encoder.main, name="encoder", daemon=True)
    t_enc.start()

    server = ControlServer(cfg, devices)
    print("[BOOT] starting TCP server thread"); sys.stdout.flush()
    t_srv = threading.Thread(target=server.start, name="server", daemon=True)
    t_srv.start()

    alive = threading.Event()
    alive.set()
    print("[BOOT] starting camera thread"); sys.stdout.flush()
    t_cam = threading.Thread(target=camera_worker, args=(cfg, devices, alive), name="camera", daemon=True)
    t_cam.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[SYS] Shutting down...")
    finally:
        alive.clear()
        server.stop()
        try:
            devices.motor.stop()
            devices.motor.servo_off()
        except Exception:
            pass
        try:
            devices.encoder.stop()
        except Exception:
            pass
        try:
            devices.lidar.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
