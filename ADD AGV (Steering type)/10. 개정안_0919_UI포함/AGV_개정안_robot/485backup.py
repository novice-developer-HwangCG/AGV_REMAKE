#!/usr/bin/env python3
# rs485_motor.py
# from __future__ import annotations

import threading
import time
import struct
import serial
import math
import threading, time, struct, serial, math
from typing import Optional

class MotorController:
    """
    RS-485(Modbus RTU) 모터 제어기.

    외부에서 사용하는 공개 API (서버와 호환):
      - properties: is_open, get_mode
      - servo_on(), servo_off(), stop()
      - send_speeds(left_rpm, right_rpm)        # 자동(라인트레이싱)에서 직접 속도 지정
      - set_manual_mode(), set_auto_mode()
      - get_drive(drive), set_speed_map(sel), set_rotate_map(sel)
      - get_distance(mm), set_remaining_distance(mm), start_distance_tracing(speed_sel)
      - update_encoder_manual(left_pulse, right_pulse)

    동작 개요:
      - 'manual' 모드에서는 내부 송신 스레드(_tx_loop)가 SPEED_MAP / ROTATE_MAP과
        거리 기반 감속(decel_scale)을 적용해 바퀴 속도(RPM)를 계산·전송한다.
      - 'auto'  모드에서는 외부(LineTracer)가 send_speeds()로 직접 속도를 지정한다.
      - 모든 전송은 디듀프(같은 값 재전송 방지)와 최소 주기(전송율 제한)를 적용한다.
    """

    # ── 맵/상수(기존 값 유지) ────────────────────────────────────────────────
    ROTATE_MAP = {
        0: 0,
        1: 45,
        2: 75,
        3: 0,
        4: 45,
        5: 75,
        6: 0
    }

    SPEED_MAP = {
        0: 0,
        1: 250,
        2: 500,
        3: 750,
        4: 1000,
        5: 1250,
        6: 0,
        7: 0,
        8: 0,
        9: 0
    }

    KP_MAP = {  # 현재 내부 사용은 없지만 호환성 위해 유지
        0: 0.0,
        1: 0.12,
        2: 0.03,
        3: 0.03,
        4: 0.024,
        5: 0.016
    }

    DECEL_ZONE = {  # mm
        0: 0,
        1: 300,
        2: 300,
        3: 750,
        4: 1000,
        5: 1000,
        6: 0,
        7: 0,
        8: 0,
        9: 0
    }

    MIN_DECEL_SCALE = {
        0: 0,
        1: 0.5,
        2: 0.5,
        3: 0.4,
        4: 0.4,
        5: 0.35,
        6: 0,
        7: 0,
        8: 0,
        9: 0
    }

    FORCE_STOP_THRESH = {  # mm
        0: 0,
        1: 25,
        2: 25,
        3: 50,
        4: 100,
        5: 100,
        6: 0,
        7: 0,
        8: 0,
        9: 0
    }

    # 엔코더 펄스→mm 환산 (로봇에 맞춰 튜닝)
    PULSES_TO_MM = 0.03795

    # Modbus RTU: 슬레이브/레지스터 정의(기본값 유지)
    SLAVE_LEFT  = 1
    SLAVE_RIGHT = 2
    REG_SERVO   = 0x0078
    REG_SPEED   = 0x0079
    FN_WRITE    = 6

    # 전송 제한(Hz) / 간격(s)
    MIN_TX_INTERVAL_S = 0.01  # 10 ms

    def __init__(self, port: str = "/dev/ttyRS485", baudrate: int = 115200, timeout: float = 1.0, debug: bool = False):
        self.debug = debug

        # ── 직렬 포트 ──────────────────────────────────────────────────────
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)

        # ── 로봇 파라미터 (참조용) ─────────────────────────────────────────
        self.W_TRACK = 0.57
        self.W_THICK = 0.05
        self.W_EFF   = self.W_TRACK + self.W_THICK
        self.WHEEL_D = 0.12
        self.MS2RPM  = 60.0 / (math.pi * self.WHEEL_D)  # 참고용(내부 직접 사용 X)
        self.BASE_SPEED_MPS = 0.3
        self.K_OMEGA       = 0.0063
        self.MAX_RPM       = 2500  # 드라이버 최대치(논리상)

        # ── 엔코더/거리 상태 ───────────────────────────────────────────────
        self.encoder_left = 0
        self.encoder_right = 0
        self.prev_encoder_left = 0
        self.prev_encoder_right = 0

        self.traveled_mm = 0.0
        self.target_distance_mm = 0.0
        self.remaining_distance_mm = 0
        self.decel_scale = 1.0

        # 수동 거리주행 튜닝 인덱스
        self.manual_decel_zone = 0
        self.manual_min_decel_scale = 0
        self.manual_force_stop_thresh = 0

        # ── 모드/수동 명령 상태 ───────────────────────────────────────────
        self.mode: Optional[str] = None  # 'manual' | 'auto' | None
        self.manual_speed = 0
        self.manual_rotate = 0
        self.manual_drive = 0
        self.last_rotate = 0
        self.manual_kp = 0
        self.KD = 0.0025  # (현재 내부 미사용, 호환성 유지)

        # ── 송신 값/락/스레드 ─────────────────────────────────────────────
        self.lock = threading.RLock()
        self.left_speed = 0
        self.right_speed = 0

        self._alive = threading.Event()
        self._alive.set()

        self._last_tx_l = None
        self._last_tx_r = None
        self._last_tx_t = 0.0

        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True, name="rs485-tx")
        self._tx_thread.start()

    # ── Properties ───────────────────────────────────────────────────────────
    @property
    def is_open(self) -> bool:
        return bool(self.ser and self.ser.is_open)

    @property
    def get_mode(self) -> Optional[str]:
        with self.lock:
            return self.mode

    # ── Modbus helpers ───────────────────────────────────────────────────────
    @staticmethod
    def _calc_crc(data: bytes) -> bytes:
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, "little")

    def _build_packet(self, slave_id: int, function: int, address: int, value: int) -> bytes:
        if value < 0:
            value = (1 << 16) + value  # 16-bit two's complement
        body = struct.pack(">B B H H", slave_id, function, address, value)
        return body + self._calc_crc(body)

    def _write_reg(self, slave_id: int, address: int, value: int):
        pkt = self._build_packet(slave_id, self.FN_WRITE, address, int(value))
        self._send_packet(pkt)

    def _send_packet(self, packet: bytes):
        # write 자체는 짧음. 슬립은 락 밖에서 수행해 버스 점유 최소화
        try:
            with self.lock:
                if self.debug:
                    print(f"[RS485 TX] {packet.hex()}")
                self.ser.write(packet)
        except Exception as e:
            if self.debug:
                print(f"[RS485] write error: {e}")
        # 짧은 프레임 간격(버스 여유) - 락 밖에서 대기
        time.sleep(0.005)

    # ── Servo / Stop ─────────────────────────────────────────────────────────
    def servo_on(self):
        # self._write_reg(self.SLAVE_LEFT,  self.REG_SERVO, 1)
        # time.sleep(0.03)
        # self._write_reg(self.SLAVE_RIGHT, self.REG_SERVO, 1)
        # 두 패스 반복: L→R, L→R (유실 대비)
        for _ in range(2):
            self._write_reg(self.SLAVE_LEFT,  self.REG_SERVO, 1)
            time.sleep(0.01)
            self._write_reg(self.SLAVE_RIGHT, self.REG_SERVO, 1)
            time.sleep(0.01)
        # 켠 직후 양쪽 0rpm을 써서 버스/드라이버를 '워밍업'
        # self._write_reg(self.SLAVE_LEFT,  self.REG_SPEED, 0)
        # time.sleep(0.01)
        # self._write_reg(self.SLAVE_RIGHT, self.REG_SPEED, 0)

    def servo_off(self):
        self._write_reg(self.SLAVE_LEFT,  self.REG_SERVO, 0)
        time.sleep(0.03)
        self._write_reg(self.SLAVE_RIGHT, self.REG_SERVO, 0)

    def stop(self):
        if self.debug:
            print("[RS485] STOP")
        self._write_reg(self.SLAVE_LEFT,  self.REG_SPEED, 0)
        time.sleep(0.03)
        self._write_reg(self.SLAVE_RIGHT, self.REG_SPEED, 0)
        with self.lock:
            self.left_speed = 0
            self.right_speed = 0
            self._last_tx_l = 0
            self._last_tx_r = 0

    # ── Mode & manual controls ───────────────────────────────────────────────
    def set_manual_mode(self):
        with self.lock:
            self.mode = "manual"

    def set_auto_mode(self):
        with self.lock:
            self.mode = "auto"

    def get_drive(self, drive: int):
        with self.lock:
            self.manual_drive = int(drive)

    def set_speed_map(self, speed_sel: int):
        with self.lock:
            self.manual_speed = int(speed_sel)

    def set_rotate_map(self, rotate_sel: int):
        with self.lock:
            self.manual_rotate = int(rotate_sel)

    # ── Distance / encoder integration (manual distance driving) ─────────────
    def get_distance(self, mm: int):
        # 새 거리 입력 시에만 traveled 초기화
        if mm != self.remaining_distance_mm:
            self.traveled_mm = 0.0
        self.target_distance_mm = float(mm)
        if self.debug:
            print(f"[MANUAL] target_distance_mm = {self.target_distance_mm:.1f}")

    def set_remaining_distance(self, remaining_mm: int):
        self.remaining_distance_mm = max(0, int(remaining_mm))
        if self.debug:
            print(f"[MANUAL] remaining_distance_mm = {self.remaining_distance_mm}")

    def start_distance_tracing(self, speed_sel: int):
        """
        거리 기반 주행에서 사용할 매핑 인덱스 지정(감속영역/하한/강제정지 임계치).
        """
        with self.lock:
            self.manual_speed = int(speed_sel)
            self.manual_kp = int(speed_sel)
            self.manual_decel_zone = int(speed_sel)
            self.manual_min_decel_scale = int(speed_sel)
            self.manual_force_stop_thresh = int(speed_sel)

    def update_encoder_manual(self, left_pulse: int, right_pulse: int):
        """
        인코더 누적 카운트를 주기적으로 넣어주면, 이동거리/감속스케일/목표도달 판정을 갱신한다.
        (자동 모드에서는 LineTracer 쪽 로직이 담당하므로 수동 거리 주행일 때 사용)
        """
        self.encoder_left = int(left_pulse)
        self.encoder_right = int(right_pulse)

        delta_left = self.encoder_left - self.prev_encoder_left
        delta_right = self.encoder_right - self.prev_encoder_right
        self.prev_encoder_left = self.encoder_left
        self.prev_encoder_right = self.encoder_right

        dist_left_mm = delta_left * self.PULSES_TO_MM
        dist_right_mm = delta_right * self.PULSES_TO_MM
        avg_mm   = (dist_left_mm + dist_right_mm) / 2.0
        step_mm  = abs(avg_mm)
        self.traveled_mm += step_mm

        # if self.traveled_mm < 0:
        #     self.traveled_mm = -1

        if self.debug:
            print(f"[ENC] traveled={self.traveled_mm:.1f} / target={self.target_distance_mm:.1f}")

        # 감속/강제정지 로직
        if self.target_distance_mm > 0:
            remaining = self.target_distance_mm - self.traveled_mm

            dz = self.DECEL_ZONE.get(self.manual_decel_zone, 0)
            min_scale = self.MIN_DECEL_SCALE.get(self.manual_min_decel_scale, 1.0)
            force_thr = self.FORCE_STOP_THRESH.get(self.manual_force_stop_thresh, 0)

            if 0 < remaining <= dz and dz > 0:
                scale = remaining / dz
                self.decel_scale = max(min_scale, min(1.0, scale))
            else:
                self.decel_scale = 1.0

            if remaining < force_thr:
                if self.debug:
                    print(f"[MANUAL] force stop (remaining {remaining:.1f} mm)")
                self.stop()
                with self.lock:
                    self.manual_drive = 0
                self._clear_distance_targets()
                return
        else:
            self.decel_scale = 1.0

        # 정지 후 재시작 시 남은 거리 재적용
        if self.manual_drive in (1,2) and getattr(self, "remaining_distance_mm", 0) > 0:
            self.target_distance_mm = float(self.remaining_distance_mm)
            self.traveled_mm = 0.0
            if self.debug:
                print(f"[MANUAL] resume distance: {self.target_distance_mm:.1f} mm")
            self.remaining_distance_mm = 0

        # 목표 도달
        if self.target_distance_mm > 0 and self.traveled_mm >= self.target_distance_mm:
            if self.debug:
                print(f"[MANUAL] reach target: {self.target_distance_mm:.1f} mm")
            self.stop()
            with self.lock:
                self.manual_drive = 0
            self._clear_distance_targets()

    def _clear_distance_targets(self):
        self.target_distance_mm = 0.0
        self.traveled_mm = 0.0
        self.remaining_distance_mm = 0

    # ── 외부에서 직접 속도 지정(자동 모드) ─────────────────────────────────
    def send_speeds(self, left_rpm: int, right_rpm: int):
        """
        LineTracer가 호출. 내부에 디듀프/레이트 제한 포함.
        """
        t = time.monotonic()
        # 1) 전송 여부/값 결정은 락 안에서
        with self.lock:
            l = int(left_rpm); r = int(right_rpm)
            self.left_speed = l
            self.right_speed = r
            if (t - self._last_tx_t) < self.MIN_TX_INTERVAL_S:
                return
            if (self._last_tx_l, self._last_tx_r) == (l, r):
                return
        # print(f"L :{l},       R : {r}")
        # 2) 실제 쓰기(버스)는 락 밖에서
        self._write_reg(self.SLAVE_LEFT,  self.REG_SPEED, l)
        time.sleep(0.03)
        self._write_reg(self.SLAVE_RIGHT, self.REG_SPEED, r)
        # 3) 마지막 전송값 갱신은 다시 락 안에서
        with self.lock:
            self._last_tx_l = l
            self._last_tx_r = r
            self._last_tx_t = t

    # ── 내부 송신 스레드 (manual 모드용) ────────────────────────────────────
    def _tx_loop(self):
        """
        manual 모드일 때만 SPEED_MAP/ROTATE_MAP/거리감속을 적용하여 바퀴 속도를 갱신·전송.
        auto 모드에서는 외부 send_speeds()가 전송을 담당한다.
        """
        # while self._alive.is_set():
        #     with self.lock:
        #         mode = self.mode
        #         ls, rs = self.left_speed, self.right_speed
        while self._alive.is_set():
            # 1) 현재 모드/명령을 기반으로 목표 속도 산출 (락 안)
            with self.lock:
                mode = self.mode
                ls = self.left_speed
                rs = self.right_speed

                if mode == "manual":
                    rotate_idx = self.manual_rotate
                    speed_idx = self.manual_speed
                    
                    if self.manual_drive == 0:
                        ls = 0
                        rs = 0

                    elif rotate_idx != 0:
                        # turn_rpm = self.ROTATE_MAP.get(rotate_idx, 0)
                        turn_rpm = self.ROTATE_MAP[rotate_idx]
                        if 1 <= rotate_idx <= 3:
                            # 제자리 우회전(기구/배선 기준에 맞춰 동일 부호 사용)
                            ls =  turn_rpm
                            rs =  turn_rpm
                        elif 4 <= rotate_idx <= 6:
                            # 제자리 좌회전
                            ls = -turn_rpm
                            rs = -turn_rpm

                    elif self.target_distance_mm > 0:
                        base = self.SPEED_MAP.get(speed_idx, 0)
                        scaled = int(base * self.decel_scale)
                        if self.manual_drive == 1:   # 전진
                            ls = -scaled
                            rs =  scaled
                        elif self.manual_drive == 2: # 후진
                            ls =  scaled
                            rs = -scaled

                    else:
                        base = self.SPEED_MAP.get(speed_idx, 0)
                        if self.manual_drive == 1:
                            ls = -base
                            rs =  base
                        elif self.manual_drive == 2:
                            ls =  base
                            rs = -base

                    self.last_rotate = rotate_idx
                    self.left_speed, self.right_speed = int(ls), int(rs)

                # auto 모드: 이 루프에서 건드리지 않음(외부 send_speeds 사용)

                # 전송 디듀프/주기 관리
                t = time.monotonic()
                need_send = False
                l_to_send = self.left_speed
                r_to_send = self.right_speed
                if (t - self._last_tx_t) >= self.MIN_TX_INTERVAL_S:
                    if (self._last_tx_l, self._last_tx_r) != (l_to_send, r_to_send):
                        need_send = True

            if need_send:
                self._write_reg(self.SLAVE_LEFT,  self.REG_SPEED, l_to_send)
                time.sleep(0.03)
                self._write_reg(self.SLAVE_RIGHT, self.REG_SPEED, r_to_send)
                with self.lock:
                    self._last_tx_l = l_to_send
                    self._last_tx_r = r_to_send
                    self._last_tx_t = t

            time.sleep(0.005)

    # ── 종료 훅 ──────────────────────────────────────────────────────────────
    def close(self):
        self._alive.clear()
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
