#!/usr/bin/env python3
import threading
import time
import serial
import struct
import numpy as np

class MotorController:
    """
    RS-485 모터 제어기.
    - 수동 모드: 명령 받은 제어 값에 따라 _run() 스레드에서 속도를 조절하여 전송
    - 자동 모드: 라인트레이싱 모드 (LineTracer가 계산한 속도를 직접 send_speeds()로 전송)
    """

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

    KP_MAP = {
        0: 0.0,
        1: 0.12,
        2: 0.03,
        3: 0.03,
        4: 0.024,
        5: 0.016
    }
    DECEL_ZONE = {
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
    FORCE_STOP_THRESH = {
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
    
    PULSES_TO_MM = 0.03795

    def __init__(self, port='/dev/ttyRS485', baudrate=115200, timeout=1):
        # --- 직렬 포트 초기화 ---
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)

        # 로봇 파라미터 (linetracing 계산에서 필요한 상수들을 노출)
        self.W_TRACK = 0.57      # m    차륜궤간
        self.W_THICK = 0.05      # m    휠 두께
        self.W_EFF   = self.W_TRACK + self.W_THICK      # 실효 가로폭 -> 회전 시 실제 회전반경, 모터 배치 여유 등을 계산할 때 사용
        self.WHEEL_D = 0.12      # m    휠 지름
        self.MS2RPM  = 60 / (np.pi * self.WHEEL_D)  # m/s → RPM 변환 계수, 예비용 사용 안함
        self.BASE_SPEED_MPS = 0.3    # 베이스 속도 0.3m/s
        self.K_OMEGA       = 0.0063  # 픽셀 오차 → rad/s 변환 이득 / 목표물 과의 상대 위치 계산 좌우 시야 중심으로부터 x 픽셀 만큼 어긋났으니 그만큼 회전하여 중앙에 맞추라는 의미 / 테스트 후 튜닝 필요

        # RS-485 모터 최대 RPM (기어비 10:1 적용으로 인해해 모터에 적용될 실제제 rpm은 250rpm, 그 때 최대 선속도는 약 1.57m/s)
        self.MAX_RPM = 2500

        self.encoder_left = 0
        self.encoder_right = 0

        self.prev_encoder_left = 0
        self.prev_encoder_right = 0

        self.traveled_mm = 0.0
        self.target_distance_mm = 0.0
        self.remaining_distance_mm = 0

        self.decel_scale = 1.0
        self.manual_decel_zone = 0
        self.manual_min_decel_scale = 0
        self.manual_force_stop_thresh = 0

        # --- 모드 관련 변수 ---
        # mode: 'manual' 또는 'auto'
        self.mode = None
        self.manual_speed = 0
        self.manual_rotate = 0
        self.manual_drive = 0
        self.last_rotate = 0
        # --- 동기화 도구 & 내부 변수 ---
        self.lock = threading.Lock()
        # 보내 주어야 할 최종 바퀴 속도(RPM)
        self.left_speed  = 0
        self.right_speed = 0

        self.KD = 0.0025

        # RS-485 패킷 송신 초기화
        # self.servo_on()
        # --- 백그라운드 전송 스레드 시작 ---
        threading.Thread(target=self._run, daemon=True).start()

    @property
    def is_open(self):
        return self.ser.is_open

    @property
    def get_mode(self):
        with self.lock:
            return self.mode

    # --- RS-485 패킷 CRC 계산 ---
    def _calc_crc(self, data: bytes) -> bytes:
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if (crc & 1):
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, byteorder='little')

    # --- Modbus RTU 패킷 생성 ---
    def _build_packet(self, slave_id: int, function: int, address: int, value: int) -> bytes:
        if value < 0:
            # 음수 표현을 16비트 2의 보수로 변환
            value = (1 << 16) + value
        packet = struct.pack('>B B H H', slave_id, function, address, value)
        return packet + self._calc_crc(packet)

    # --- 패킷을 UART로 전송 ---
    def _send_packet(self, packet: bytes):
        """
        :param packet: RS-485(Modbus RTU) 패킷
        """
        with self.lock:
            # print(f"[RS485 SEND] {packet.hex()}")
            self.ser.write(packet)
            time.sleep(0.005)  # 짧은 지연

    # --- 서보 온/오프 ---
    def servo_on(self):
        pkt_l = self._build_packet(1, 6, 0x0078, 1)
        pkt_r = self._build_packet(2, 6, 0x0078, 1)
        self._send_packet(pkt_l)
        time.sleep(0.03)
        self._send_packet(pkt_r)

    def servo_off(self):
        pkt_l = self._build_packet(1, 6, 0x0078, 0)
        pkt_r = self._build_packet(2, 6, 0x0078, 0)
        self._send_packet(pkt_l)
        time.sleep(0.03)
        self._send_packet(pkt_r)

    def stop(self):
        print("[RS485] Sending STOP packet")
        pkt_l = self._build_packet(1, 6, 0x0079, 0)
        pkt_r = self._build_packet(2, 6, 0x0079, 0)
        self._send_packet(pkt_l)
        time.sleep(0.03)
        self._send_packet(pkt_r)

    def send_speeds(self, left_rpm: int, right_rpm: int):
        with self.lock:
            self.left_speed  = left_rpm
            self.right_speed = right_rpm
        now = time.time()
        if now - getattr(self, "last_send_time", 0) < 0.01:  # 최소 50ms 간격
            return
        if getattr(self, "last_sent", (None, None)) == (left_rpm, right_rpm):
            return  # 같은 값이면 무시
        self.last_sent = (left_rpm, right_rpm)
        self.last_send_time = now
        print(f"[RS485] Sending Speeds L={left_rpm}, R={right_rpm}")
        pkt_l = self._build_packet(1, 6, 0x0079, left_rpm)
        pkt_r = self._build_packet(2, 6, 0x0079, right_rpm)
        self._send_packet(pkt_l)
        time.sleep(0.03)
        self._send_packet(pkt_r)
    
    def set_manual_mode(self):
        with self.lock:
            self.mode = 'manual'

    def set_auto_mode(self):
        with self.lock:
            self.mode = 'auto'

    def get_drive(self, drive):
        self.manual_drive = drive

    def set_speed_map(self, speed_sel):
        self.manual_speed = speed_sel
        #print(f"[MOTOR]manual_speed : {self.manual_speed}")
    
    def set_rotate_map(self, rotate):
        self.manual_rotate = rotate
        # print(f"[MOTOR]manual_rotate : {self.manual_rotate}")

    def get_distance(self, mm):
        if mm != self.remaining_distance_mm:
            self.traveled_mm = 0  # 새 거리 입력 시에만 초기화
        self.target_distance_mm = mm
        print(f"[LineTracer] target_distance_mm → {self.target_distance_mm} mm")

    def set_remaining_distance(self, remaining_mm):
        """jetson_server.py에서 넘겨준 남은 거리 값 저장"""
        self.remaining_distance_mm = max(0, remaining_mm)
        print(f"[LineTracer] remaining_distance_mm → {self.remaining_distance_mm} mm")

    def start_distance_tracing(self, speed_sel):
        # 거리 입력 기반 라인 트레이싱 일 때 speed_sel은 jetson_sever.py로부터 받아서 사용하기
        self.manual_speed = speed_sel
        self.manual_kp = speed_sel
        self.manual_decel_zone = speed_sel
        self.manual_min_decel_scale = speed_sel
        self.manual_force_stop_thresh = speed_sel

    def update_encoder_manual(self, left_pulse, right_pulse):
        self.encoder_left  = left_pulse
        self.encoder_right = right_pulse
    
        delta_left  = self.encoder_left  - self.prev_encoder_left
        delta_right = self.encoder_right - self.prev_encoder_right

        self.prev_encoder_left  = self.encoder_left
        self.prev_encoder_right = self.encoder_right

        dist_left_mm  = delta_left  * self.PULSES_TO_MM
        dist_right_mm = delta_right * self.PULSES_TO_MM

        # 5) 이동한 거리 누적
        self.traveled_mm += (dist_left_mm + dist_right_mm) / 2.0
        print(f"[ENC] traveled_mm: {self.traveled_mm:.1f} / target: {self.target_distance_mm:.1f}")

        # ——— 감속 구간 로직 + 강제 도달 임계 추가 ———
        if self.target_distance_mm > 0:
            remaining = self.target_distance_mm - self.traveled_mm

            GET_DECEL_ZONE = self.DECEL_ZONE[self.manual_decel_zone]
            GET_MIN_DECEL_SCALE = self.MIN_DECEL_SCALE[self.manual_min_decel_scale]
            GET_FORCE_STOP_THRESH = self.FORCE_STOP_THRESH[self.manual_force_stop_thresh]

            if 0 < remaining <= GET_DECEL_ZONE:
                scale = remaining / GET_DECEL_ZONE
                # 감속 스케일 하한 적용
                self.decel_scale = max(GET_MIN_DECEL_SCALE, min(1.0, scale))
            else:
                self.decel_scale = 1.0
            # ----- 강제 정지 임계 -----
            if remaining < GET_FORCE_STOP_THRESH:
                print(f"[LineTracer] Reach Target (force stop, {remaining:.1f}mm left)")
                self.stop()
                self.manual_drive = 0
                self.target_distance_mm = 0
                self.traveled_mm = 0
                self.remaining_distance_mm = 0
                return  # 더 처리할 것 없이 즉시 종료
        else:
            self.decel_scale = 1.0

        # 주행 중 정지 후 다시 주행 명령을 내릴 때 남은 거리 값이 있을 시 그 거리 값으로 재주행 
        if (self.manual_drive == 1           # 1) 다시 주행 명령이 들어온 상태이고
           and getattr(self, 'remaining_distance_mm', 0) > 0):      # 2) 남은 거리 값이 있다면
            # 남은 거리로 재설정
            self.target_distance_mm = self.remaining_distance_mm
            # 이미 traveled_mm 은 0 이거나 “멈춘 시점” 값이므로 다시 0으로 초기화
            self.traveled_mm = 0.0
            print(f"[LineTracer] Resuming distance: {self.target_distance_mm} mm left")
            # 한 번 적용했으면 남은 거리는 초기화
            self.remaining_distance_mm = 0

        # 6) 목표 도달 시
        if self.target_distance_mm > 0 and self.traveled_mm >= self.target_distance_mm:
            print(f"[LineTracer] Reach Target Distance: {self.target_distance_mm}mm")
            self.stop()
            self.manual_drive = 0
            self.target_distance_mm = 0
            self.traveled_mm = 0
            self.remaining_distance_mm = 0

    # --- 백그라운드 스레드: 모터 속도 업데이트 ---
    def _run(self):
        """
        별도 데몬 스레드로 실행됩니다. 수동 모드, 엔코더 모드, 라인트레이싱 모드에 따라
        left_speed, right_speed를 결정하고, 실제 패킷을 주기적으로 전송합니다.
        """
        while True:
            with self.lock:
                # 1) 수동 모드
                mode = self.mode
                if mode == 'manual':
                    # 키/회전 우선, drive_flag는 완전히 무시
                    rotate_idx = self.manual_rotate    # 0–6
                    speed_idx  = self.manual_speed     # 0–9
                    # drive 값을 0을 받을 시 정지 시키고 남아있는 값들도 0으로 초기화
                    # 1) drive==0 → 무조건 정지 (회전도 금지)
                    if self.manual_drive == 0:
                        self.left_speed  = 0
                        self.right_speed = 0

                    # 주행 명령을 받았는데 거리 값이 있다면 거리 값을 우선 처리
                    elif self.target_distance_mm > 0:
                        # 1) 원래 속도값 (RPM) 가져오기
                        target_rpm = self.SPEED_MAP[speed_idx]
                        # 2) update_encoder_manual() 에서 갱신된 self.decel_scale 을 곱해 감속 적용
                        scaled_rpm = int(target_rpm * self.decel_scale)

                        # 3) 전진/후진에 따라 양쪽 바퀴 속도 지정
                        if self.manual_drive == 1:    # 전진
                            self.left_speed  = -scaled_rpm
                            self.right_speed =  scaled_rpm
                        elif self.manual_drive == 2:  # 후진
                            self.left_speed  =  scaled_rpm
                            self.right_speed = -scaled_rpm

                    # 2) drive!=0 이고 거리 값은 없을 때 rotate 명령이 있으면 회전
                    elif rotate_idx != 0:
                        turn_rpm = self.ROTATE_MAP[rotate_idx]
                        if 1 <= rotate_idx <= 3:
                            # 제자리 우회전
                            self.left_speed  =  turn_rpm
                            self.right_speed =  turn_rpm
                        elif 4<= rotate_idx <=6:
                            # 제자리 좌회전
                            self.left_speed  = -turn_rpm
                            self.right_speed = -turn_rpm

                    # 3) drive!=0 이고 rotate_idx==0, 거리 값은 없을 때 → 전진/후진
                    else:
                        target = self.SPEED_MAP[speed_idx]
                        if self.manual_drive == 1:    # 전진
                            self.left_speed  = -target
                            self.right_speed =  target
                        elif self.manual_drive == 2:  # 후진
                            self.left_speed  =  target
                            self.right_speed = -target

                    # 변경값 보존
                    self.last_rotate = rotate_idx
                # 2) 자동 모드 → 엔코더 기반 거리 값 주행 or 라인트레이싱 해당 부분은 linetracing.py가 send_speed()함수에게 속도값을 직접 전달하므로 pass
                elif mode == 'auto':
                    #print("auto")
                    pass
                ls = int(self.left_speed)
                rs = int(self.right_speed)
            # 두 바퀴 속도를 각각 RS-485 패킷으로 만들어 전송
            pkt_l = self._build_packet(1, 6, 0x0079, ls)
            pkt_r = self._build_packet(2, 6, 0x0079, rs)
            self._send_packet(pkt_l)
            self._send_packet(pkt_r)

            # 너무 빠르지 않게 10ms 간격을 둡니다
            time.sleep(0.01)
