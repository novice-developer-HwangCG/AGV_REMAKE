# linetracing.py
# from __future__ import annotations

import cv2
import numpy as np
from typing import Optional, Tuple
import collections

from rs485_motor import MotorController


class LineTracer:
    """
    카메라 프레임으로 라인을 추적해 좌/우 바퀴 RPM을 산출하고,
    MotorController.send_speeds()로 전송한다.

    공개 API(서버/엔코더에서 사용):
      - set_drive_mode(mode:int)              # 0=stop, 1=forward, 2=reverse
      - start_basic_tracing(speed_sel:int)
      - start_distance_tracing(speed_sel:int)
      - stop_line_tracing()
      - get_distance(mm:int)
      - set_remaining_distance(mm:int)
      - update_encoder(left_pulse:int, right_pulse:int)
      - process_frame(frame) -> (binary, cx)

    외부 참조 필드(서버가 읽음):
      - auto_move, no_line
      - target_distance_mm, remaining_distance_mm, traveled_mm
      - prev_encoder_left, prev_encoder_right
    """

    # --- 이미지 전처리 파라미터 ---
    ROI_Y = 240
    ROI_H = 240
    CLAHE_CLIP = 2.0
    CLAHE_GRID = (8, 8)
    GAUSS_KERNEL = (5, 5)
    MORPH_KERNEL = (5, 5)
    MORPH_ITERS = 2

    # --- 컨투어 필터 ---
    MIN_AREA = 350
    MAX_AREA = 50_000
    ASPECT_MIN = 0.2
    ASPECT_MAX = 5.0
    DIST_THRESH = 100  # tracking jump px

    # --- 속도/튜닝 맵 (UI 0~9) ---
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
        9: 0,
    }
    KP_MAP = {
        0: 0.0,
        1: 0.12,
        2: 0.03,
        3: 0.03,
        4: 0.024,
        5: 0.016,
    }
    KP_BWD_MAP = {
        0: 0.0, 
        1: 0.08,
        2: 0.15,
        3: 0.06,
        4: 0.12,
        5: 0.08,
        6: 0, 
        7: 0, 
        8: 0, 
        9: 0
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
        9: 0,
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
        9: 0,
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
        9: 0,
    }

    # 후진용 추가 설정값
    CAMERA_TO_REAR_DISTANCE = 150  # 카메라에서 후축까지의 거리 (mm)
    BWD_LOOKAHEAD_FACTOR = 0.7     # 후진 시 예측 계수
    BWD_DAMPING_FACTOR = 0.8       # 후진 시 제어 감쇠 계수

    BWD_EDGE_TARGET = 0.60        # 후진 시 먼저 라인을 화면 가장자리 근처까지 보낼 목표 오프셋(|e|≈0.6)
    BWD_SWITCH_TOL_IN = 0.15      # push→pull 전환 임계(타겟 근처에 왔을 때)
    BWD_SWITCH_TOL_OUT = 0.25     # pull→push 복귀 임계(중앙 잡다 다시 벌어지면)

    # 엔코더 펄스→mm 환산 (로봇에 맞춰 유지)
    PULSES_TO_MM = 0.03795

    # 제어 파라미터
    ACCEL_STEP = 40      # rpm/frame limit
    DEAD_PIX = 2
    KD = 0.0025          # D게인 기본값(속도에 따라 스케일)

    def __init__(self, motor: MotorController, debug: bool = False):
        self.debug = debug
        self.motor = motor

        # 엔코더 누적
        self.encoder_left = 0
        self.encoder_right = 0
        self.prev_encoder_left = 0
        self.prev_encoder_right = 0

        # 거리 기반 주행 상태
        self.traveled_mm = 0.0
        self.target_distance_mm = 0.0
        self.remaining_distance_mm = 0
        self.decel_scale = 1.0

        # 자동 주행 상태
        self.auto_move = 0  # 0=stop, 1=forward, 2=reverse
        self.auto_speed = 0
        self.auto_decel_zone = 0
        self.auto_min_decel_scale = 0
        self.auto_force_stop_thresh = 0
        self.no_line = False

        # 영상 처리 유틸
        self._clahe = None
        self._kernel = None
        self.initial_center_set = False
        self.pre_cx = None
        self.pre_cy = None

        # self.clahe = cv2.createCLAHE(self.CLAHE_CLIP, self.CLAHE_GRID)
        # self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, self.MORPH_KERNEL)
        # self.initial_center_set = False
        # self.pre_cx: Optional[int] = None
        # self.pre_cy: Optional[int] = None

        # 제어 누적
        self.auto_kp = 0
        self.prev_error = 0.0
        self.prev_L = 0
        self.prev_R = 0

        # 후진용 제어 변수 추가
        self.bwd_error_buffer = collections.deque(maxlen=3)  # 에러 평활화용
        self.bwd_prev_steer = 0.0  # 이전 조향값 저장
        self.bwd_phase = 0

        # 모드 전환 감지
        self.last_mode = "manual"   # self.motor.get_mode

    # ──────────────────────────────────────────────────────────────────
    # 전처리 & 컨투어 선택
    # ──────────────────────────────────────────────────────────────────
    def _ensure_opencv(self):
        if self._clahe is None:
            try:
                self._clahe = cv2.createCLAHE(self.CLAHE_CLIP, self.CLAHE_GRID)
            except Exception:
                self._clahe = None
        if self._kernel is None:
            self._kernel = cv2.getStructuringElement(cv2.MORPH_RECT, self.MORPH_KERNEL)

    def preprocess(self, roi) -> np.ndarray:
        self._ensure_opencv()
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        if self._clahe is not None:
            gray = self._clahe.apply(gray)

        mean_val = float(gray.mean())
        offset = 65
        thre = int(max(0, min(255, mean_val - offset)))
        _, binary = cv2.threshold(gray, thre, 255, cv2.THRESH_BINARY_INV)

        # 커널 이름 수정 + 커널 없을 때는 형태학 연산 생략
        if self._kernel is not None:
            binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, self._kernel, iterations=self.MORPH_ITERS)
            binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self._kernel, iterations=self.MORPH_ITERS)
        return binary

    # def preprocess(self, roi) -> np.ndarray:
    #     self._ensure_opencv()
    #     gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    #     if self._clahe is not None:
    #         gray = self._clahe.apply(gray)
    #     else:
    #         pass

    #     # gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    #     # gray = self.clahe.apply(gray)
    #     mean_val = float(gray.mean())
    #     # 동적 임계치: 평균 - 오프셋
    #     offset = 65
    #     thre = int(max(0, min(255, mean_val - offset)))
    #     _, binary = cv2.threshold(gray, thre, 255, cv2.THRESH_BINARY_INV)
    #     binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, self.kernel, iterations=self.MORPH_ITERS)
    #     binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self.kernel, iterations=self.MORPH_ITERS)
    #     return binary

    def _find_contours(self, binary: np.ndarray):
        # OpenCV 3/4 호환
        res = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(res) == 2:
            cnts, _ = res
        else:
            _, cnts, _ = res
        return cnts

    def filter_contours(self, contours):
        res = []
        for c in contours:
            a = cv2.contourArea(c)
            if not (self.MIN_AREA < a < self.MAX_AREA):
                continue
            x, y, w, h = cv2.boundingRect(c)
            aspect = (h / float(w)) if w else 0.0
            if self.ASPECT_MIN < aspect < self.ASPECT_MAX:
                res.append(c)
        return res

    def select_contour(self, contours):
        if not self.initial_center_set:
            if contours:
                best = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(best)
                self.pre_cx, self.pre_cy = x + w // 2, y + h // 2
                self.initial_center_set = True
                return best
            return None

        best, min_d = None, float("inf")
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            cx, cy = x + w // 2, y + h // 2
            d = np.hypot(cx - self.pre_cx, cy - self.pre_cy)
            if d < min_d:
                best, min_d = c, d
        return best if (best is not None and min_d <= self.DIST_THRESH) else None

    # ──────────────────────────────────────────────────────────────────
    # 서버/엔코더 훅
    # ──────────────────────────────────────────────────────────────────
    def set_drive_mode(self, mode: int):
        self.auto_move = max(0, min(2, int(mode)))
        self.prev_L = 0
        self.prev_R = 0
        if self.auto_move == 0:
            self.motor.send_speeds(0, 0)
        
        self.bwd_phase = 1 if self.auto_move == 2 else 0

    def get_distance(self, mm: int):
        if mm != self.remaining_distance_mm:
            self.traveled_mm = 0.0
        self.target_distance_mm = int(mm)
        if self.debug:
            print(f"[LineTracer] target_distance_mm → {self.target_distance_mm} mm")

    def set_remaining_distance(self, remaining_mm: int):
        self.remaining_distance_mm = max(0, int(remaining_mm))
        if self.debug:
            print(f"[LineTracer] remaining_distance_mm → {self.remaining_distance_mm} mm")

    def start_basic_tracing(self, speed_sel: int):
        self.auto_speed = int(speed_sel)
        self.auto_kp = int(speed_sel)
        self.auto_decel_zone = int(speed_sel)
        self.auto_min_decel_scale = int(speed_sel)
        self.auto_force_stop_thresh = int(speed_sel)
        self.no_line = False

    def start_distance_tracing(self, speed_sel: int):
        self.auto_speed = int(speed_sel)
        self.auto_kp = int(speed_sel)
        self.auto_decel_zone = int(speed_sel)
        self.auto_min_decel_scale = int(speed_sel)
        self.auto_force_stop_thresh = int(speed_sel)
        self.no_line = False

    def stop_line_tracing(self):
        self.prev_L = 0
        self.prev_R = 0
        self.auto_move = 0
        self.motor.send_speeds(0, 0)

    def update_encoder(self, left_pulse: int, right_pulse: int):
        self.encoder_left = int(left_pulse)
        self.encoder_right = int(right_pulse)

        dL = self.encoder_left - self.prev_encoder_left
        dR = self.encoder_right - self.prev_encoder_right
        self.prev_encoder_left = self.encoder_left
        self.prev_encoder_right = self.encoder_right

        dist_left_mm = dL * self.PULSES_TO_MM
        dist_right_mm = dR * self.PULSES_TO_MM

        step_mm = abs((dist_left_mm + dist_right_mm) / 2.0)
        self.traveled_mm += step_mm

        if self.debug:
            print(f"[ENC] traveled_mm: {self.traveled_mm:.1f} / target: {self.target_distance_mm:.1f}")

        # 거리 기반 감속/정지
        if self.target_distance_mm > 0:
            remaining = self.target_distance_mm - self.traveled_mm

            dz = self.DECEL_ZONE.get(self.auto_decel_zone, 0)
            min_scale = self.MIN_DECEL_SCALE.get(self.auto_min_decel_scale, 1.0)
            force_thr = self.FORCE_STOP_THRESH.get(self.auto_force_stop_thresh, 0)

            if 0 < remaining <= dz and dz > 0:
                scale = remaining / dz
                self.decel_scale = max(min_scale, min(1.0, scale))
            else:
                self.decel_scale = 1.0

            if remaining < force_thr:
                if self.debug:
                    print(f"[LineTracer] Reach Target (force stop, {remaining:.1f}mm left)")
                self.motor.send_speeds(0, 0)
                self.auto_move = 0
                self._clear_distance_targets()
                return
        else:
            self.decel_scale = 1.0

        # 정지 후 재시작 시 남은거리 재적용
        if self.auto_move in (1, 2) and getattr(self, "remaining_distance_mm", 0) > 0:
            self.target_distance_mm = int(self.remaining_distance_mm)
            self.traveled_mm = 0.0
            if self.debug:
                print(f"[LineTracer] Resuming distance: {self.target_distance_mm} mm")
            self.remaining_distance_mm = 0

        # 목표 도달
        if self.target_distance_mm > 0 and self.traveled_mm >= self.target_distance_mm:
            if self.debug:
                print(f"[LineTracer] Reach Target Distance: {self.target_distance_mm} mm")
            self.motor.send_speeds(0, 0)
            self.auto_move = 0
            self._clear_distance_targets()

    def _clear_distance_targets(self):
        self.target_distance_mm = 0
        self.traveled_mm = 0.0
        self.remaining_distance_mm = 0

    # ──────────────────────────────────────────────────────────────────
    # 메인 비전 루프 (카메라 프레임 1장 처리)
    # ──────────────────────────────────────────────────────────────────
    def process_frame(self, frame) -> Tuple[np.ndarray, Optional[int]]:
        """
        Returns:
          binary: 전처리된 바이너리 ROI
          cx:     선택된 라인의 중심 x (없으면 None)
        """
        h, w = frame.shape[:2]
        roi = frame[self.ROI_Y:self.ROI_Y + self.ROI_H, :]
        binary = self.preprocess(roi)

        cnts = self._find_contours(binary)
        cnts = self.filter_contours(cnts)
        best = self.select_contour(cnts)

        cx = None
        # 디버그용 시각화가 필요하면 아래 두 줄 활성화
        # vis = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        # if best is not None: self._draw_box(vis, best)

        if best is not None:
            x, y, wc, hc = cv2.boundingRect(best)
            cx, cy = x + wc // 2, y + hc // 2
            self.pre_cx = cx

        # 모드 전환 감지(수동→자동 진입 시 램프업 초기화)
        try:
            cur_mode = self.motor.get_mode
        except Exception:
            cur_mode = "manual"

        if self.last_mode != cur_mode and cur_mode == "auto":
            self.prev_L = 0
            self.prev_R = 0
        self.last_mode = cur_mode

        if cur_mode != "auto":
            return binary, cx

        # if self.last_mode != self.motor.get_mode and self.motor.get_mode == "auto":
        #     self.prev_L = 0
        #     self.prev_R = 0
        # self.last_mode = self.motor.get_mode

        # 자동 모드가 아니면 제어하지 않음
        # if self.motor.get_mode != "auto":
        #     return binary, cx

        # 자동이지만 정지 상태
        if self.auto_move == 0:
            self.motor.send_speeds(0, 0)
            self.prev_L = 0
            self.prev_R = 0
            return binary, cx

        # 라인 미검출: 남은거리 저장 후 정지
        if cx is None:
            if self.target_distance_mm > 0:
                remaining = max(0.0, self.target_distance_mm - self.traveled_mm)
                self.set_remaining_distance(int(remaining))
            self.no_line = True
            self.auto_move = 0
            self.motor.send_speeds(0, 0)
            return binary, cx

        # 라인 추적 제어
        error_px = (w // 2) - cx
        if abs(error_px) <= self.DEAD_PIX:
            error_px = 0
        error = error_px / float(w // 2)

        base_max = self.SPEED_MAP.get(self.auto_speed, 0)
        speed_scale = (self.auto_speed + 1) / 10.0  # 0.1 ~ 1.0

        # 후진 모드면 조향 반전
        if self.auto_move == 2:
            e = -error
            if self.bwd_phase == 0:
                self.bwd_phase = 1  # 안전장치

            if self.bwd_phase == 1:
                # PUSH: 라인을 화면 가장자리 근처까지 의도적으로 보냄
                # e==0이면 이전 오차 부호를 우선 사용, 둘 다 0이면 + 방향
                side = np.sign(e) if e != 0 else (np.sign(self.prev_error) if self.prev_error != 0 else 1.0)
                target = side * self.BWD_EDGE_TARGET
                e_ctrl = e - target
                e_use  = -e_ctrl                 # 후진은 조향 부호 반전
                if abs(e_ctrl) <= self.BWD_SWITCH_TOL_IN:
                    self.bwd_phase = 2           # 충분히 끝에 붙었으면 중앙 복귀 단계로 전환
            else:
                # PULL: 중앙(0)으로 복귀
                target = 0.0
                e_ctrl = e - target
                e_use  = -e_ctrl                 # 후진은 조향 부호 반전
                if abs(e_ctrl) >= self.BWD_SWITCH_TOL_OUT:
                    self.bwd_phase = 1           # 다시 벌어지면 push로 복귀
            
            self.bwd_error_buffer.append(e)
            e_smooth = sum(self.bwd_error_buffer) / len(self.bwd_error_buffer)

            Kp = self.KP_BWD_MAP.get(self.auto_kp, 0.0)
            Kd = (0.001 if self.auto_speed < 5 else 0.0005) * (1.0 + speed_scale)

            raw_steer = Kp * e_smooth + Kd * (e_smooth - self.prev_error)
            steer = self.BWD_DAMPING_FACTOR * self.bwd_prev_steer + (1.0 - self.BWD_DAMPING_FACTOR) * raw_steer
            self.bwd_prev_steer = steer
            self.prev_error = e_smooth

            # 에러 클수록 보수적으로 감속
            vel_scale = max(0.3 if self.auto_speed < 5 else 0.25,
                            1.0 - (1.2 if self.auto_speed < 5 else 1.5) * min(abs(e_smooth), 1.0))
            accel_step = int(self.ACCEL_STEP * (0.8 if self.auto_speed < 5 else 0.6))
        else:   # 전진은 그대로
            e = error
            Kp = self.KP_MAP.get(self.auto_kp, 0.0) * (1.0 + speed_scale)
            Kd = self.KD * (1.0 + speed_scale)

            steer = Kp * e + Kd * (e - self.prev_error)
            self.prev_error = e

            vel_scale = max(0.4, 1.0 - min(abs(e), 1.0))
            accel_step = self.ACCEL_STEP

        base = base_max * vel_scale

        L = base - steer * base
        R = base + steer * base

        # 거리 기반 감속 적용
        if self.target_distance_mm > 0:
            L *= self.decel_scale
            R *= self.decel_scale

        # RPM 클램프
        L = int(max(-base_max, min(base_max, int(L))))
        R = int(max(-base_max, min(base_max, int(R))))

        # 가속 제한(램프업/다운)
        L = self.prev_L + int(np.clip(L - self.prev_L, -accel_step, accel_step))
        R = self.prev_R + int(np.clip(R - self.prev_R, -accel_step, accel_step))
        self.prev_L, self.prev_R = L, R

        # 구동 방향에 맞춰 부호 적용(전진: L 음, R 양)
        if self.auto_move == 1:      # forward
            send_L = -L
            send_R = R
        else:                        # reverse
            send_L = L
            send_R = -R

        self.motor.send_speeds(int(send_L), int(send_R))
        return binary, cx

    # 디버그 시각화 박스 (option)
    @staticmethod
    def _draw_box(img, cnt):
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cx, cy = x + w // 2, y + h // 2
        cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
