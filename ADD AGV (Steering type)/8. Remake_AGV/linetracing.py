import cv2
import numpy as np
import collections
from rs485_motor import MotorController

class LineTracer:
    # --- Image‑processing params ---
    ROI_Y = 240
    ROI_H = 240          # use lower half of 640×480
    CLAHE_CLIP = 2.0
    CLAHE_GRID = (8, 8)
    GAUSS_KERNEL = (5, 5)
    MORPH_KERNEL = (5, 5)
    MORPH_ITERS = 2

    # contour filter
    MIN_AREA = 350
    MAX_AREA = 50_000
    ASPECT_MIN = 0.2
    ASPECT_MAX = 5.0
    DIST_THRESH = 100                 # px jump threshold when tracking
    # speed selector (UI 0‑9 → RPM)
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
        2: 0.03,       # 0.030
        3: 0.03,       # 0.026 증가해서 다시
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

    PULSES_TO_MM = 0.03795            # encoder scale
    # PD gains
    ACCEL_STEP = 40                   # rpm/frame limit

    DEAD_PIX = 2

    def __init__(self, motor: MotorController):
        self.motor = motor
        self.encoder_left = 0
        self.encoder_right = 0

        self.prev_encoder_left = 0
        self.prev_encoder_right = 0

        self.traveled_mm = 0.0
        self.target_distance_mm = 0.0
        self.remaining_distance_mm = 0

        self.decel_scale = 1.0
        self.auto_move = 0          # 0=stop, 1=forward
        self.auto_speed = 0         # UI speed idx
        self.auto_decel_zone = 0
        self.auto_min_decel_scale = 0
        self.auto_force_stop_thresh = 0
        self.no_line = False

        self.clahe = cv2.createCLAHE(self.CLAHE_CLIP, self.CLAHE_GRID)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, self.MORPH_KERNEL)
        self.initial_center_set = False
        self.pre_cx = None
        self.pre_cy = None

        self.prev_error = 0.0
        self.prev_L = 0
        self.prev_R = 0

        self.KD = 0.0025

        self.last_mode = self.motor.get_mode

    # ---------------- UI hooks ----------------
    def preprocess(self, roi):
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = self.clahe.apply(gray)
        mean_val = gray.mean()            # float
        offset = 65
        thre = max(0, min(255, mean_val - offset))
        _, binary = cv2.threshold(gray, int(thre), 255, cv2.THRESH_BINARY_INV)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, self.kernel, iterations=self.MORPH_ITERS)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self.kernel, iterations=self.MORPH_ITERS)
        return binary

    def filter_contours(self, contours):
        res = []
        for c in contours:
            a = cv2.contourArea(c)
            if not (self.MIN_AREA < a < self.MAX_AREA):
                continue
            x, y, w, h = cv2.boundingRect(c)
            aspect = h / float(w) if w else 0
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

        best, min_d = None, float('inf')

        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            cx, cy = x + w // 2, y + h // 2
            d = np.hypot(cx - self.pre_cx, cy - self.pre_cy)

            if d < min_d:
                best, min_d = c, d

        return best if best is not None and min_d <= self.DIST_THRESH else None

    def visualize(self, img, cnt):
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cx, cy = x + w // 2, y + h // 2
        cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
        return cx

    def set_drive_mode(self, mode: int):
        self.auto_move = max(0, min(1, mode))
        self.prev_L = 0
        self.prev_R = 0
        if self.auto_move == 0:
            self.motor.stop()

    def get_distance(self, mm):
        if mm != self.remaining_distance_mm:
            self.traveled_mm = 0  # 새 거리 입력 시에만 초기화
        self.target_distance_mm = mm
        print(f"[LineTracer] target_distance_mm → {self.target_distance_mm} mm")

    def set_remaining_distance(self, remaining_mm):
        """jetson_server.py에서 넘겨준 남은 거리 값 저장"""
        self.remaining_distance_mm = max(0, remaining_mm)
        print(f"[LineTracer] remaining_distance_mm → {self.remaining_distance_mm} mm")

    def start_basic_tracing(self, speed_sel):
        # 기본 라인 트레이싱 일 때 speed_sel은 jetson_sever.py로부터 받아서 사용하기
        self.auto_speed = speed_sel
        self.auto_kp = speed_sel
        self.auto_decel_zone = speed_sel
        self.auto_min_decel_scale = speed_sel
        self.auto_force_stop_thresh = speed_sel
        self.no_line = False

    def start_distance_tracing(self, speed_sel):
        # 거리 입력 기반 라인 트레이싱 일 때 speed_sel은 jetson_sever.py로부터 받아서 사용하기
        self.auto_speed = speed_sel
        self.auto_kp = speed_sel
        self.auto_decel_zone = speed_sel
        self.auto_min_decel_scale = speed_sel
        self.auto_force_stop_thresh = speed_sel
        self.no_line = False

    def stop_line_tracing(self):
        # print("Stop_Line_Tracing")
        self.prev_L = 0
        self.prev_R = 0
        self.auto_move = 0
        self.motor.stop()

    def update_encoder(self, left_pulse, right_pulse):
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

            GET_DECEL_ZONE = self.DECEL_ZONE[self.auto_decel_zone]
            GET_MIN_DECEL_SCALE = self.MIN_DECEL_SCALE[self.auto_min_decel_scale]
            GET_FORCE_STOP_THRESH = self.FORCE_STOP_THRESH[self.auto_force_stop_thresh]

            if 0 < remaining <= GET_DECEL_ZONE:
                scale = remaining / GET_DECEL_ZONE
                # 감속 스케일 하한 적용
                self.decel_scale = max(GET_MIN_DECEL_SCALE, min(1.0, scale))
            else:
                self.decel_scale = 1.0
            # ----- 강제 정지 임계 -----
            if remaining < GET_FORCE_STOP_THRESH:
                print(f"[LineTracer] Reach Target (force stop, {remaining:.1f}mm left)")
                self.motor.stop()
                self.auto_move = 0
                self.target_distance_mm = 0
                self.traveled_mm = 0
                self.remaining_distance_mm = 0
                return  # 더 처리할 것 없이 즉시 종료
        else:
            self.decel_scale = 1.0

        # 주행 중 정지 후 다시 주행 명령을 내릴 때 남은 거리 값이 있을 시 그 거리 값으로 재주행 
        if (self.auto_move == 1           # 1) 다시 주행 명령이 들어온 상태이고
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
            self.motor.stop()
            self.auto_move = 0
            self.target_distance_mm = 0
            self.traveled_mm = 0
            self.remaining_distance_mm = 0

    def process_frame(self, frame):
        h, w = frame.shape[:2]
        roi = frame[self.ROI_Y:self.ROI_Y + self.ROI_H, :]
        binary = self.preprocess(roi)
        _, cnts, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = self.filter_contours(cnts)
        best = self.select_contour(cnts)
        cx = None
        vis = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        # cx = self.visualize(vis, best) if best is not None else None

        send_L = 0
        send_R = 0

        # if cx is not None:
        #     self.pre_cx = cx

        if best is not None:
            x,y,wc,hc = cv2.boundingRect(best)
            cx,cy = x+wc//2, y+hc//2
            cv2.rectangle(vis,(x,y),(x+wc,y+hc),(0,255,0),2)
            cv2.circle(vis,(cx,cy),5,(0,0,255),-1)
            self.pre_cx = cx

        # (A) 자동 모드가 시작된 순간 prev_L/R 리셋, self.motor.get_mode변수 property 방식으로 받음
        if self.last_mode != self.motor.get_mode and self.motor.get_mode == 'auto':
            self.prev_L = 0
            self.prev_R = 0
            # print("[DEBUG] Entered auto mode → prev_L/R reset")
        self.last_mode = self.motor.get_mode
        # print(f"get mode: {self.last_mode}")

        if self.motor.get_mode == 'auto':
            # print(f"auto_speed : {self.auto_speed}")
            #print("auto")
            if self.auto_move == 0:
                #print("0")
                self.motor.send_speeds(0, 0)
                # self.motor.stop()
                self.prev_L = 0
                self.prev_R = 0
                # dynamic_step = max(abs(self.prev_L), abs(self.prev_R), self.ACCEL_STEP)
                return binary, cx
            else:
                #print("1")
                
                if cx is None:
                    print("none")
                    if self.target_distance_mm > 0:
                        remaining = max(0.0, self.target_distance_mm - self.traveled_mm)
                        self.set_remaining_distance(remaining)
                    self.no_line = True
                    self.auto_move = 0
                    self.motor.send_speeds(0, 0)
                    return binary, cx

                error_px = w//2 - cx

                if abs(error_px) <= self.DEAD_PIX:
                    error_px = 0

                error = error_px / (w//2)

                if self.auto_move == 2:
                    error = -error

                speed_scale = (self.auto_speed+1)/10  # 0.1~1.0

                get_kp = self.KP_MAP[self.auto_kp]
                
                Kp = get_kp * (1 + speed_scale)  # 빠를수록 gain ↑
                Kd = self.KD * (1 + speed_scale)
                steer = Kp*error + Kd*(error - self.prev_error)
                self.prev_error = error
                # 에러가 클수록 기본 속도 줄이기
                vel_scale = max(0.4, 1.0 - min(abs(error),1.0))
                base = self.SPEED_MAP[self.auto_speed] * vel_scale

                L = base - steer*base
                R = base + steer*base

                if self.target_distance_mm > 0:
                    L = int(L*self.decel_scale)
                    R = int(R*self.decel_scale)

                maxd = self.SPEED_MAP[self.auto_speed]

                L = int(max(-maxd,min(maxd,L)))
                R = int(max(-maxd,min(maxd,R)))

                L = self.prev_L + int(np.clip(L-self.prev_L,-self.ACCEL_STEP,self.ACCEL_STEP))
                R = self.prev_R + int(np.clip(R-self.prev_R,-self.ACCEL_STEP,self.ACCEL_STEP))

                self.prev_L = L
                self.prev_R = R

                if self.auto_move == 1:       # forward
                    send_L =-L
                    send_R = R
                elif self.auto_move == 2:
                    send_L = L
                    send_R = -R                    

                #print(f"send_speed L, R : {send_L}, {send_R}")
                self.motor.send_speeds(int(send_L), int(send_R))
                return binary, cx
        else:
            return binary, cx