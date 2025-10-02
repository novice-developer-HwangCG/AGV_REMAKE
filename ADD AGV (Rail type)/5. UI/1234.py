#!/usr/bin/env python3
import os
import sys, socket, threading, time
from PyQt5.QtWidgets import (
    QApplication, QWidget, QGridLayout, QFrame,
    QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QTextEdit,
    QSizePolicy, QCheckBox, qApp
)

from PyQt5.QtGui import QIcon, QPixmap
from PyQt5 import QtGui
from PyQt5.QtCore import Qt

PICO_IP        = "192.168.100.10"
PICO_CTRL_PORT = 5024  # pico 에 맞춤

# 이 스크립트(.py) 파일이 있는 폴더 경로
base_dir = os.path.dirname(os.path.abspath(__file__))

class ControlUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AGV Debug Client")
        icon_path = os.path.join(base_dir, 'development.png')
        self.setWindowIcon(QIcon(icon_path))
        self.setFixedSize(1000,800)

        # 상태 변수 (Byte0~Byte6)   
        self.mode         = 1
        self.drive        = 0
        self.speed        = 0
        self.rsv_1        = 0
        self.status_call  = 0
        self.power_call   = 0
        self.reserved     = 0

        self.buffer = bytearray()
        # 소켓
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((PICO_IP, PICO_CTRL_PORT))
        except Exception as e:
            print(f"[Error] connect: {e}")

        self._build_ui()
        self._start_reader()

    def _build_ui(self):
        grid = QGridLayout(self)
        grid.setContentsMargins(16,16,16,16)
        grid.setSpacing(12)

        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        grid.setRowStretch(0, 1)
        grid.setRowStretch(1, 1)

        # 왼쪽 상단(0,0)에 로고 삽입
        logo = QLabel()
        logo.setAlignment(Qt.AlignCenter)
        # QLabel 크기를 최대 200×200으로 고정
        max_w, max_h = 420, 500
        logo.setFixedSize(max_w, max_h)
        logo_path = os.path.join(base_dir, "logo_ADD.png")
        pix = QPixmap(logo_path)
        logo.setPixmap(pix.scaled(max_w, max_h, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        grid.addWidget(logo, 0, 0, 1, 1, Qt.AlignCenter)
        
        # Mode
        ctrl_frame = QFrame()
        ctrl_frame.setFrameShape(QFrame.StyledPanel)
        ctrl_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding) 
        cl = QVBoxLayout(ctrl_frame)
        cl.setContentsMargins(8,8,8,8)
        cl.setSpacing(8)

        # Mode + 스위치
        h = QHBoxLayout()
        self.btn_auto = QPushButton("자동조작"); self.btn_auto.setObjectName("btnAuto")
        self.btn_man  = QPushButton("수동조작"); self.btn_man .setObjectName("btnMan")
        for b in (self.btn_auto, self.btn_man):
            b.setCheckable(True)
        self.btn_auto.setChecked(True)
        h.addWidget(self.btn_auto); h.addWidget(self.btn_man)
        h.addStretch()
        self.switch = QCheckBox(objectName="toggleSwitch")
        self.switch.setFixedSize(50,25); self.switch.setChecked(True)
        h.addWidget(self.switch)
        cl.addLayout(h)
        
        self.switch.toggled.connect(lambda checked: self._update('mode', 1 if checked else 0))
        # h.addWidget(self.btn_auto); h.addWidget(self.btn_man)
        # # main.addLayout(h)

        # Drive
        ## h = QHBoxLayout()
        ## self.btn_stop = QPushButton("정지")
        h = QHBoxLayout()

        self.btn_stop = QPushButton("⏹️정지")
        self.btn_stop.setObjectName("btnDrive")
        
        self.btn_fwd = QPushButton("⬆️ 전진")
        self.btn_fwd.setObjectName("btnDrive")
        
        self.btn_bwd = QPushButton("⬇️후진")
        self.btn_bwd.setObjectName("btnDrive")
        for b in (self.btn_stop, self.btn_fwd, self.btn_bwd):
            b.setCheckable(True)
            b.setMinimumSize(100, 40)
            b.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        for b in (self.btn_stop, self.btn_fwd, self.btn_bwd):
            b.setCheckable(True)
        self.btn_stop.setChecked(True)
        ## h.addWidget(self.btn_stop); h.addWidget(self.btn_fwd); h.addWidget(self.btn_bwd)
        ## main.addLayout(h)
        h.addWidget(self.btn_stop); h.addWidget(self.btn_fwd); h.addWidget(self.btn_bwd)
        cl.addLayout(h)

        # Distance
        h = QHBoxLayout()
        h.addWidget(QLabel("거리 (m):"))
        self.le_dist = QLineEdit()
        self.le_dist.setPlaceholderText("예시: 2.000m")
        self.le_dist.setMinimumSize(100, 40)

        self.btn_send_dist  = QPushButton("전송")
        self.btn_clear_dist = QPushButton("초기화")
        for w in (self.btn_send_dist, self.btn_clear_dist):
            w.setMinimumSize(100, 40)    # 너비100×높이40
            w.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        h.addWidget(self.le_dist); h.addWidget(self.btn_send_dist); h.addWidget(self.btn_clear_dist)
        # main.addLayout(h)
        cl.addLayout(h)

        # SpeedSel
        cl.addWidget(QLabel("SpeedSel (0–9)"))
        sg = QGridLayout()
        self.speed_buttons = []
        for i in range(10):
            btn = QPushButton(str(i))
            btn.setCheckable(True)
            if i == 0:
                btn.setChecked(True)
            btn.setMinimumSize(60, 40)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)                
            self.speed_buttons.append(btn)
            sg.addWidget(btn, i//5, i%5)
            
        cl.addLayout(sg)

        # Status_call
        ## main.addWidget(QLabel("Status Call"))
        ## h = QHBoxLayout()
        cl.addWidget(QLabel("Status Call"))
        h = QHBoxLayout()
        labels = [("AGV Check",0), ("LiDAR On",1), ("LiDAR Off",2), ("LiDAR Reset",3)]
        self.status_buttons = []
        
        for text,val in labels:
            btn = QPushButton(text); btn.setCheckable(True); btn.val=val
            if val==0: btn.setChecked(True)
            
            btn.setMinimumSize(100, 40)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

            self.status_buttons.append(btn); h.addWidget(btn)
        ## main.addLayout(h)
        b_status_send = QPushButton("전송")
        b_status_send.setMinimumSize(100, 40)
        b_status_send.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        h.addWidget(b_status_send)
        cl.addLayout(h)
    
        cl.addWidget(QLabel("Power / ADC Call"))
        h2 = QHBoxLayout()
        labels = [("Null",0),("ADC Call",1)]
        self.power_buttons = []
        for text,val in labels:
            btn = QPushButton(text)
            btn.setCheckable(True)
            btn.val = val
            if val == 0: btn.setChecked(True)
            btn.setMinimumSize(100, 40)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            self.power_buttons.append(btn)
            h2.addWidget(btn)
        cl.addLayout(h2)
        # 컨트롤 패널을 (0,1)에 추가
        grid.addWidget(ctrl_frame, 0, 1)

        # PWR / ADC
        ## main.addWidget(QLabel("Power / ADC Call"))

        # 왼쪽 하단(1,0): Power 로그창
        power_frame = QFrame()
        power_frame.setFrameShape(QFrame.StyledPanel)
        power_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding) #
        power_layout = QVBoxLayout(power_frame)
        power_layout.setContentsMargins(8,8,8,8)
        power_layout.addWidget(QLabel("LIADR Log"))
        self.log_power = QTextEdit()
        self.log_power.setReadOnly(True)
        power_layout.addWidget(self.log_power)
        grid.addWidget(power_frame, 1, 0)

        # 오른쪽 하단(1,1): Control 로그창
        control_frame = QFrame()
        control_frame.setFrameShape(QFrame.StyledPanel)
        control_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding) 
        control_layout = QVBoxLayout(control_frame)
        control_layout.setContentsMargins(8,8,8,8)
        control_layout.addWidget(QLabel("Control Log"))
        self.log_control = QTextEdit()
        self.log_control.setReadOnly(True)
        control_layout.addWidget(self.log_control)
        grid.addWidget(control_frame, 1, 1)
        
        # Bind
        self.btn_auto.clicked.connect(lambda: self._update('mode',1))
        self.btn_man.clicked.connect(lambda: self._update('mode',0))
        self.btn_stop.clicked.connect(lambda: self._update('drive',0))
        self.btn_fwd.clicked.connect(lambda: self._update('drive',1))
        self.btn_bwd.clicked.connect(lambda: self._update('drive',2))
        for btn in self.speed_buttons:
            btn.clicked.connect(lambda _,b=btn: self._update('speed',int(b.text())))
        for btn in self.status_buttons:
            btn.clicked.connect(lambda _,b=btn: self._update('status_call',b.val))
        for btn in self.power_buttons:
            btn.clicked.connect(lambda _,b=btn: self._update('power_call',b.val))
        self.btn_send_dist.clicked.connect(self._send_dist)
        self.btn_clear_dist.clicked.connect(self._clear_dist)

    def _build_frame(self):
        f = bytearray(8)
        f[0]=self.mode; 
        f[1]=self.drive; 
        f[2]=self.speed; 
        f[3]=self.rsv_1
        f[4]=self.status_call; 
        f[5]=self.power_call; 
        f[6]=self.reserved
        f[7]=sum(f[:7])&0xFF
        return f

    def _send_frame(self):
        pkt = self._build_frame()
        try:
            self.sock.sendall(pkt)
            ## self.log.append(f">>> {pkt.hex()}")
            if self.power_call == 1:
                self.log_power.append(f">>> {pkt.hex()}")
            else:
                self.log_control.append(f">>> {pkt.hex()}")
        except Exception as e:
            ## self.log.append(f"[Error] send: {e}")
            self.log_power.append(f"[Error] send: {e}")

    def _update(self, name, val):
        setattr(self, name, val)
        # toggle buttons
        if name=='mode':
            self.btn_auto.setChecked(val==1); self.btn_man.setChecked(val==0)
            self.switch.setChecked(val==1)
        if name=='drive':
            self.btn_stop.setChecked(val==0)
            self.btn_fwd.setChecked(val==1)
            self.btn_bwd.setChecked(val==2)
        if name=='speed':
            for b in self.speed_buttons: b.setChecked(int(b.text())==val)
        if name=='status_call':
            for b in self.status_buttons: b.setChecked(b.val==val)
        if name=='power_call':
            for b in self.power_buttons: b.setChecked(b.val==val)
        self._send_frame()

        # if name == 'power_call':
        #     # Power/ADC Call 로그는 왼쪽에
        #     text = "Null" if val==0 else "ADC Call"
        #     self.log_power.append(f"[Power Call] → {text}")

        # elif name == 'status_call':
        #     # LiDAR On/Off/Reset 로그도 왼쪽에
        #     btn = next(b for b in self.status_buttons if b.val == val)
        #     self.log_power.append(f"[LiDAR Call] → {btn.text()}")

        # else:
        #     # 나머지(모드/Drive/Speed)는 오른쪽에
        #     self.log_control.append(f"[{name}] → {val}")

    def _send_dist(self):
        # txt = self.le_dist.text().strip()
        # if not txt.isdigit(): return
        # mm = int(txt)
        # dist_hi  = (mm // 10000) % 100    # 앞 두 자리
        # dist_mid = (mm // 100) % 100      # 중간 두 자리
        # dist_lo  = mm % 100               # 뒤 두 자리

        # old_speed = self.speed
        # old_drive = self.drive
        # old_rsv_1= self.rsv_1

        # # 2) 거리 전송
        # self.mode = 2
        # self.drive = dist_hi
        # self.speed = dist_mid
        # self.rsv_1 = dist_lo
        # self._send_frame()

        # # 3) 모드 복귀 및 이전 상태 복원
        # self.mode   = 1
        # self.drive  = old_drive
        # self.speed  = old_speed
        # self.rsv_1 = old_rsv_1
        # # (버튼 UI도 다시 체크 상태로 복원)
        # self._update('mode', 1)
        # self._update('drive', old_drive)
        # self._update('speed', old_speed)
        # self._update('rsv_1', old_rsv_1)
        
        txt = self.le_dist.text().strip()
        # 1) 입력을 float(미터)로 파싱
        try:
            meters = float(txt)
        except ValueError:
            return

        # 2) mm 단위로 환산 (예: 1.0 → 1000 mm)
        mm = int(meters * 1000)

        # 3) 기존 매핑 그대로
        dist_hi  = (mm // 10000) % 100    # 앞 두 자리
        dist_mid = (mm // 100)   % 100    # 중간 두 자리
        dist_lo  = mm % 100               # 뒤 두 자리

        # --- frame 전송 ---
        old_speed = self.speed
        old_drive = self.drive
        old_rsv_1 = self.rsv_1

        self.mode  = 2
        self.drive = dist_hi
        self.speed = dist_mid
        self.rsv_1 = dist_lo
        self._send_frame()

        # --- 상태 복원 ---
        self.mode   = 1
        self.drive  = old_drive
        self.speed  = old_speed
        self.rsv_1  = old_rsv_1
        self._update('mode', 1)
        self._update('drive', old_drive)
        self._update('speed', old_speed)
        self._update('rsv_1', old_rsv_1)

    def _clear_dist(self):
        self.mode=3
        self.drive=self.speed=self.rsv_1=self.status_call=self.power_call=self.reserved=0
        self._send_frame()
        self.mode = 1

    def _start_reader(self):
        threading.Thread(target=self._reader,daemon=True).start()

    def _reader(self):
        self.buffer = bytearray()
        while True:
            data = self.sock.recv(64)
            hex_str = ' '.join(f"{b:02X}" for b in data)
            # self.log.append(f"[RESP RAW] {hex_str}")
            
            if not data:
                break
            # 1) 새로 받은 덩어리를 버퍼에 축적
            self.buffer.extend(data)

            # 2) 버퍼에 3바이트 이상 남아 있으면 프레임 단위로 꺼내기
            while len(self.buffer) >= 3:
                hdr = self.buffer[0]
                if hdr == 0x5A:
                    # 거리 확인 응답
                    _, status, hi, md, lo = self.buffer[:5]
                    del self.buffer[:5]
                    # self.log.append(f"[FRAME] LIDAR dist: {hi}.{md:02d}{lo:02d}m (status={status})")
                    self.log_control.append(f"[FRAME] LIDAR dist: {hi}.{md:02d}{lo:02d}m (status={status})")

                elif hdr == 0xB0:
                    _, hi, lo = self.buffer[:3]
                    del self.buffer[:3]
                    dist = hi*256 + lo
                    # self.log.append(f"[FRAME] Traveled: {dist} mm")
                    self.log_control.append(f"[FRAME] Traveled: {dist} mm")
                    continue

                elif hdr == 0xA1:
                    # AGV 상태 또는 배터리 잔량
                    _, b1, b2, b3 = self.buffer[:4]
                    del self.buffer[:4]
                    # 최근에 Power 버튼을 눌러 배터리 요청한 상태면 배터리로 처리
                    if self.power_call == 1:
                        # self.log.append(f"[FRAME] Battery: {b1}%")
                        self.log_power.append(f"[FRAME] Battery: {b1}%")
                    else:
                        # AGV 상태 프레임
                        if b1 == 0:
                            txt = "AGV & LiDAR & PICO OK"
                        elif b1 == 1:
                            if b2 == 1: txt = "PICO CONNECTION ERROR"
                            elif b2 == 2: txt = "LiDAR UART ERROR"
                            else:        txt = "UNKNOWN ERROR"
                        else:
                            txt = f"Unknown status({b1})"
                        # self.log.append(f"[FRAME] AGV Status: {txt}")
                        self.log_control.append(f"[FRAME] AGV Status: {txt}")

                else:
                    # 알 수 없는 헤더면 한 바이트 버리고 재시도
                    self.buffer.pop(0)
                    continue

            # 3) 남은 데이터(텍스트 ACK 등)는 기존 로직대로
            try:
                txt = data.decode(errors='ignore').strip()
                if txt and any(32 <= ord(c) <= 126 for c in txt):
                    # self.log.append(f"[RESP] {txt}")
                    self.log_control.append(f"[RESP] {txt}")
                else:
                    # self.log.append(f"[RESP] 0x{data.hex()}")
                    self.log_control.append(f"[RESP] 0x{data.hex()}")
            except:
                # self.log.append(f"[RESP] 0x{data.hex()}")
                self.log_control.append(f"[RESP] 0x{data.hex()}")

    def closeEvent(self, e):
        try: self.sock.close()
        except: pass

if __name__=="__main__":
    app = QApplication(sys.argv)
    # ─────────────────────────────────────────────────────
    # 군대 테마 + 토글 스위치 & 탭 버튼 스타일
    app.setStyleSheet("""
        /* 탭 버튼 (자동/수동) */
        QPushButton#btnAuto, QPushButton#btnMan {
            background: #CCC;
            color: #333;
            border: none;
            border-radius: 12px;
            padding: 6px 16px;
            font-weight: bold;
        }
        QPushButton#btnAuto:checked { background: #4CD964; color: white; }
        QPushButton#btnMan:checked  { background: #A0A0A0; color: white; }

        /* 토글 스위치 기본(언체크=수동): 회색 배경 + 동그라미 오른쪽 */
        QCheckBox#toggleSwitch {
        background: #A0A0A0;
        border-radius: 12px;
        min-width: 50px; min-height: 25px;
        margin: 0 4px;
        }
        /* 체크(자동) 상태: 녹색 */
        QCheckBox#toggleSwitch:checked {
            background: #4CD964;
        }
        /* Indicator: 언체크(수동)일 때 오른쪽, 체크(자동)일 때 왼쪽 */
        QCheckBox#toggleSwitch::indicator {
            subcontrol-origin: margin;
            width: 22px; height: 22px;
            border-radius: 11px;
            background: #FFF;
        }
        /* 수동(언체크)일 때 오른쪽 배치 */
        QCheckBox#toggleSwitch::indicator:unchecked {
            subcontrol-position: right center;
        }
        /* 자동(체크)일 때 왼쪽 배치 */
        QCheckBox#toggleSwitch::indicator:checked {
            subcontrol-position: left center;
        }
        """)
    w = ControlUI()
    w.show()
    w._start_reader()
    sys.exit(app.exec_())