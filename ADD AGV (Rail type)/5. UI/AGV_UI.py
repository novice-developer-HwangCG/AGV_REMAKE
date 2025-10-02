#!/usr/bin/env python3
import os
import sys, socket, threading, time
from PyQt5.QtWidgets import (
    QApplication, QWidget, QGridLayout, QFrame,
    QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QTextEdit,
    QSizePolicy, QCheckBox, qApp
)

from PyQt5.QtGui import QIcon, QPixmap, QTextCursor
from PyQt5 import QtGui
from PyQt5.QtCore import Qt

# PICO_IP        = "192.168.100.10"
# PICO_CTRL_PORT = 5024  # pico 에 맞춤

# 이 스크립트(.py) 파일이 있는 폴더 경로
base_dir = os.path.dirname(os.path.abspath(__file__))

class ControlUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AGV Controller")
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

        self.connected = False
        # 소켓
        # self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # try:
        #     self.sock.connect((PICO_IP, PICO_CTRL_PORT))
        # except Exception as e:
        #     print(f"[Error] connect: {e}")
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
        
        # ─── 여기에 서버 연결 입력창/버튼/스위치 추가 ───
        connect_h = QHBoxLayout()
        connect_h.addWidget(QLabel("서버 연결:"))

        # IP 입력창
        self.le_ip = QLineEdit()
        self.le_ip.setPlaceholderText("IP ex:192.168.100.10")
        self.le_ip.setMinimumSize(100, 40)
        self.le_ip.setFixedWidth(160)
        connect_h.addWidget(self.le_ip)

        # Port 입력창
        self.le_port = QLineEdit()
        self.le_port.setPlaceholderText("PORT ex:5024")
        self.le_port.setMinimumSize(100, 40)
        self.le_port.setFixedWidth(80)
        connect_h.addWidget(self.le_port)

        # Connect 버튼
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setFixedWidth(40)
        self.btn_connect.setMinimumSize(80, 40)
        
        self.btn_connect.clicked.connect(self._on_connect)
        connect_h.addWidget(self.btn_connect)

        connect_h.addStretch()

        self.switch = QCheckBox()
        self.switch.setObjectName("toggleSwitch")
        self.switch.setEnabled(False)    # 사용자 토글 비허용
        self.switch.setFixedSize(50, 25)
        connect_h.addWidget(self.switch)

        cl.addLayout(connect_h)

        # Mode + 스위치
        h = QHBoxLayout()
        # 버튼 생성
        self.btn_auto = QPushButton("거리조작")
        self.btn_auto.setObjectName("btnAuto")
        self.btn_man  = QPushButton("수동조작")
        self.btn_man .setObjectName("btnMan")

        for btn in (self.btn_auto, self.btn_man):
            btn.setCheckable(True)
            btn.setMinimumSize(100, 40)
            # 가로 공간을 동일하게 나눠 채우도록 Expanding
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            btn.setFixedHeight(40)
            # stretch=1 로 레이아웃의 남는 가로를 1:1 나눔
            h.addWidget(btn, 1)

        # 기본 선택은 거리조작
        self.btn_auto.setChecked(True)
        cl.addLayout(h)
        # Drive
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
        h.addWidget(self.btn_stop); h.addWidget(self.btn_fwd); h.addWidget(self.btn_bwd)
        cl.addLayout(h)

        # Distance
        h = QHBoxLayout()
        h.addWidget(QLabel("거리 (m):"))
        self.le_dist = QLineEdit()
        self.le_dist.setPlaceholderText("ex) 2.000 or 2.0000")
        self.le_dist.setMinimumSize(100, 40)

        self.btn_send_dist  = QPushButton("전송")
        self.btn_clear_dist = QPushButton("초기화")
        for w in (self.btn_send_dist, self.btn_clear_dist):
            w.setMinimumSize(100, 40)    # 너비100×높이40
            w.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        h.addWidget(self.le_dist); h.addWidget(self.btn_send_dist); h.addWidget(self.btn_clear_dist)
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
        cl.addWidget(QLabel("Status Call"))
        h = QHBoxLayout()
        labels = [("AGV Check",0), ("LiDAR On",1), ("LiDAR Off",2), ("LiDAR Reset",3)]
        self.status_buttons = []
        for text,val in labels:
            btn = QPushButton(text); btn.setCheckable(True); btn.val=val
            if val==0: 
                btn.setChecked(True)
            btn.setMinimumSize(100, 40)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            self.status_buttons.append(btn); h.addWidget(btn)
        cl.addLayout(h)

        cl.addWidget(QLabel("Power / BAT Call"))
        h2 = QHBoxLayout()
        labels = [("Null",0),("BAT Call",1)]
        self.power_buttons = []
        for text,val in labels:
            btn = QPushButton(text)
            btn.setCheckable(True)
            btn.val = val
            if val == 0: 
                btn.setChecked(True)
            btn.setMinimumSize(100, 40)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            self.power_buttons.append(btn)
            h2.addWidget(btn)
        cl.addLayout(h2)
        # 컨트롤 패널을 (0,1)에 추가
        grid.addWidget(ctrl_frame, 0, 1)

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

    def _on_connect(self):
        # 1) IP와 Port를 각 입력창에서 가져오기
        ip = self.le_ip.text().strip()
        port_str = self.le_port.text().strip()
        # 2) 간단한 입력 검증
        if not ip or not port_str.isdigit():
            self.log_control.append("IP 또는 Port 형식 오류")
            self.switch.setChecked(False)
            return
        port = int(port_str)
        # 3) 소켓 연결 시도
        # ── 토글 동작: 아직 연결 안 된 상태면 connect, 연결된 상태면 disconnect
        if not self.connected:
            # --- CONNECT 로직 ---
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((ip, port))
                self.connected = True
                self.btn_connect.setText("Disconnect")
                self.switch.setChecked(True)
                self.log_control.append(f"Connected to {ip}:{port}")
                # 리더 스레드도 한 번만 띄우기
                self._start_reader()
            except Exception as e:
                self.switch.setChecked(False)
                self.log_control.append(f"Connect failed: {e}")

        else:
            # --- DISCONNECT 로직 ---
            # mode=0xFF 로 disconnect 신호 전송
            self.mode = 0xFF
            self._send_frame()
            try:
                self.sock.close()
            except:
                pass
            self.connected = False
            self.btn_connect.setText("Connect")
            self.switch.setChecked(False)
            self.log_control.append("Disconnected")

    def _start_reader(self):
        threading.Thread(target=self._reader,daemon=True).start()

    def _reader(self):
        self.buffer = bytearray()
        while True:
            data = self.sock.recv(64)
            if not data:
                break
            self.buffer.extend(data)

            # ─── 1) 바이너리 프레임 파싱 ──────────────────
            #   LiDAR 프레임(0x5A, 5바이트), ACK(0xD2,4바이트), STATUS(0xA1,4바이트)
            while True:
                if len(self.buffer) >= 5 and self.buffer[0] == 0x5A:
                    _, status, hi, md, lo = self.buffer[:5]
                    del self.buffer[:5]
                    dist_mm = hi * 10000 + md * 100 + lo
                    dist_m  = dist_mm / 10000.0 
                    self.log_power.append(f"[FRAME] LIDAR dist: {dist_m:.3f} m (status={status})")
                    self.log_power.moveCursor(QTextCursor.End)

                elif len(self.buffer) >= 4 and self.buffer[0] == 0xD2:
                    _, hi, md, lo = self.buffer[:4]
                    del self.buffer[:4]
                    resp_dist = hi * 10000 + md * 100 + lo
                    pico_dist = resp_dist / 10000.0 
                    self.log_control.append(f"[FRAME] Target distance set: {pico_dist} mm")
                    self.log_control.moveCursor(QTextCursor.End)

                elif len(self.buffer) >= 4 and self.buffer[0] == 0xA2:
                    _, b1, b2, b3 = self.buffer[:4]
                    del self.buffer[:4]
                    if b1 = 0:
                        txt = "Motor STOP"
                    elif b1 = 1:
                        txt = "Motor RUN (FOWARD)"
                    elif b1 = 2:
                        txt = "Motor RUN (BACKWARD)"
                    else:
                        txt = "UNKNOWN ERROR"
                    self.log_control.append(f"[FRAME] Motor Status: {txt}")
                    self.log_control.moveCursor(QTextCursor.End)

                elif len(self.buffer) >= 4 and self.buffer[0] == 0xA1:
                    _, b1, b2, b3 = self.buffer[:4]
                    del self.buffer[:4]
                    # power_call 분기 등 기존 로직…
                    if self.power_call == 1:
                        self.log_control.append(f"[FRAME] Battery: {b1}%")
                        self.log_control.moveCursor(QTextCursor.End)
                    else:

                        if b1 == 0:
                            txt = "AGV & LiDAR OK"
                        elif b1 == 1:
                            if b2 == 1: txt = "PICO CONNECTION ERROR"
                            elif b2 == 2: txt = "LiDAR UART ERROR"
                            else:        txt = "UNKNOWN ERROR"
                        else:
                            txt = f"Unknown status({b1})"
                        self.log_control.append(f"[FRAME] AGV Status: {txt}")
                    self.log_control.moveCursor(QTextCursor.End)

                else:
                    # 더 이상 파싱할 완전한 프레임이 남아 있지 않으면 빠져나감
                    break

            # ─── 2) 텍스트 응답 처리 ─────────────────────
            #    (printable ASCII + '\n' 만 걸러내기)
            while b'\n' in self.buffer:
                idx  = self.buffer.find(b'\n')
                line = self.buffer[:idx+1]
                del self.buffer[:idx+1]
                text = line.decode('utf-8', errors='ignore').strip()
                if text and all(32 <= ord(c) <= 126 for c in text):
                    self.log_control.append(f"[RESP] {text}")
                    self.log_control.moveCursor(QTextCursor.End)

    def _build_frame(self):
        f = bytearray(8)
        f[0]=self.mode; f[1]=self.drive; f[2]=self.speed; f[3]=self.rsv_1
        f[4]=self.status_call; f[5]=self.power_call; f[6]=self.reserved
        f[7]=sum(f[:7])&0xFF
        return f

    def _send_frame(self):
        pkt = self._build_frame()
        try:
            self.sock.sendall(pkt)
            # if self.power_call == 1:
            #     self.log_power.append(f">>> {pkt.hex()}")
            # else:
            #     self.log_control.append(f">>> {pkt.hex()}")
        except Exception as e:
            ## self.log.append(f"[Error] send: {e}")
            self.log_control.append(f"[Error] send: {e}")

    def _update(self, name, val):
        setattr(self, name, val)
        # toggle buttons
        if name=='mode':
            self.btn_auto.setChecked(val==1); self.btn_man.setChecked(val==0)
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

    def _send_dist(self):
        txt = self.le_dist.text().strip()
        # 1) 입력을 float(미터)로 파싱
        try:
            meters = float(txt)
        except ValueError:
            return

        # 2) raw 단위로 환산 (예: 27.1144 m → 271144)
        raw = int(meters * 10000)

        # 3) 기존 매핑 그대로
        dist_hi  = (raw // 10000) % 100    # 앞 두 자리 (27)
        dist_mid = (raw //   100) % 100    # 중간 두 자리 (11)
        dist_lo  = raw % 100               # 뒤 두 자리 (44)

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

    def closeEvent(self, e):
        # 0xFF를 mode 에 담아 "disconnect" 신호 전송
        self.mode = 0xFF
        self._send_frame()
        # 그 다음에 실제로 소켓을 닫고 창 닫기
        try: self.sock.close()
        except: pass
        super().closeEvent(e)

if __name__=="__main__":
    app = QApplication(sys.argv)
    # ─────────────────────────────────────────────────────
    # 토글 스위치 & 탭 버튼 스타일
    app.setStyleSheet("""
        /* 스위치 전체 */
        QCheckBox#toggleSwitch {
            background: #A0A0A0;          /* 언체크 시 회색 */
            border-radius: 12px;
            min-width: 24px; min-height: 24px;
            padding: 2px;                 /* 내부 여백 확보 */
        }
        /* 체크(연결됨) 상태: 초록 */
        QCheckBox#toggleSwitch:checked {
            background: #4CD964;
        }

        /* 동그라미(Indicator) */
        QCheckBox#toggleSwitch::indicator {
            width: 20px; height: 20px;
            border-radius: 10px;
            background: #FFF;
            margin: 0;                    /* 움직이지 않도록 */
            subcontrol-origin: margin;
            subcontrol-position: center;  /* 항상 중앙 고정 */
        }
        /* 위치 전환 룰 모두 제거 */
        /* QCheckBox#toggleSwitch::indicator:unchecked { … } */
        /* QCheckBox#toggleSwitch::indicator:checked   { … } */
    """)

    w = ControlUI()
    w.show()
    w._start_reader()
    sys.exit(app.exec_())