#!/usr/bin/env python3
import sys, socket, threading, time
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGridLayout, QSizePolicy,
    QTextEdit, QLineEdit
)
from PyQt5 import QtGui
from PyQt5.QtCore import Qt

JETSON_IP        = "192.168.100.2"
JETSON_CTRL_PORT = 9000  # jetson_server.py 에 맞춤

class ControlUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AGV Debug Client")
        self.resize(500,700)

        # 상태 변수 (Byte0~Byte6)
        self.mode         = 1
        self.drive        = 0
        self.speed        = 0
        self.rotate       = 0
        self.status_call  = 0
        self.power_call   = 0
        self.reserved     = 0

        # 소켓
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((JETSON_IP, JETSON_CTRL_PORT))
        except Exception as e:
            print(f"[Error] connect: {e}")

        self._build_ui()
        self._start_reader()

    def _build_ui(self):
        main = QVBoxLayout(self)

        # Mode
        h = QHBoxLayout()
        self.btn_auto = QPushButton("자동"); self.btn_man = QPushButton("수동")
        for b in (self.btn_auto, self.btn_man):
            b.setCheckable(True)
        self.btn_auto.setChecked(True)
        h.addWidget(self.btn_auto); h.addWidget(self.btn_man)
        main.addLayout(h)

        # Drive
        h = QHBoxLayout()
        self.btn_stop = QPushButton("정지")
        self.btn_fwd  = QPushButton("전진")
        self.btn_bwd  = QPushButton("후진")
        for b in (self.btn_stop, self.btn_fwd, self.btn_bwd):
            b.setCheckable(True)
        self.btn_stop.setChecked(True)
        h.addWidget(self.btn_stop); h.addWidget(self.btn_fwd); h.addWidget(self.btn_bwd)
        main.addLayout(h)

        # Distance
        h = QHBoxLayout()
        h.addWidget(QLabel("거리 (mm):"))
        self.le_dist = QLineEdit()
        self.le_dist.setPlaceholderText("0~65535")
        self.btn_send_dist  = QPushButton("전송")
        self.btn_clear_dist = QPushButton("초기화")
        for w in (self.btn_send_dist, self.btn_clear_dist):
            w.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        h.addWidget(self.le_dist); h.addWidget(self.btn_send_dist); h.addWidget(self.btn_clear_dist)
        main.addLayout(h)

        # SpeedSel
        main.addWidget(QLabel("SpeedSel (0–9)"))
        grid = QGridLayout()
        self.speed_buttons = []
        for i in range(10):
            btn = QPushButton(str(i)); btn.setCheckable(True)
            if i==0: btn.setChecked(True)
            self.speed_buttons.append(btn)
            grid.addWidget(btn, i//5, i%5)
        main.addLayout(grid)

        # Rotate
        main.addWidget(QLabel("Rotate"))
        h = QHBoxLayout()
        labels = [("None",0),("LFT-L",1),("LHT-M",2),
                  ("LFT-H",3),("RHT-L",4),("RHT-M",5),("RHT-H",6)]
        self.rotate_buttons = []
        for text,val in labels:
            btn = QPushButton(text); btn.setCheckable(True); btn.val=val
            if val==0: btn.setChecked(True)
            self.rotate_buttons.append(btn); h.addWidget(btn)
        main.addLayout(h)

        # Status_call
        main.addWidget(QLabel("Status Call"))
        h = QHBoxLayout()
        labels = [("AGV Check",0),("LiDAR On",1),
                  ("LiDAR Off",2),("LiDAR Reset",3)]
        self.status_buttons = []
        for text,val in labels:
            btn = QPushButton(text); btn.setCheckable(True); btn.val=val
            if val==0: btn.setChecked(True)
            self.status_buttons.append(btn); h.addWidget(btn)
        main.addLayout(h)

        # SBC / ADC
        main.addWidget(QLabel("Power / ADC Call"))
        h = QHBoxLayout()
        labels = [("Null",0),("Power Off",1),("ADC Call",2)]
        self.power_buttons = []
        for text,val in labels:
            btn = QPushButton(text); btn.setCheckable(True); btn.val=val
            if val==0: btn.setChecked(True)
            self.power_buttons.append(btn); h.addWidget(btn)
        main.addLayout(h)

        # Log
        main.addWidget(QLabel("Log"))
        self.log = QTextEdit(); self.log.setReadOnly(True)
        main.addWidget(self.log)

        # Bind
        self.btn_auto.clicked.connect(lambda: self._update('mode',1))
        self.btn_man.clicked.connect(lambda: self._update('mode',0))
        self.btn_stop.clicked.connect(lambda: self._update('drive',0))
        self.btn_fwd.clicked.connect(lambda: self._update('drive',1))
        self.btn_bwd.clicked.connect(lambda: self._update('drive',2))
        for btn in self.speed_buttons:
            btn.clicked.connect(lambda _,b=btn: self._update('speed',int(b.text())))
        for btn in self.rotate_buttons:
            btn.clicked.connect(lambda _,b=btn: self._update('rotate',b.val))
        for btn in self.status_buttons:
            btn.clicked.connect(lambda _,b=btn: self._update('status_call',b.val))
        for btn in self.power_buttons:
            btn.clicked.connect(lambda _,b=btn: self._update('power_call',b.val))
        self.btn_send_dist.clicked.connect(self._send_dist)
        self.btn_clear_dist.clicked.connect(self._clear_dist)

    def _start_reader(self):
        threading.Thread(target=self._reader,daemon=True).start()

    def _reader(self):
        while True:
            try:
                data = self.sock.recv(64)    # <-- self.ctrl_sock → self.sock 로 수정
                if not data:
                    break

                # 라이다 외 agv 상태 받기
                if len(data) == 4:
                    header, status, hi, lo = data
                    if header == 0xA1:
                        # AGV 상태
                        if status == 0:
                            status_txt = "AGV & LiDAR & PICO OK"
                        elif status == 1:
                            if hi == 1:
                                status_txt = "PICO CONNECTION ERROR"
                            elif hi == 2:
                                status_txt = "MOTOR CONNECTION ERROR"
                            elif hi == 3:
                                status_txt = "LiDAR CONNECTION ERROR"
                        else:
                            status_txt = f"Unknown status ({status})"
                        msg = f"[FRAME] AGV Status: {status_txt}"

                # 라이다 데이터 받기
                if len(data) == 5:
                    header, status, hi, mid, lo = data
                    if header == 0x5A:
                        # LiDAR 데이터
                        # distance_mm = int.from_bytes(bytes([hi, mid, lo]), byteorder='big', signed=True)
                        distance_mm = int(f"{hi:02d}{mid:02d}{lo:02d}")
                        if status == 0:
                            status_txt = "LiDAR OK"
                        elif status == 1:
                            status_txt = "Signal too weak"
                        elif status == 2:
                            status_txt = "Signal too strong"
                        elif status == 3:
                            status_txt = "Out of range"
                        elif status == 4:
                            status_txt = "System error"
                        else:
                            status_txt = f"LiDAR error ({status})"
                        msg = f"[FRAME] LiDAR {status_txt}, Distance: {distance_mm} mm"

                    else:
                        msg = f"[FRAME] Unknown header: 0x{header:02X}"

                    # **여기서** 반드시 append!
                    self.log.append(msg)
                    self.log.moveCursor(QtGui.QTextCursor.End)

                else:
                    # 4, 5바이트 아닌 경우(ACK, 텍스트 응답 등)
                    txt = data.decode(errors='ignore').strip()
                    self.log.append(f"[RESP] {txt}")

            except Exception:
                break

    def _build_frame(self):
        f = bytearray(8)
        f[0]=self.mode; f[1]=self.drive; f[2]=self.speed; f[3]=self.rotate
        f[4]=self.status_call; f[5]=self.power_call; f[6]=self.reserved
        f[7]=sum(f[:7])&0xFF
        return f

    def _send_frame(self):
        pkt = self._build_frame()
        try:
            self.sock.sendall(pkt)
            self.log.append(f">>> {pkt.hex()}")
        except Exception as e:
            self.log.append(f"[Error] send: {e}")

    def keyPressEvent(self, e):
        if e.key() == Qt.Key_S:
            # 0byte는 's'의 아스키코드, 나머지 0으로
            f = bytearray(8)
            f[0] = ord('s')
            # 나머지 모두 0
            f[7] = sum(f[:7]) & 0xFF
            try:
                self.sock.sendall(f)
                self.log.append(f"[EMG STOP] >>> {f.hex()}")
            except Exception as ex:
                self.log.append(f"[Error] EMG STOP send: {ex}")
            e.accept()
        else:
            super().keyPressEvent(e)

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
        if name=='rotate':
            for b in self.rotate_buttons: b.setChecked(b.val==val)
        if name=='status_call':
            for b in self.status_buttons: b.setChecked(b.val==val)
        if name=='power_call':
            for b in self.power_buttons: b.setChecked(b.val==val)
        self._send_frame()

    def _send_dist(self):
        txt = self.le_dist.text().strip()
        if not txt.isdigit(): return
        dist_hi, dist_mid, dist_lo = mm
        self.mode = 2
        # {hi:02d}{mid:02d}{lo:02d}
        self.drive = dist_hi
        self.speed = dist_mid
        self.rotate = dist_lo
        self._send_frame()
        self.mode = 1

    def _clear_dist(self):
        self.mode=3
        self.drive=self.speed=self.rotate=self.status_call=self.power_call=self.reserved=0
        self._send_frame()
        self.mode = 1

    def closeEvent(self, e):
        try: self.sock.close()
        except: pass

if __name__=="__main__":
    app = QApplication(sys.argv)
    w = ControlUI()
    w.show()
    w._start_reader()
    sys.exit(app.exec_())
