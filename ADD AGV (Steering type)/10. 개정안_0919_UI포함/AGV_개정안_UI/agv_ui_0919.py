#!/usr/bin/env python3
"""
AGV Debug Client (refactored for Ubuntu 20.04 / PyQt5)
- Non-blocking connect (moved to QThread)
- Thread-safe UI updates via signals/slots
- Fixed bugs: undefined vars (md→mid), missing log append for 4B frames, wrong widget name (log_control→log),
  restoring state after distance send, double reader start, mm/meters confusion
- Clean separation: UI ↔ NetWorker (QThread with TX queue)
"""
import sys
import socket
import time
from queue import Queue, Empty

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGridLayout, QSizePolicy,
    QTextEdit, QLineEdit
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5 import QtGui

# ─── Settings ───────────────────────────────────────────────────────────────
JETSON_IP        = "192.168.100.2"
JETSON_CTRL_PORT = 9000
SOCKET_TIMEOUT_S = 3.0        # connect timeout
POLL_SLEEP_S     = 0.01       # light sleep between polls in thread loop


# ─── Networking worker (runs in its own thread) ─────────────────────────────
class NetWorker(QThread):
    log       = pyqtSignal(str)
    connected = pyqtSignal(bool)
    data      = pyqtSignal(bytes)

    def __init__(self, host: str, port: int):
        super().__init__()
        self.host = host
        self.port = port
        self.sock = None  # type: socket.socket | None
        self._stop = False
        self.txq: Queue[bytes] = Queue()

    def run(self):
        while not self._stop:
            # Ensure connected
            if self.sock is None:
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.settimeout(SOCKET_TIMEOUT_S)
                    s.connect((self.host, self.port))
                    s.settimeout(0.1)  # non-blocking-ish
                    self.sock = s
                    self.connected.emit(True)
                    self.server_log.emit(f"[NET] Connected to {self.host}:{self.port}")
                except Exception as e:
                    self.connected.emit(False)
                    self.server_log.emit(f"[NET] Connect failed: {e}")
                    time.sleep(1.0)
                    continue

            # Send pending
            try:
                pkt = self.txq.get_nowait()
                try:
                    self.sock.sendall(pkt)
                    self.server_log.emit(f">>> {pkt.hex()}")
                except Exception as e:
                    self.server_log.emit(f"[NET] Send error: {e}")
                    self._teardown()
                    continue
            except Empty:
                pass

            # Receive
            try:
                data = self.sock.recv(64)
                if data:
                    self.data.emit(data)
                else:
                    # orderly close
                    self.server_log.emit("[NET] Server closed connection")
                    self._teardown()
                    continue
            except socket.timeout:
                pass
            except Exception as e:
                self.server_log.emit(f"[NET] Recv error: {e}")
                self._teardown()
                continue

            time.sleep(POLL_SLEEP_S)

        # On stop request
        self._teardown()

    def _teardown(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        self.connected.emit(False)

    def send(self, pkt: bytes):
        self.txq.put(pkt)

    def stop(self):
        self._stop = True


# ─── Main UI ────────────────────────────────────────────────────────────────
class ControlUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AGV Debug Client")
        self.resize(560, 760)

        # State bytes (Byte0~Byte6)
        self.mode        = 1
        self.drive       = 0
        self.speed       = 0
        self.rotate      = 0
        self.status_call = 0
        self.power_call  = 0
        self.reserved    = 0

        # Network worker
        self.net = NetWorker(JETSON_IP, JETSON_CTRL_PORT)
        self.net.server_log.connect(self._append_server_log)
        self.net.data.connect(self._on_data)
        self.net.connected.connect(self._on_connected)

        self._build_ui()
        self.net.start()  # non-blocking connect in background

    # ── UI Build ────────────────────────────────────────────────────────────
    def _build_ui(self):
        main = QVBoxLayout(self)

        # Connection status
        status_row = QHBoxLayout()
        status_row.addWidget(QLabel("Connection:"))
        self.lbl_status = QLabel("…")
        self.lbl_status.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        status_row.addWidget(self.lbl_status)
        status_row.addStretch(1)
        main.addLayout(status_row)

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

        # Distance (accepts mm or meters with decimal)
        h = QHBoxLayout()
        h.addWidget(QLabel("거리 입력:"))
        self.le_dist = QLineEdit()
        self.le_dist.setPlaceholderText("정수=mm/소수=M 예)2500 or 2.5")
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
            if i == 0: btn.setChecked(True)
            self.speed_buttons.append(btn)
            grid.addWidget(btn, i // 5, i % 5)
        main.addLayout(grid)

        # Rotate
        main.addWidget(QLabel("Rotate"))
        h = QHBoxLayout()
        labels = [("None",0),("LFT-L",1),("LHT-M",2),
                  ("LFT-H",3),("RHT-L",4),("RHT-M",5),("RHT-H",6)]
        self.rotate_buttons = []
        for text,val in labels:
            btn = QPushButton(text); btn.setCheckable(True); btn.val = val
            if val == 0: btn.setChecked(True)
            self.rotate_buttons.append(btn); h.addWidget(btn)
        main.addLayout(h)

        # Status_call
        main.addWidget(QLabel("Status Call"))
        h = QHBoxLayout()
        labels = [("AGV Check",0),("LiDAR On",1),("LiDAR Off",2),("LiDAR Reset",3)]
        self.status_buttons = []
        for text,val in labels:
            btn = QPushButton(text); btn.setCheckable(True); btn.val = val
            if val == 0: btn.setChecked(True)
            self.status_buttons.append(btn); h.addWidget(btn)
        main.addLayout(h)

        # Power
        main.addWidget(QLabel("Power"))
        h = QHBoxLayout()
        labels = [("Null",0),("Power Off",1)]
        self.power_buttons = []
        for text,val in labels:
            btn = QPushButton(text); btn.setCheckable(True); btn.val = val
            if val == 0: btn.setChecked(True)
            self.power_buttons.append(btn); h.addWidget(btn)
        main.addLayout(h)

        # Motor status
        main.addWidget(QLabel("Motor Status"))
        h = QHBoxLayout()
        labels = [("FWD",0),("STOP",1),("BWD",2)]
        # 버튼이 아닌 단순 라벨로 표시
        main.addLayout(h)

        # Battery status
        """
        배터리 표시 라벨 추가 필요, 이제 adc값을 요청해서 배터리 값을 받는 것이 아닌 지속적으로 받음, 라벨 색상은 단순히 초록색으로만 만약 [0xA1, 2, 0, 5]를 받았을 경우 5칸만 색칠
        받는 데이터 값은 총 4byte 예시) [0xA1, 2, 0, 5], 마지막 3byte 자리 값은 0 ~ 9까지, 0=none, 1=20%이하, 2=20%이상, ... , 9=90%이상
        """

        # Server Log
        main.addWidget(QLabel("Server Log"))
        self.server_log = QTextEdit(); self.server_log.setReadOnly(True)
        main.addWidget(self.server_log)
    
        # Lidar Log
        main.addWidget(QLabel("Lidar Log"))
        self.lidar_log = QTextEdit(); self.lidar_log.setReadOnly(True)
        main.addWidget(self.lidar_log)

        # Bind actions
        self.btn_auto.clicked.connect(lambda: self._update('mode', 1))
        self.btn_man.clicked.connect(lambda: self._update('mode', 0))
        self.btn_stop.clicked.connect(lambda: self._update('drive', 0))
        self.btn_fwd.clicked.connect(lambda: self._update('drive', 1))
        self.btn_bwd.clicked.connect(lambda: self._update('drive', 2))
        for btn in self.speed_buttons:
            btn.clicked.connect(lambda _, b=btn: self._update('speed', int(b.text())))
        for btn in self.rotate_buttons:
            btn.clicked.connect(lambda _, b=btn: self._update('rotate', b.val))
        for btn in self.status_buttons:
            btn.clicked.connect(lambda _, b=btn: self._update('status_call', b.val))
        for btn in self.power_buttons:
            btn.clicked.connect(lambda _, b=btn: self._update('power_call', b.val))
        self.btn_send_dist.clicked.connect(self._send_dist)
        self.btn_clear_dist.clicked.connect(self._clear_dist)

    # ── Slots ───────────────────────────────────────────────────────────────
    def _append_server_log(self, msg: str):
        self.server_log.append(msg)
        self.server_log.moveCursor(QtGui.QTextCursor.End)

    def _append_lidar_log(self, msg: str):
        self.server_log.append(msg)
        self.server_log.moveCursor(QtGui.QTextCursor.End)

    def _on_connected(self, ok: bool):
        self.lbl_status.setText("Connected" if ok else "Disconnected")
        self.lbl_status.setStyleSheet("color: %s" % ("#00d084" if ok else "#f25c54"))

    def _on_data(self, data: bytes):
        # Parse frames: 4-byte AGV status or 5-byte LiDAR, else text
        try:
            if len(data) == 4:
                header, status, hi, lo = data
                if header == 0xA1:
                    if status == 0:
                        status_txt = "AGV & LiDAR & PICO OK"
                    elif status == 1:
                        if   hi == 1: status_txt = "PICO CONNECTION ERROR"
                        elif hi == 2: status_txt = "MOTOR CONNECTION ERROR"
                        elif hi == 3: status_txt = "LiDAR CONNECTION ERROR"
                        elif hi == 4: status_txt = "NONE LINES"
                        else:         status_txt = f"Unknown substatus ({hi})"
                    else:
                        status_txt = f"Unknown status ({status})"
                    msg = f"[FRAME] AGV Status: {status_txt}"
                    self._append_server_log(msg)
                elif header == 0xA2:
                    # 모터 상태 값은 받아서 텍스트 출력이 아닌 라벨 표시로 점등 시키기 FWD, STOP, BWD
                    if   status == 0: status_txt = "Motor State Stop"
                    elif status == 1: status_txt = "Motor State Run (Forward)"
                    elif status == 2: status_txt = "Motor State Run (Backward)"
                    else:             status_txt = f"Unknown motor state ({status})"
                    self._append_server_log(f"[FRAME] {status_txt}")
                else:
                    self._append_server_log(f"[FRAME] Unknown 4B header: 0x{header:02X}")

            elif len(data) == 5:
                header, status, hi, mid, lo = data
                if header == 0x5A:
                    distance_mm = hi * 10000 + mid * 100 + lo  # {hi:02d}{mid:02d}{lo:02d}
                    distance_m = raw / 10000.0
                    if   status == 0: status_txt = "LiDAR OK"
                    elif status == 2: status_txt = "System error"
                    else:             status_txt = f"LiDAR {status})"
                    self._append_lidar_log(f"[FRAME] {status_txt}, Distance: {distance_m:.4f} mm")
                else:
                    self._append_lidar_log(f"[FRAME] Unknown 5B header: 0x{header:02X}")

            else:
                # Fallback: try decode text
                txt = data.decode(errors='ignore').strip()
                if txt:
                    self._append_server_log(f"[RESP] {txt}")
                else:
                    self._append_server_log(f"[RAW] {data.hex()}")
        except Exception as e:
            self._append_server_log(f"[ParseError] {e} (raw={data.hex()})")

    # ── Frame helpers ───────────────────────────────────────────────────────
    def _build_frame(self) -> bytes:
        f = bytearray(8)
        f[0] = self.mode
        f[1] = self.drive
        f[2] = self.speed
        f[3] = self.rotate
        f[4] = self.status_call
        f[5] = self.power_call
        f[6] = self.reserved
        f[7] = sum(f[:7]) & 0xFF
        return bytes(f)

    def _send_frame(self):
        self.net.send(self._build_frame())

    # ── UI Actions ─────────────────────────────────────────────────────────
    def _update(self, name: str, val: int):
        setattr(self, name, val)
        # toggle buttons (visual sync)
        if name == 'mode':
            self.btn_auto.setChecked(val == 1)
            self.btn_man.setChecked(val == 0)
        if name == 'drive':
            self.btn_stop.setChecked(val == 0)
            self.btn_fwd.setChecked(val == 1)
            self.btn_bwd.setChecked(val == 2)
        if name == 'speed':
            for b in self.speed_buttons: b.setChecked(int(b.text()) == val)
        if name == 'rotate':
            for b in self.rotate_buttons: b.setChecked(b.val == val)
        if name == 'status_call':
            for b in self.status_buttons: b.setChecked(b.val == val)
        if name == 'power_call':
            for b in self.power_buttons: b.setChecked(b.val == val)
        self._send_frame()

    def _send_dist(self):
        txt = self.le_dist.text().strip()
        if not txt:
            return
        try:
            # Allow either mm (int) or meters (float like 2.71)
            if '.' in txt:
                meters = float(txt)
                distance_mm = int(round(meters * 1000))
            else:
                distance_mm = int(txt)
        except ValueError:
            self._append_server_log("숫자를 입력하세요 (정수=mm, 소수점=미터)")
            return

        if not (0 <= distance_mm <= 999_999):
            self._append_server_log("범위 초과: 0 ~ 999,999")
            return

        # Map to three BCD-like fields {hi:02d}{mid:02d}{lo:02d}
        dist_hi  = (distance_mm // 10000) % 100
        dist_mid = (distance_mm //   100) % 100
        dist_lo  =  distance_mm % 100

        old_state = (self.mode, self.drive, self.speed, self.rotate)

        # Mode 2: distance payload (encoded in drive/speed/rotate)
        self.mode   = 2
        self.drive  = dist_hi
        self.speed  = dist_mid
        self.rotate = dist_lo
        self._send_frame()
        self._append_lidar_log(f"[DIST] {distance_mm} m") #  hi={dist_hi:02d}, mid={dist_mid:02d}, lo={dist_lo:02d}

        # Restore & resync buttons visibly
        self.mode, self.drive, self.speed, self.rotate = old_state
        self._sync_buttons()
        self._send_frame()

    def _sync_buttons(self):
        self.btn_auto.setChecked(self.mode == 1)
        self.btn_man.setChecked(self.mode == 0)
        self.btn_stop.setChecked(self.drive == 0)
        self.btn_fwd.setChecked(self.drive == 1)
        self.btn_bwd.setChecked(self.drive == 2)
        for b in self.speed_buttons: b.setChecked(int(b.text()) == self.speed)
        for b in self.rotate_buttons: b.setChecked(b.val == self.rotate)
        for b in self.status_buttons: b.setChecked(b.val == self.status_call)
        for b in self.power_buttons: b.setChecked(b.val == self.power_call)

    def _clear_dist(self):
        # Mode 3: clear (protocol-specific)
        self.mode = 3
        self.drive = self.speed = self.rotate = self.status_call = self.power_call = self.reserved = 0
        self._send_frame()
        self.mode = 1
        self._sync_buttons()
        self._append_server_log("[DIST] clear sent")

    # ── Qt events ──────────────────────────────────────────────────────────
    def closeEvent(self, e):
        try:
            # Send shutdown frame (mode=0xFF)
            self.mode = 0xFF
            self._send_frame()
        except Exception:
            pass
        try:
            self.net.stop()
            self.net.wait(500)
        except Exception:
            pass
        e.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = ControlUI()
    w.show()
    sys.exit(app.exec_())
