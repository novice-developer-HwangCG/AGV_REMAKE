#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py
- new_agv_ui_form.ui 를 로드해서 사용하는 AGV GUI 메인 코드 골격
- 기존 agv_ui_kinetix.py 에 들어있던 통신/프로토콜 기능을 UI 밖으로 분리한 형태

실행 위치 예시:
    main.py
    new_agv_ui_form.ui
    logo_ADD.png
세 파일을 같은 폴더에 둔 뒤 실행:
    python3 main.py
"""

import os
import sys
import socket
import threading
from typing import List, Dict, Any, Optional

from PyQt5 import uic
from PyQt5.QtCore import QObject, Qt, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QLayout, QSizePolicy
from PyQt5.QtGui import QIcon

def resource_path(relative_path: str) -> str:
    """
    python main.py 실행 시와 PyInstaller exe 실행 시 모두 리소스 파일을 찾기 위한 함수
    """
    if hasattr(sys, "_MEIPASS"):
        return os.path.join(sys._MEIPASS, relative_path)
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), relative_path)

# =========================
# Protocol constants
# =========================
FRAME_HEADER_LIDAR = 0x5A
FRAME_HEADER_DIST = 0xD2
FRAME_HEADER_STATUS = 0xA1
FRAME_HEADER_MOTOR = 0xA2
FRAME_DISCONNECT = 0xFF

MOTOR_STOP = 0
MOTOR_FWD = 1
MOTOR_BWD = 2

KNOWN_RX_HEADERS = (FRAME_HEADER_LIDAR, FRAME_HEADER_DIST, FRAME_HEADER_STATUS, FRAME_HEADER_MOTOR)


class AGVProtocol:
    """
    UI -> Pico 송신 프레임 상태를 관리

    송신 프레임 8 byte:
        [mode, drive, speed, rsv_1, status_call, power_call, reserved, checksum]

    mode:
        0 = Manual Mode
        1 = Distance/Auto Mode
        2 = Distance value send
        3 = Distance reset
        0xFF = disconnect

    drive:
        0 = STOP
        1 = FWD
        2 = BWD
        3 = Zero Position Move

    status_call:
        0 = AGV Check
        1 = LiDAR On
        2 = LiDAR Off
        3 = Zero Set / LiDAR Reset / baseline set
    """

    def __init__(self):
        self.mode = 1
        self.drive = 0
        self.speed = 0
        self.rsv_1 = 0
        self.status_call = 0
        self.power_call = 0
        self.reserved = 0

    def build_frame(self) -> bytes:
        frame = bytearray(8)
        frame[0] = self.mode & 0xFF
        frame[1] = self.drive & 0xFF
        frame[2] = self.speed & 0xFF
        frame[3] = self.rsv_1 & 0xFF
        frame[4] = self.status_call & 0xFF
        frame[5] = self.power_call & 0xFF
        frame[6] = self.reserved & 0xFF
        frame[7] = sum(frame[:7]) & 0xFF
        return bytes(frame)

    def build_disconnect_frame(self) -> bytes:
        old_mode = self.mode
        self.mode = FRAME_DISCONNECT
        pkt = self.build_frame()
        self.mode = old_mode
        return pkt

    def build_distance_frame(self, meters: float) -> bytes:
        """
        거리 입력값을 기존 방식대로 2자리 + 2자리로 포장
        예) 2.00m  -> raw=0200 -> drive=2,  speed=0
            27.44m -> raw=2744 -> drive=27, speed=44
        """
        raw = int(meters * 100)
        if raw < 0:
            raise ValueError("거리는 음수가 될 수 없습니다")
        if raw >= 9999:
            raise ValueError("최대 거리 99.99m 초과")

        dist_hi = (raw // 100) % 100
        dist_lo = raw % 100

        old_drive = self.drive
        old_speed = self.speed

        self.mode = 2
        self.drive = dist_hi
        self.speed = dist_lo
        pkt = self.build_frame()

        # 기존 UI 로직과 동일하게 거리 전송 후 기본 거리모드로 복귀
        self.mode = 1
        self.drive = old_drive
        self.speed = old_speed
        return pkt

    def build_distance_reset_frame(self) -> bytes:
        self.mode = 3
        self.drive = 0
        self.speed = 0
        self.rsv_1 = 0
        self.status_call = 0
        self.power_call = 0
        self.reserved = 0
        pkt = self.build_frame()
        self.mode = 1
        return pkt


class AGVRxParser:
    """
    Pico -> UI 수신 버퍼 파서.

    기존 PC UI 쪽은 LiDAR 9 byte를 파싱하고 있었고,
    Pico 코드 쪽은 경우에 따라 6 byte LiDAR 프레임을 보낼 수 있으므로
    골격 단계에서는 두 형식을 모두 받을 수 있게 만들어 둠

    9 byte LiDAR:
        [0x5A, status, abs_hi, abs_mid, abs_lo, rel_hi, rel_mid, rel_lo, sign]

    6 byte LiDAR:
        [0x5A, status, hi, mid, lo, sign]
    """

    def __init__(self):
        self.buffer = bytearray()

    @staticmethod
    def _decode_6digit_to_m(hi: int, mid: int, lo: int, sign: int = 0) -> float:
        raw = hi * 10000 + mid * 100 + lo
        if sign:
            raw = -raw
        return raw / 10000.0

    def feed(self, data: bytes) -> List[Dict[str, Any]]:
        events: List[Dict[str, Any]] = []
        if data:
            self.buffer.extend(data)

        while self.buffer:
            head = self.buffer[0]

            # -------------------------
            # 1) LiDAR binary frame
            # -------------------------
            if head == FRAME_HEADER_LIDAR:
                # 6 byte 프레임 바로 뒤에 다른 known header가 오면 6 byte로 판단
                if (
                    len(self.buffer) >= 6
                    and self.buffer[5] in (0, 1)
                    and (len(self.buffer) == 6 or (len(self.buffer) > 6 and self.buffer[6] in KNOWN_RX_HEADERS))
                ):
                    _, status, hi, mid, lo, sign = self.buffer[:6]
                    del self.buffer[:6]
                    value_m = self._decode_6digit_to_m(hi, mid, lo, sign)
                    events.append({"type": "lidar_single", "status": status, "value_m": value_m})
                    continue

                # 9 byte 프레임: 절대/상대거리 동시 수신
                if len(self.buffer) >= 9 and self.buffer[8] in (0, 1):
                    _, status, a_hi, a_mid, a_lo, r_hi, r_mid, r_lo, sign = self.buffer[:9]
                    del self.buffer[:9]
                    abs_m = self._decode_6digit_to_m(a_hi, a_mid, a_lo, 0)
                    rel_m = self._decode_6digit_to_m(r_hi, r_mid, r_lo, sign)
                    events.append({"type": "lidar_pair", "status": status, "abs_m": abs_m, "rel_m": rel_m})
                    continue

                # 아직 프레임이 덜 들어온 상태
                break

            # -------------------------
            # 2) Distance ACK frame
            #    [0xD2, hi, lo]
            # -------------------------
            if head == FRAME_HEADER_DIST:
                if len(self.buffer) < 3:
                    break
                _, hi, lo = self.buffer[:3]
                del self.buffer[:3]
                target_m = (hi * 100 + lo) / 100.0
                events.append({"type": "distance_ack", "target_m": target_m})
                continue

            # -------------------------
            # 3) Motor state frame
            #    [0xA2, state, 0, 0]
            # -------------------------
            if head == FRAME_HEADER_MOTOR:
                if len(self.buffer) < 4:
                    break
                _, state, _, _ = self.buffer[:4]
                del self.buffer[:4]
                events.append({"type": "motor_state", "state": state})
                continue

            # -------------------------
            # 4) Status/Battery frame
            #    [0xA1, b1, b2, b3]
            # -------------------------
            if head == FRAME_HEADER_STATUS:
                if len(self.buffer) < 4:
                    break
                _, b1, b2, b3 = self.buffer[:4]
                del self.buffer[:4]
                if b1 == 0x02:
                    events.append({"type": "battery", "bin": b3})
                else:
                    events.append({"type": "status", "b1": b1, "b2": b2, "b3": b3})
                continue

            # -------------------------
            # 5) Text response
            #    예: b"Dist_reset\n"
            # -------------------------
            newline_idx = self.buffer.find(b"\n")
            if newline_idx >= 0:
                candidate = self.buffer[: newline_idx + 1]
                try:
                    decoded = candidate.decode("ascii")
                except UnicodeDecodeError:
                    # 깨진 데이터면 1 byte 버리고 재동기화
                    del self.buffer[0]
                    continue

                if all((32 <= ord(ch) <= 126) or ch in "\r\n\t" for ch in decoded):
                    del self.buffer[: newline_idx + 1]
                    text = decoded.strip()
                    if text:
                        events.append({"type": "text", "text": text})
                    continue

            # Unknown byte: 다음 known header까지 버리면서 재동기화
            next_positions = [self.buffer.find(bytes([h]), 1) for h in KNOWN_RX_HEADERS]
            next_positions = [p for p in next_positions if p >= 0]
            if next_positions:
                del self.buffer[: min(next_positions)]
            else:
                if len(self.buffer) > 128:
                    self.buffer.clear()
                break

        return events


class AGVClient(QObject):
    """
    실제 TCP 통신 담당.
    UI 클래스는 이 객체의 메서드를 호출하고, 결과는 signal로 받음
    """

    connectedChanged = pyqtSignal(bool, str)
    logLine = pyqtSignal(str)
    lidarAbsLine = pyqtSignal(str)
    lidarRelLine = pyqtSignal(str)
    motorStateChanged = pyqtSignal(int)
    batteryChanged = pyqtSignal(int)

    def __init__(self, parent: Optional[QObject] = None):
        super().__init__(parent)
        self.protocol = AGVProtocol()
        self.parser = AGVRxParser()
        self.sock: Optional[socket.socket] = None
        self.connected = False
        self._reader_alive = False
        self._reader_thread: Optional[threading.Thread] = None
        self._send_lock = threading.Lock()
        self._last_motor_state: Optional[int] = None
        self.lidar_value_mode = "absolute"  # 6 byte LiDAR 수신 시 ABS/REL 로그 분배용
        self._last_lidar_status: Optional[int] = None

    def connect_to_server(self, ip: str, port: int) -> None:
        if self.connected:
            return
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3.0)
            sock.connect((ip, port))
            sock.settimeout(None)
            self.sock = sock
            self.connected = True
            self._reader_alive = True
            self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._reader_thread.start()
            self.connectedChanged.emit(True, f"Connected to {ip}:{port}")
        except Exception as e:
            self.connected = False
            self.sock = None
            self.connectedChanged.emit(False, f"Connect failed: {e}")

    def disconnect_from_server(self) -> None:
        if not self.connected:
            return
        try:
            self.send_raw(self.protocol.build_disconnect_frame())
        except Exception:
            pass
        self._reader_alive = False
        try:
            if self.sock:
                self.sock.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass
        self.sock = None
        self.connected = False
        self.connectedChanged.emit(False, "Disconnected")

    def send_raw(self, pkt: bytes) -> None:
        if not self.connected or not self.sock:
            self.logLine.emit("[WARN] Not connected")
            return
        with self._send_lock:
            self.sock.sendall(pkt)

    def send_update(self, name: str, value: int) -> None:
        setattr(self.protocol, name, value)

        if name == "status_call":
            if value == 1:
                self.lidar_value_mode = "absolute"
            elif value == 3:
                self.lidar_value_mode = "relative"

        self.send_raw(self.protocol.build_frame())

    def send_distance(self, meters: float) -> None:
        pkt = self.protocol.build_distance_frame(meters)
        self.send_raw(pkt)

    def reset_distance(self) -> None:
        pkt = self.protocol.build_distance_reset_frame()
        self.send_raw(pkt)

    def _reader_loop(self) -> None:
        try:
            while self._reader_alive and self.sock:
                data = self.sock.recv(64)
                if not data:
                    break

                for event in self.parser.feed(data):
                    self._handle_event(event)

        except OSError:
            pass
        except Exception as e:
            self.logLine.emit(f"[Reader] stopped: {e}")
        finally:
            self._reader_alive = False
            if self.connected:
                self.connected = False
                self.connectedChanged.emit(False, "Connection closed")

    def _handle_event(self, event: Dict[str, Any]) -> None:
        etype = event.get("type")

        if etype == "lidar_pair":
            status = int(event["status"])

            # 거리 로그박스에는 거리값만 출력
            self.lidarAbsLine.emit(f"{event['abs_m']:.2f} m")
            self.lidarRelLine.emit(f"{event['rel_m']:.2f} m")

            # status는 0이 아닐 때만 control logbox에 출력
            if status != 0 and status != self._last_lidar_status:
                self._last_lidar_status = status
                self.logLine.emit(f"[FRAME] LiDAR Status: {status}")

            if status == 0:
                self._last_lidar_status = 0

        elif etype == "lidar_single":
            status = int(event["status"])
            value_m = event["value_m"]

            # 거리 로그박스에는 거리값만 출력
            if self.lidar_value_mode == "relative":
                self.lidarRelLine.emit(f"{value_m:.2f} m")
            else:
                self.lidarAbsLine.emit(f"{value_m:.2f} m")

            # status는 0이 아닐 때만 control logbox에 출력
            if status != 0 and status != self._last_lidar_status:
                self._last_lidar_status = status
                self.logLine.emit(f"[FRAME] LiDAR Status: {status}")

            if status == 0:
                self._last_lidar_status = 0
                
        # if etype == "lidar_pair":
        #     status = event["status"]
        #     self.lidarAbsLine.emit(f"{event['abs_m']:.2f} m")
        #     self.lidarRelLine.emit(f"{event['rel_m']:.2f} m")

        # elif etype == "lidar_single":
        #     status = event["status"]
        #     value_m = event["value_m"]
        #     if self.lidar_value_mode == "relative":
        #         self.lidarRelLine.emit(f"{value_m:.2f} m")
        #     else:
        #         self.lidarAbsLine.emit(f"{value_m:.2f} m")

        elif etype == "distance_ack":
            self.logLine.emit(f"Target distance set: {event['target_m']:.2f} m")

        elif etype == "motor_state":
            state = int(event["state"])
            if state != self._last_motor_state:
                self._last_motor_state = state
                self.motorStateChanged.emit(state)

        elif etype == "battery":
            self.batteryChanged.emit(int(event["bin"]))

        elif etype == "status":
            b1 = event["b1"]
            b2 = event["b2"]
            if b1 == 0:
                text = "AGV & LiDAR FINE"
            elif b1 == 1:
                if b2 == 1:
                    text = "PICO CONNECTION ERROR"
                elif b2 == 2:
                    text = "LiDAR UART ERROR"
                else:
                    text = "UNKNOWN ERROR"
            else:
                text = f"Unknown status({b1})"
            self.logLine.emit(f"AGV Status: {text}")

        elif etype == "text":
            self.logLine.emit(f"{event['text']}")


class MainWindow(QMainWindow):
    _LABEL_ACTIVE = """
        QLabel {
            background: #ff3b30;
            color: #ffffff;
            font-weight: 700;
            border-radius: 8px;
            padding: 6px 0;
        }
    """
    _LABEL_INACTIVE = """
        QLabel {
            background: #ffffff;
            color: #000000;
            border: 1px solid #444;
            border-radius: 8px;
            padding: 6px 0;
        }
    """

    _CONN_ON = """
        QLabel {
            background: #34c759;
            color: white;
            font-weight: 700;
            border-radius: 8px;
            padding: 6px;
        }
    """
    _CONN_OFF = """
        QLabel {
            background: #a0a0a0;
            color: white;
            font-weight: 700;
            border-radius: 8px;
            padding: 6px;
        }
    """

    def __init__(self):
        super().__init__()
        ui_path = resource_path("new_agv_ui_form.ui")
        uic.loadUi(ui_path, self)
        self.setWindowIcon(QIcon(resource_path("ADD_icon.png")))

        self.client = AGVClient(self)
        self.speed_buttons: List[QPushButton] = []
        self.mode_buttons: List[QPushButton] = []
        self.drive_buttons: List[QPushButton] = []
        self.status_buttons: List[QPushButton] = []

        self._init_ui_state()
        self._connect_signals()

    # ---------- UI helper ----------
    def _read_text(self, widget) -> str:
        if hasattr(widget, "toPlainText"):
            return widget.toPlainText().strip()
        if hasattr(widget, "text"):
            return widget.text().strip()
        return ""

    def _set_text(self, widget, text: str) -> None:
        if hasattr(widget, "setPlainText"):
            widget.setPlainText(text)
        elif hasattr(widget, "setText"):
            widget.setText(text)

    def _append_plain(self, widget, text: str) -> None:
        if hasattr(widget, "appendPlainText"):
            widget.appendPlainText(text)
        elif hasattr(widget, "append"):
            widget.append(text)

    def _init_ui_state(self) -> None:
        self.setWindowTitle("AGV Controller")
        self._apply_ui_size_fixes()

        # QTextEdit로 만들어진 IP/PORT 입력창 초기값 보정
        self._set_text(self.ip_textedit, self._read_text(self.ip_textedit) or "192.168.100.2")
        self._set_text(self.port_textedit, self._read_text(self.port_textedit) or "5024")

        # 로그창 초기화
        self.abs_lidar_logbox.clear()
        self.rel_lidar_logbox.clear()
        self.control_logbox.clear()
        # self.abs_lidar_logbox.setPlaceholderText("Absolute Distance (m)")
        # self.rel_lidar_logbox.setPlaceholderText("Relative Distance (m)")
        self.abs_lidar_logbox.setPlainText("Absolute Distance (m)")
        self.rel_lidar_logbox.setPlainText("Relative Distance (m)")

        for logbox in (self.abs_lidar_logbox, self.rel_lidar_logbox, self.control_logbox):
            logbox.setReadOnly(True)
            logbox.setFocusPolicy(Qt.NoFocus)

        self.agv_bat_bar.setRange(0, 100)
        self.agv_bat_bar.setValue(0)
        self.agv_bat_bar.setTextVisible(False)

        # 버튼 checkable 설정
        self.mode_buttons = [self.dist_mode_btn, self.manual_mode_btn]
        self.drive_buttons = [self.stop_btn, self.forward_btn, self.backward_btn, self.zero_move_btn]
        self.status_buttons = [self.agv_state_btn, self.lidar_on_btn, self.lidar_off_btn, self.lidar_set_btn]
        self.speed_buttons = [
            self.spd_0_btn, self.spd_1_btn, self.spd_2_btn, self.spd_3_btn, self.spd_4_btn,
            self.spd_5_btn, self.spd_6_btn, self.spd_7_btn, self.spd_8_btn, self.spd_9_btn,
        ]

        for btn in self.mode_buttons + self.drive_buttons + self.status_buttons + self.speed_buttons:
            btn.setCheckable(True)

        self.dist_mode_btn.setChecked(True)
        self.stop_btn.setChecked(True)
        self.agv_state_btn.setChecked(True)
        self.spd_0_btn.setChecked(True)

        self._set_connected_ui(False, "connection check")
        self._set_drive_indicator(MOTOR_STOP)
        self._set_controls_enabled(False)

    def _apply_ui_size_fixes(self) -> None:
        self.setFixedSize(960, 960)

        if hasattr(self, "mainLayout"):
            self.mainLayout.setSizeConstraint(QLayout.SetDefaultConstraint)

        if hasattr(self, "centralwidget"):
            self.centralwidget.setFixedSize(960, 960)

        if hasattr(self, "gridLayoutWidget"):
            self.gridLayoutWidget.setGeometry(8, 8, 940, 940)
            self.gridLayoutWidget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        input_specs = (
            ("ip_textedit", 150, 32),
            ("port_textedit", 70, 32),
            ("dist_textedit", 110, 32),
        )

        for object_name, width, height in input_specs:
            widget = getattr(self, object_name, None)
            if widget is None:
                continue

            widget.setMinimumSize(0, 0)
            widget.setMaximumSize(width, height)
            widget.setFixedSize(width, height)
            widget.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

            if hasattr(widget, "setAcceptRichText"):
                widget.setAcceptRichText(False)
            if hasattr(widget, "setVerticalScrollBarPolicy"):
                widget.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            if hasattr(widget, "setHorizontalScrollBarPolicy"):
                widget.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            if hasattr(widget, "setTabChangesFocus"):
                widget.setTabChangesFocus(True)

        if hasattr(self, "agv_bat_bar"):
            self.agv_bat_bar.setRange(0, 100)
            self.agv_bat_bar.setValue(0)
            self.agv_bat_bar.setTextVisible(False)

    def _connect_signals(self) -> None:
        # Client -> UI
        self.client.connectedChanged.connect(self._on_connected_changed, type=Qt.QueuedConnection)
        self.client.logLine.connect(self._append_control_log, type=Qt.QueuedConnection)
        self.client.lidarAbsLine.connect(self._append_abs_lidar_log, type=Qt.QueuedConnection)
        self.client.lidarRelLine.connect(self._append_rel_lidar_log, type=Qt.QueuedConnection)
        self.client.motorStateChanged.connect(self._set_drive_indicator, type=Qt.QueuedConnection)
        self.client.batteryChanged.connect(self._set_battery, type=Qt.QueuedConnection)

        # UI -> Client
        self.connect_btn.clicked.connect(self._on_connect_clicked)

        self.dist_mode_btn.clicked.connect(lambda: self._send_mode(1))
        self.manual_mode_btn.clicked.connect(lambda: self._send_mode(0))

        self.stop_btn.clicked.connect(lambda: self._send_drive(0))
        self.forward_btn.clicked.connect(lambda: self._send_drive(1))
        self.backward_btn.clicked.connect(lambda: self._send_drive(2))
        self.zero_move_btn.clicked.connect(lambda: self._send_drive(3))

        for idx, btn in enumerate(self.speed_buttons):
            btn.clicked.connect(lambda _, v=idx: self._send_speed(v))

        self.agv_state_btn.clicked.connect(lambda: self._send_status_call(0))
        self.lidar_on_btn.clicked.connect(lambda: self._send_status_call(1))
        self.lidar_off_btn.clicked.connect(lambda: self._send_status_call(2))
        self.lidar_set_btn.clicked.connect(lambda: self._send_status_call(3))

        self.dist_send_btn.clicked.connect(self._on_distance_send_clicked)
        self.dist_rst_btn.clicked.connect(self._on_distance_reset_clicked)

    def _set_controls_enabled(self, enabled: bool) -> None:
        widgets = (
            self.mode_buttons
            + self.drive_buttons
            + self.status_buttons
            + self.speed_buttons
            + [self.dist_textedit, self.dist_send_btn, self.dist_rst_btn]
        )
        for widget in widgets:
            widget.setEnabled(enabled)

    def _set_connected_ui(self, connected: bool, message: str) -> None:
        self.connect_btn.setText("Disconnect" if connected else "Connect")
        self.connection_check_label.setText("CONNECTED" if connected else "DISCONNECTED")
        self.connection_check_label.setStyleSheet(self._CONN_ON if connected else self._CONN_OFF)
        self._append_control_log(message)

    def _sync_checked(self, buttons: List[QPushButton], active_btn: QPushButton) -> None:
        for btn in buttons:
            btn.setChecked(btn is active_btn)

    # ---------- slots ----------
    @pyqtSlot(bool, str)
    def _on_connected_changed(self, connected: bool, message: str) -> None:
        self._set_connected_ui(connected, message)
        self._set_controls_enabled(connected)

    @pyqtSlot(str)
    def _append_control_log(self, line: str) -> None:
        if line:
            self._append_plain(self.control_logbox, line)

    @pyqtSlot(str)
    def _append_abs_lidar_log(self, line: str) -> None:
        self._append_plain(self.abs_lidar_logbox, line)

    @pyqtSlot(str)
    def _append_rel_lidar_log(self, line: str) -> None:
        self._append_plain(self.rel_lidar_logbox, line)

    @pyqtSlot(int)
    def _set_battery(self, bin_value: int) -> None:
        bin_value = max(0, min(9, int(bin_value)))
        percent = int(round(bin_value * 100 / 9))
        self.agv_bat_bar.setValue(percent)

    @pyqtSlot(int)
    def _set_drive_indicator(self, state: int) -> None:
        # Pico state: 0=STOP, 1=FWD, 2=BWD
        labels = {
            MOTOR_FWD: self.state_fwd_label,
            MOTOR_STOP: self.state_stop_label,
            MOTOR_BWD: self.state_bwd_label,
        }
        for label in labels.values():
            label.setStyleSheet(self._LABEL_INACTIVE)
        labels.get(state, self.state_stop_label).setStyleSheet(self._LABEL_ACTIVE)

    # ---------- UI event handlers ----------
    def _on_connect_clicked(self) -> None:
        if self.client.connected:
            self.client.disconnect_from_server()
            return

        ip = self._read_text(self.ip_textedit)
        port_text = self._read_text(self.port_textedit)
        if not ip or not port_text.isdigit():
            self._append_control_log("IP 또는 Port 형식 오류")
            return

        self.client.connect_to_server(ip, int(port_text))

    def _send_mode(self, value: int) -> None:
        self._sync_checked(self.mode_buttons, self.dist_mode_btn if value == 1 else self.manual_mode_btn)
        self.client.send_update("mode", value)

    def _send_drive(self, value: int) -> None:
        active_map = {
            0: self.stop_btn,
            1: self.forward_btn,
            2: self.backward_btn,
            3: self.zero_move_btn,
        }
        self._sync_checked(self.drive_buttons, active_map.get(value, self.stop_btn))
        self.client.send_update("drive", value)

    def _send_speed(self, value: int) -> None:
        self._sync_checked(self.speed_buttons, self.speed_buttons[value])
        self.client.send_update("speed", value)

    def _send_status_call(self, value: int) -> None:
        active_map = {
            0: self.agv_state_btn,
            1: self.lidar_on_btn,
            2: self.lidar_off_btn,
            3: self.lidar_set_btn,
        }
        self._sync_checked(self.status_buttons, active_map.get(value, self.agv_state_btn))
        self.client.send_update("status_call", value)

    def _on_distance_send_clicked(self) -> None:
        txt = self._read_text(self.dist_textedit)
        try:
            meters = float(txt)
            self.client.send_distance(meters)
        except ValueError as e:
            self._append_control_log(f"거리 입력 오류: {e}")

    def _on_distance_reset_clicked(self) -> None:
        self.client.reset_distance()
        self._set_text(self.dist_textedit, "")

    def closeEvent(self, event) -> None:
        self.client.disconnect_from_server()
        super().closeEvent(event)


def main() -> int:
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
