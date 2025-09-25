# #!/usr/bin/env python3
# # jetson_server_compat.py
# import os
# import socket
# import threading
# import time
# import cv2
# from queue import Queue

# from lidar import LiDAR
# from rs485_motor import MotorController
# from linetracing import LineTracer
# from encoder_pico import AGVENCODER

# # ─── 설정 ─────────────────────────────────────────────────────────
# JETSON_HOST      = '0.0.0.0'
# JETSON_CTRL_PORT = 5024
# PARAM_SIZE       = 8

# LIDAR_SERIAL_PORT = '/dev/ttyLiDAR'
# LIDAR_BAUD_RATE   = 115200

# PICO_SERIAL_PORT  = '/dev/ttyPICO'
# PICO_BAUD_RATE    = 115200

# CAMERA_DEVICE     = 0
# # ─────────────────────────────────────────────────────────────────

# # 전역 상태
# sock = None
# command_queue = Queue()
# send_lock = threading.Lock()
# lidar_lock = threading.Lock()

# last_adc = None
# last_sent_state = None

# # 거리 주행/라인트레이싱 상태
# target_distance_mm    = 0
# remaining_distance_mm = 0
# lidar_stream_enabled  = False

# # 장치 상태 보고용
# pico_st  = None
# motor_st = None
# lidar_st = None

# # 배터리 분등
# V_20PIN  = 2.05
# V_99PIN  = 2.85

# # 엔코더 판정
# THRESH_PULSES = 5

# def on_adc_value(adc):
#     global last_adc
#     last_adc = adc

# def pin_voltage_to_soc(v):
#     if v is None:
#         return 0
#     if v <= V_20PIN:
#         return 1
#     if v >= V_99PIN:
#         return 9
#     step = (V_99PIN - V_20PIN) / 8.0
#     idx = int((v - V_20PIN) // step) + 2
#     return max(2, min(9, idx))

# # ─── 디바이스 초기화 (구버전과 동일한 싱글톤 패턴) ─────────────────────────
# def _open_lidar():
#     try:
#         return LiDAR(LIDAR_SERIAL_PORT, LIDAR_BAUD_RATE)
#     except TypeError:
#         try:
#             return LiDAR(LIDAR_SERIAL_PORT)
#         except TypeError:
#             return LiDAR()

# motor   = MotorController(port='/dev/ttyRS485', baudrate=115200)
# lidar   = _open_lidar()
# tracer  = LineTracer(motor)
# encoder = AGVENCODER(tracer, motor, adc_callback=on_adc_value,
#                      port=PICO_SERIAL_PORT, baudrate=PICO_BAUD_RATE)

# # ─── 유틸 ─────────────────────────────────────────────────────────
# def recv_exact(conn, n):
#     buf = bytearray()
#     while len(buf) < n:
#         try:
#             chunk = conn.recv(n - len(buf))
#         except socket.timeout:
#             # 부분 프레임 누적 계속 시도
#             continue
#         if not chunk:
#             return None
#         buf.extend(chunk)
#     return bytes(buf)

# def get_agv_status():
#     global pico_st, motor_st, lidar_st
#     pico_st  = 0 if encoder is not None else 1
#     motor_st = 0 if getattr(motor, 'is_open', False) else 1
#     lidar_st = 0 if lidar is not None else 1
#     return 0 if (pico_st == 0 and motor_st == 0 and lidar_st == 0) else 1

# def send_state_on_command(conn, wait_ms=100, force_state=None):
#     global last_sent_state, last_adc

#     if force_state in (0, 1, 2):
#         state = force_state
#     else:
#         try:
#             l0, r0 = encoder.read_counts()
#         except Exception:
#             l0 = r0 = 0
#         time.sleep(max(0, wait_ms) / 1000.0)
#         try:
#             l1, r1 = encoder.read_counts()
#         except Exception:
#             l1, r1 = l0, r0

#         dl, dr = l1 - l0, r1 - r0
#         mag = abs(dl) + abs(dr)
#         if mag < max(THRESH_PULSES, 1):
#             state = 0
#         else:
#             s = dr - dl
#             state = 1 if s > 0 else 2

#     # (필요시 변화시에만 전송하도록 last_sent_state 체크 가능)
#     with send_lock:
#         # A2: 모터 상태
#         conn.sendall(bytes([0xA2, state & 0xFF, 0x00, 0x00]))
#         # A1: 배터리
#         soc = int(max(0, min(9, pin_voltage_to_soc(last_adc))))
#         conn.sendall(bytes([0xA1, 0x02, 0x00, soc]))
#     last_sent_state = state

# # ─── 네트워킹 ─────────────────────────────────────────────────────
# def handle_client(conn, addr):
#     global lidar_stream_enabled, target_distance_mm, remaining_distance_mm
#     global last_sent_state, lidar

#     print(f"[CTRL] Connected from {addr}")
#     conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
#     conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 3)
#     conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 1)
#     conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 2)
#     # reader는 수신만 → 큐로 전달
#     with conn:
#         while True:
#             try:
#                 param = recv_exact(conn, PARAM_SIZE)
#                 if param is None:
#                     break
#                 # 7바이트 합 체크섬
#                 if (sum(param[:7]) & 0xFF) != param[7]:
#                     continue
#                 command_queue.put((conn, param))
#             except (ConnectionResetError, BrokenPipeError):
#                 print("[CTRL] Connection reset by peer")
#                 break
#             except Exception as e:
#                 print(f"[CTRL] Reader error: {e}")
#                 break

#     # 연결 종료 후 안전 상태로
#     print(f"[CTRL] Client disconnected: {addr}")
#     tracer.auto_move = 0
#     target_distance_mm = 0
#     remaining_distance_mm = 0
#     tracer.traveled_mm = tracer.target_distance_mm = tracer.remaining_distance_mm = 0
#     motor.stop()
#     with command_queue.mutex:
#         command_queue.queue.clear()
#     with lidar_lock:
#         lidar_stream_enabled = False
#     try:
#         lidar.pause()
#     except Exception:
#         pass
#     try:
#         conn.close()
#     except Exception:
#         pass

# def start_ctrl_server():
#     global sock
#     sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#     sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
#     sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 3)
#     sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 1)
#     sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 2)

#     sock.bind((JETSON_HOST, JETSON_CTRL_PORT))
#     sock.listen(2)
#     print(f"[CTRL] Listening on {JETSON_CTRL_PORT}")

#     while True:
#         conn, addr = sock.accept()
#         threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()

# # ─── 디스패처 & 스트림 ───────────────────────────────────────────
# def dispatcher_loop():
#     global target_distance_mm, remaining_distance_mm, lidar_stream_enabled
#     import traceback, sys
#     print("[DISP] dispatcher_loop start")  # 진입 로그
#     try:
#         print("[DISP] servo_on ..."); sys.stdout.flush()
#         motor.servo_on()
#         print("[DISP] servo_on ok"); sys.stdout.flush()

#         print("[DISP] encoder.reset_counter ..."); sys.stdout.flush()
#         encoder.reset_counter()
#         print("[DISP] init done, entering while"); sys.stdout.flush()
        
#         tracer.traveled_mm = 0

#         set_mode = None            # 0=수동, 1=자동
#         last_status_call = None
#         last_dist_set = None
#         remaining = 0
#         hi = mid = lo = 0

#         while True:
#             # print("2")
#             if command_queue.empty():
#                 time.sleep(0.005)
#                 continue
#             conn, param = command_queue.get()

#             b0 = param[0]
#             status_call = param[4]
#             sbc_power   = param[5]

#             # 파라미터 해석
#             if b0 == 0:
#                 drive      = param[1]
#                 speed_sel  = param[2]
#                 rotate     = param[3]
#             elif b0 == 1:
#                 drive      = param[1]
#                 speed_sel  = param[2]
#                 rotate     = 0
#             elif b0 == 0x02:
#                 hi, mid, lo = param[1], param[2], param[3]
#                 drive = speed_sel = rotate = 0
#             else:
#                 drive = speed_sel = rotate = 0

#             # 모드 전환
#             if b0 == 0:
#                 set_mode = 0
#                 motor.prev_encoder_left = motor.prev_encoder_right = 0
#                 motor.get_drive(drive)
#                 send_state_on_command(conn, force_state=drive)
#                 motor.set_manual_mode()
#                 encoder.reset_counter()
#             elif b0 == 1:
#                 set_mode = 1
#                 motor.prev_encoder_left = motor.prev_encoder_right = 0
#                 encoder.reset_counter()
#                 motor.set_auto_mode()

#             # 수동 모드
#             if set_mode == 0:
#                 motor.set_speed_map(speed_sel)
#                 motor.set_rotate_map(rotate)

#                 if b0 == 0x02:
#                     meters = hi
#                     hundredths = mid
#                     thousandths = (lo // 10)
#                     target_distance_mm = meters*1000 + hundredths*10 + thousandths
#                     motor.get_distance(target_distance_mm)
#                     motor.set_remaining_distance(target_distance_mm)
#                     motor.traveled_mm = motor.step_mm = 0
#                     motor.prev_encoder_left = motor.prev_encoder_right = 0
#                     encoder.reset_counter()
#                     last_dist_set = target_distance_mm

#                 if b0 == 0x03:
#                     motor.get_distance(0)
#                     motor.set_remaining_distance(0)
#                     motor.traveled_mm = motor.step_mm = 0
#                     motor.prev_encoder_left = motor.prev_encoder_right = 0
#                     encoder.reset_counter()
#                     target_distance_mm = 0
#                     last_dist_set = 0

#                 if drive == 0:
#                     motor.get_drive(0)
#                     send_state_on_command(conn, force_state=0)
#                 elif drive in (1, 2):
#                     if motor.target_distance_mm > 0:
#                         remaining = motor.remaining_distance_mm
#                         send_state_on_command(conn, force_state=drive)
#                         motor.start_distance_tracing(speed_sel=speed_sel)
#                         motor.on_distance_done = lambda: send_state_on_command(conn, force_state=0)

#                     if remaining > 0:
#                         send_state_on_command(conn, force_state=drive)
#                         motor.start_distance_tracing(speed_sel=speed_sel)
#                         motor.on_distance_done = lambda: send_state_on_command(conn, force_state=0)
#                     elif last_dist_set != motor.target_distance_mm:
#                         send_state_on_command(conn, force_state=drive)
#                         motor.start_distance_tracing(speed_sel=speed_sel)
#                         motor.on_distance_done = lambda: send_state_on_command(conn, force_state=0)
#                     else:
#                         motor.get_drive(drive)
#                         send_state_on_command(conn, force_state=drive)

#             # 자동 모드: 거리 입력/초기화
#             if set_mode == 1 and b0 == 0x02:
#                 meters = hi
#                 hundredths = mid
#                 thousandths = (lo // 10)
#                 target_distance_mm = meters*1000 + hundredths*10 + thousandths
#                 tracer.get_distance(target_distance_mm)
#                 tracer.set_remaining_distance(target_distance_mm)
#                 tracer.traveled_mm = tracer.step_mm = 0
#                 tracer.prev_encoder_left = tracer.prev_encoder_right = 0
#                 encoder.reset_counter()
#                 last_dist_set = target_distance_mm
#                 continue

#             if set_mode == 1 and b0 == 0x03:
#                 tracer.get_distance(0)
#                 tracer.set_remaining_distance(0)
#                 tracer.traveled_mm = tracer.step_mm = 0
#                 tracer.prev_encoder_left = tracer.prev_encoder_right = 0
#                 encoder.reset_counter()
#                 target_distance_mm = 0
#                 last_dist_set = 0
#                 continue

#             # 자동 모드 진행
#             if set_mode == 1:
#                 if drive == 0:
#                     tracer.set_drive_mode(0)
#                     tracer.stop_line_tracing()
#                     tracer.start_distance_tracing(0)
#                     tracer.start_basic_tracing(0)
#                     send_state_on_command(conn, force_state=0)

#                     if tracer.target_distance_mm > 0:
#                         traveled = tracer.traveled_mm
#                         remaining = max(0, tracer.target_distance_mm - traveled)
#                         tracer.set_remaining_distance(remaining)

#                 elif drive in (1, 2):
#                     tracer.set_drive_mode(drive)
#                     send_state_on_command(conn, force_state=drive)
#                     if tracer.no_line:
#                         with send_lock:
#                             conn.sendall(bytes([0xA1, 1, 4, 0]))
#                         send_state_on_command(conn, force_state=0)
#                     else:
#                         with send_lock:
#                             conn.sendall(bytes([0xA1, 0, 0, 0]))
#                         tracer.on_distance_done = lambda: send_state_on_command(conn, force_state=0)

#                     if tracer.target_distance_mm > 0:
#                         remaining = tracer.remaining_distance_mm
#                         tracer.start_distance_tracing(speed_sel=speed_sel)
#                         tracer.on_distance_done = lambda: send_state_on_command(conn, force_state=0)

#                     if remaining > 0:
#                         tracer.start_distance_tracing(speed_sel=speed_sel)
#                         tracer.on_distance_done = lambda: send_state_on_command(conn, force_state=0)
#                     elif last_dist_set != tracer.target_distance_mm:
#                         tracer.start_distance_tracing(speed_sel=speed_sel)
#                         tracer.on_distance_done = lambda: send_state_on_command(conn, force_state=0)
#                     else:
#                         tracer.start_basic_tracing(speed_sel=speed_sel)
#                         tracer.on_distance_done = lambda: send_state_on_command(conn, force_state=0)

#             # 상태/라이다/전원
#             if status_call == 0 and last_status_call != 0:
#                 check = get_agv_status()
#                 if check == 0:
#                     resp = bytes([0xA1, 0, 0, 0])
#                 else:
#                     if pico_st == 1:
#                         resp = bytes([0xA1, 1, 1, 0])
#                     elif motor_st == 1:
#                         resp = bytes([0xA1, 1, 2, 0])
#                     elif lidar_st == 1:
#                         resp = bytes([0xA1, 1, 3, 0])
#                     else:
#                         resp = bytes([0xA1, 1, 0, 0])
#                 with send_lock:
#                     conn.sendall(resp)
#                 last_status_call = 0

#             elif status_call == 1 and last_status_call != 1:
#                 print("[LIDAR] on")
#                 with lidar_lock:
#                     lidar_stream_enabled = True
#                 try:
#                     lidar.resume()
#                 except Exception:
#                     pass
#                 threading.Thread(target=lidar_loop, args=(conn,), daemon=True).start()
#                 last_status_call = 1

#             elif status_call == 2 and last_status_call != 2:
#                 print("[LIDAR] off")
#                 with lidar_lock:
#                     lidar_stream_enabled = False
#                 try:
#                     lidar.pause()
#                 except Exception:
#                     pass
#                 last_status_call = 2

#             elif status_call == 3 and last_status_call != 3:
#                 print("[LIDAR] reset baseline")
#                 try:
#                     lidar.reset_baseline()
#                 except Exception:
#                     pass
#                 last_status_call = 3

#             if sbc_power == 1:
#                 with send_lock:
#                     conn.sendall(b"ACK:AGV down\n")
#                 os.system('sudo shutdown now')
            
#             time.sleep(0.001)
#     except Exception as e:
#         print("[DISP] FATAL in dispatcher_loop:", repr(e))
#         traceback.print_exc()

# def lidar_loop(conn):
#     global lidar_stream_enabled
#     while True:
#         with lidar_lock:
#             if not lidar_stream_enabled:
#                 break
#         try:
#             pkt = lidar.read_frame()
#             if pkt:
#                 with send_lock:
#                     conn.sendall(pkt)
#         except (BrokenPipeError, ConnectionResetError):
#             print("[LIDAR] Client disconnected, stopping stream")
#             with lidar_lock:
#                 lidar_stream_enabled = False
#             break
#         except Exception as e:
#             print(f"[LIDAR] Streaming error: {e}")
#         time.sleep(0.5)

# def camera_loop():
#     cap = cv2.VideoCapture(CAMERA_DEVICE)
#     if not cap.isOpened():
#         print("[CAM] Cannot open camera")
#         return
#     while True:
#         ok, frame = cap.read()
#         if not ok:
#             print("[CAM] Frame grab failed")
#             break

#         if tracer.auto_move == 1:
#             try:
#                 tracer.process_frame(frame)
#             except Exception:
#                 pass
#         else:
#             if tracer.motor.get_mode == 'auto':
#                 tracer.motor.send_speeds(0, 0)
#             try:
#                 tracer.process_frame(frame)
#             except Exception:
#                 pass
#     cap.release()

# # ─── 엔트리포인트 ─────────────────────────────────────────────────
# if __name__ == '__main__':
#     print("[BOOT] starting servers")
#     threading.Thread(target=start_ctrl_server, daemon=True).start()
#     threading.Thread(target=dispatcher_loop,   daemon=True).start()
#     threading.Thread(target=encoder.main,      daemon=True).start()
#     threading.Thread(target=camera_loop,       daemon=True).start()

#     try:
#         while True:
#             time.sleep(1)
#     except KeyboardInterrupt:
#         print("\n[SYS] Shutting down...")
#         try: motor.stop(); motor.servo_off()
#         except Exception: pass
#         try: encoder.stop()
#         except Exception: pass
#         try: lidar.pause(); lidar.close()
#         except Exception: pass
#         try:
#             if sock: sock.close()
#         except Exception: pass