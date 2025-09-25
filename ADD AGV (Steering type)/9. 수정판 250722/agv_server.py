#!/usr/bin/env python3
# jetson_server.py
import os
import socket
import threading
import time
import cv2
from queue import Queue

from lidar import LiDAR
from rs485_motor import MotorController
from linetracing import LineTracer
from encoder_pico import AGVENCODER

# ─── 설정 ─────────────────────────────────────────────────────────
JETSON_HOST      = '0.0.0.0'
JETSON_CTRL_PORT = 5024
PARAM_SIZE       = 8

# LiDAR 포트 (읽기 연결 여부 판단용)
LIDAR_SERIAL_PORT = '/dev/ttyLiDAR'
LIDAR_BAUD_RATE   = 115200

# PIOC 포트 (읽기 연결 여부 판단용)
PICO_SERIAL_PORT = '/dev/ttyPICO'
PICO_BAUD_RATE   = 115200

# 카메라 포트 (읽기 연결 여부 판단용)
# CAMERA_SERIAL_PORT = '/dev/video0'
CAMERA_DEVICE    = 0   # '/dev/video0' 대신 0 (리눅스 디바이스 인덱스)
# ─────────────────────────────────────────────────────────────────

last_adc = None
sock = None
last_sent_state = None

V_20PIN  = 2.05   # 20%
V_99PIN = 2.85   # 100%

def on_adc_value(adc):
    global last_adc
    last_adc = adc
    # print(f"[ADC] {adc:.2f}")

def pin_voltage_to_soc(Vpin: float) -> int:
    if last_adc <= 2.05: return 1
    elif last_adc < 2.107: return 2
    elif last_adc < 2.221: return 3
    elif last_adc < 2.336: return 4
    elif last_adc < 2.450: return 5
    elif last_adc < 2.564: return 6
    elif last_adc < 2.679: return 7
    elif last_adc < 2.793: return 8
    else: return 9

motor = MotorController(port='/dev/ttyRS485', baudrate=115200)
lidar = LiDAR(port=LIDAR_SERIAL_PORT, baud=LIDAR_BAUD_RATE)
tracer = LineTracer(motor)
encoder = AGVENCODER(tracer, motor, adc_callback=on_adc_value)
command_queue = Queue()
lidar_lock = threading.Lock()

lidar_stream_enabled = True
target_distance_mm    = 0       # (A) UI에서 “거리 입력(0x02)” 시 설정
remaining_distance_mm = 0       # (B) 실제 남은 거리(정지/재개 시 갱신)

# 점검에 따라 ui측에게 어느 것이 문제가 생겼는지 알려주기 위해 상태 값 저장 변수
pico_st = None
motor_st = None
lidar_st = None

prev_left, prev_right = 0, 0
THRESH_PULSES = 5  # 0.3초 동안 최소 펄스 수 임계값

def get_agv_status():
    global lidar, motor_st, lidar_st
    try:
        # agv 상태 점검 정상이면 0, 불량이면 1 (젯슨의 포트 상태)
        if encoder is not None:
            pico_st = 0
        else:
            pico_st = 1
        # 다음 모터 포트 상태 점검 정상이면 0, 불량이면 1 (rs485 usb 포트가 연결되어 있는지)
        if motor.is_open:
            motor_st = 0
        else:
            motor_st = 1
        # 다음 라이다 포트 상태 점검(모터는 is_open, 라이다는 is not None) 정상이면 0, 불량이면 1 (라이다 usb 포트가 연결되어 있는지)
        if lidar is not None:
            lidar_st = 0
        else:
            lidar_st = 1
        # 다음 카메라 포트 상태 점검 이 부분은 대기
        # 모든 상태가 정상이면 0 하나라도 불량이라면 1 보내기
        if pico_st == 0 and motor_st == 0 and lidar_st == 0:
            agv_check = 0
        else:
            agv_check = 1
    except (BrokenPipeError):
        return
    return agv_check

def recv_exact(conn, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)

def handle_client(conn, addr):
    global lidar_stream_enabled, target_distance_mm, remaining_distance_mm
    print(f"[CTRL] Connected from {addr}")
    with conn:
        while True:
            try:
                param = recv_exact(conn, PARAM_SIZE)
                if param is None:
                    break
                checksum = sum(param[:7]) & 0xFF
                if param[7] != checksum:
                    continue
                command_queue.put((conn, param))
            except socket.timeout:
                print("[EMERGENCY] comms timeout")
                tracer.auto_move = 0
                motor.stop()
                break
            except ConnectionResetError:
                print("[CTRL] Connection reset by peer")
                tracer.auto_move = 0
                motor.stop()
                break
            except Exception as e:
                print(f"[CTRL] Error: {e}")
                tracer.auto_move = 0
                motor.stop()
                break
    # 클라이언트가 정상적으로 빠져나갈 시
    print(f"[CTRL] Client disconnected: {addr}")
    tracer.auto_move = 0
    target_distance_mm = 0
    remaining_distance_mm = 0
    tracer.traveled_mm = 0
    tracer.target_distance_mm = 0
    tracer.remaining_distance_mm = 0
    motor.stop()
    with command_queue.mutex:
        command_queue.queue.clear()
    # motor.servo_off() # 해당 부분은 클라이언트가 재접속 할 수 있으니 servo를 꺼두면 안됨 이유는 servo 끄면 다시 servo on신호를 받을 때 까지 제어가 불가능함
    lidar_stream_enabled = False        # 스트리밍 루프 종료 플래그 내리기
    try:
        lidar.close()                   # 포트 닫기
    except Exception as e:
        print(f"[LIDAR] Error closing port: {e}")
    encoder.stop()
    conn.close()

def dispatcher_loop():
    global target_distance_mm, remaining_distance_mm, lidar_stream_enabled
    global last_adc, pico_st, motor_st, lidar_st

    set_mode = None        # 0=수동, 1=자동
    last_status_call = None # 이전에 받은 status_call 값 저장
    last_dist_set = None
    remaining = 0
    hi = 0
    mid = 0
    lo = 0

    motor.servo_on()    # 서버 시작 시 servo_on 신호를 보내어 모터를 제어할 수 있도록 하기
    encoder.reset_counter()
    tracer.traveled_mm = 0
    while True:
        if command_queue.empty():
            time.sleep(0.01)
            continue
        conn, param = command_queue.get()

        b0 = param[0]
        status_call = param[4]
        sbc_power = param[5]

        if b0 == 0xFF:
            print("SERVER SHUTDOWN")
            # sock.close()

        # 헤더 값에 따라 파라미터 값 받기 변화 이유 거리 값을 줄 시 거리 값으로 인해 속도 값과 회전 값을 잘못 받음
        if b0 == 0:
            drive = param[1]
            speed_sel = param[2]
            rotate = param[3]
        elif b0 == 1:
            drive = param[1]
            speed_sel = param[2]
            rotate = 0
        elif b0 == 0x02:
            hi = param[1]
            mid = param[2]
            lo = param[3]
            pass

        # print(f"[SERVER]speed_sel : {speed_sel}")
        # print(f"[SERVER]rotate : {rotate}")

        # (D) 모드 전환: 0=수동, 1=자동
        # 목표 거리 값 기반 라인트레이싱 중 주행 정지나 선이 없어서 정지할 때 남은 목표 거리 값을 저장하는데 이 때 수동으로 전환해서 라인을 다시 찾거나 로봇 위치를 재조정 이후 다시 거리 값 기반 주행 명령을 내릴 때 초기화 되어서는 안되니 이 부분 생각 다시 해봐야 함
        if b0 == 0:
            set_mode = 0
            tracer.prev_encoder_left  = 0
            tracer.prev_encoder_right = 0
            motor.get_drive(drive)
            motor.set_manual_mode()
            encoder.reset_counter()
            # conn.sendall(b"ACK:MANUAL\n")
            # continue
        elif b0 == 1:
            set_mode = 1
            motor.prev_encoder_left  = 0
            motor.prev_encoder_right = 0
            encoder.reset_counter()
            motor.set_auto_mode()   # rs485_motor.py에게 현재 자동 모드임을 알리기 위해 함수 호출하여 mode를 'auto'로 바꿈
            # conn.sendall(b"ACK:AUTO\n")
            # continue

        # (C) 수동 모드 처리: 자동 모드 동일 drive=0 정지, drive=1 전진, drive=2이면 후진
        if set_mode == 0:
            motor.set_speed_map(speed_sel)
            motor.set_rotate_map(rotate)
            if b0 == 0x02:
                # print(hi, mid, lo)
                meters = hi
                hundredths = mid
                ten_thousandths = lo
                thousandths = ten_thousandths // 10

                target_distance_mm = meters*1000 + hundredths*10 + thousandths
                motor.get_distance(target_distance_mm)
                motor.set_remaining_distance(target_distance_mm)
                motor.traveled_mm = 0
                motor.prev_encoder_left  = 0
                motor.prev_encoder_right = 0
                encoder.reset_counter()
                last_dist_set = target_distance_mm  # 이전 거리 기억
                # conn.sendall(b"ACK:DIST\n")
                msg = f"SET_DIST:{target_distance_mm}\n".encode('utf-8')
                # conn.sendall(msg)
                continue

            if b0 == 0x03:
                # 남은 거리/전체 거리/누적 거리 모두 초기화
                motor.get_distance(0)
                motor.set_remaining_distance(0)
                motor.traveled_mm = 0
                motor.prev_encoder_left  = 0
                motor.prev_encoder_right = 0
                encoder.reset_counter()
                target_distance_mm = 0
                last_dist_set = 0
                print("[CTRL] Distance clear, Manual tracing")
                # conn.sendall(b"ACK:DCLR\n")
                continue
            if drive == 0:
                # drive != 1 (즉 drive==0) → 무조건 정지
                send_state_on_command(conn, 0)
                motor.get_drive(0)
            elif drive in (1, 2):
                # 목표 거리값이 있다면 목표 거리 값 기반 일반 주행
                send_state_on_command(conn, drive)
                if motor.target_distance_mm > 0:
                    remaining = motor.remaining_distance_mm
                    motor.start_distance_tracing(speed_sel=speed_sel)
                    print(f"[CTRL] Start Distance Tracing (Distance = {motor.target_distance_mm})")
                    
                # 목표 거리 값 기반 주행 중 정지 후 다시 주행 명령을 내리면 남은 목표 거리 값으로 주행
                if remaining > 0:
                    print(f"[CTRL] Resume Distance Tracing ({remaining} mm)")
                    motor.start_distance_tracing(speed_sel=speed_sel)

                # 새로운 목표 거리 값을 받으면 이전에 받은 목표 거리 값은 초기화 새로운 목표 거리 값으로 주행
                elif last_dist_set != motor.target_distance_mm:
                    print("[CTRL] New Distance Input Detected. Starting Fresh Distance Trace.")
                    motor.start_distance_tracing(speed_sel=speed_sel)
                # 목표 거리 값이나 남은 목표 거리 값 등 받은게 없다면 기본 조작
                else:
                    motor.get_drive(drive)
                    continue
            # continue

        # (A) 거리 입력 값 받기
        if set_mode == 1:
            if b0 == 0x02:
                # print(hi, mid, lo)
                meters = hi
                hundredths = mid
                ten_thousandths = lo
                thousandths = ten_thousandths // 10

                target_distance_mm = meters*1000 + hundredths*10 + thousandths
                tracer.get_distance(target_distance_mm)
                tracer.set_remaining_distance(target_distance_mm)
                tracer.traveled_mm = 0
                tracer.prev_encoder_left  = 0
                tracer.prev_encoder_right = 0
                encoder.reset_counter()
                last_dist_set = target_distance_mm  # 이전 거리 기억
                # conn.sendall(b"ACK:DIST\n")
                msg = f"SET_DIST:{target_distance_mm}\n".encode('utf-8')
                # conn.sendall(msg)
                continue

        # (B) 입력 받은 거리 값 및 남은 거리 값 초기화
        if set_mode == 1:
            if b0 == 0x03:
                # 남은 거리/전체 거리/누적 거리 모두 초기화
                tracer.get_distance(0)
                tracer.set_remaining_distance(0)
                tracer.traveled_mm = 0
                tracer.prev_encoder_left  = 0
                tracer.prev_encoder_right = 0
                encoder.reset_counter()
                target_distance_mm = 0
                last_dist_set = 0
                print("[CTRL] Distance clear, line tracing")
                # conn.sendall(b"ACK:DCLR\n")
                continue

        # (E) 자동 모드 진행: ui로부터 받은 거리 값이 있다면 그 거리 값으로 라인트레이싱 명령 없다면 기본 라인트레이싱 시작
        if set_mode == 1:
            # drive 값은 주행 방향과 함께 주행 시작 명령이 함께 있음 0 = 정지, 1 = 전진과 함께 주행 시작, 2 = 후진과 함께 주행 시작
            if drive == 0:
                print("debug auto stop")
                tracer.set_drive_mode(drive)       # drive 상태가 0임을 알림(대기 or 정지)
                tracer.stop_line_tracing()
                tracer.start_distance_tracing(0)
                tracer.start_basic_tracing(0)
                send_state_on_command(conn, 0)

                traveled = tracer.traveled_mm
                if tracer.target_distance_mm > 0:
                    remaining = max(0, tracer.target_distance_mm - traveled)
                    tracer.set_remaining_distance(remaining)
            elif drive in (1, 2):
                # drive == 1 :
                print("debug auto run")
                tracer.set_drive_mode(drive) # drive 상태가 1 또는 2임을 알림(주행 시작)
                send_state_on_command(conn, 1)
                # 라인트레이싱 하기 전 받은 거리 값이 있다면 거리 값으로 라인트레이싱 없다면 기본 라인트레이싱
                if tracer.no_line:
                    conn.sendall(bytes([0xa1, 1, 4, 0]))   # 라인트레이싱을 하라는 명령이 내려왔는데 선이 없다면 선이 없다는 오류 프로토콜 전송
                else:
                    conn.sendall(bytes([0xa1, 0, 0, 0]))
                    pass                            # 선이 있다면 정상 신호 보내고 넘어가기

                # 목표 거리값이 있다면 목표 거리 값 기반 라인트레이싱
                if tracer.target_distance_mm > 0:
                    remaining = tracer.remaining_distance_mm
                    tracer.start_distance_tracing(speed_sel=speed_sel)
                    print(f"[CTRL] Start Distance Line Tracing (Distance = {tracer.target_distance_mm})")
                    
                # 목표 거리 값 기반 라인트레이싱 중 정지하거나 선이 없어서 주행이 멈췄을 때 다시 주행 명령을 내리면 남은 목표 거리 값으로 주행
                if remaining > 0:
                    print(f"[CTRL] Resume Distance Line Tracing ({remaining} mm)")
                    tracer.start_distance_tracing(speed_sel=speed_sel)

                # 새로운 목표 거리 값을 받으면 이전에 받은 목표 거리 값은 초기화 새로운 목표 거리 값으로 주행
                elif last_dist_set != tracer.target_distance_mm:
                    print("[CTRL] New Distance Input Detected. Starting Fresh Distance Trace.")
                    tracer.start_distance_tracing(speed_sel=speed_sel)
                # 목표 거리 값이나 남은 목표 거리 값 등 받은게 없다면 기본 라인트레이싱
                else:
                    print("[CTRL] No Distance Set. Default Line Tracing.")
                    tracer.start_basic_tracing(speed_sel=speed_sel)
                    continue
            else:
                pass
        else:
            pass

        # AGV 상태를 알려달라는 신호 파라미터를 받으면 get_agv_status()확인 후 ui로 정상인지 불량인지에 대한 프레임 값 전달
        if status_call == 0:
            check = get_agv_status()
            if check == 0:
                resp = bytes([0xA1, 0, 0, 0])
            else:
                # AGV 상태가 오류임을 알림
                if pico_st == 1:
                    resp = bytes([0xA1, 1, 1, 0])      # pico 포트가 오류라는 프로토콜 전송
                elif motor_st == 1:
                    resp = bytes([0xA1, 1, 2, 0])      # 485 포트가 오류라는 프로토콜 전송
                elif lidar_st == 1:
                    resp = bytes([0xA1, 1, 3, 0])      # lidar 포트가 오류라는 프로토콜 전송
                else:
                    pass
            last_status_call = 0
            conn.sendall(resp)
        elif status_call == 1 and last_status_call != 1:
            # 라이다를 on 시키는 신호 파라미터를 받으면 lidar 값 보내기기
            print("lidar on")
            lidar_stream_enabled = True
            lidar.resume()  # 절대 모드로
            threading.Thread(target=lidar_loop, args=(conn,), daemon=True).start()
            last_status_call = 1
        elif status_call == 2 and last_status_call != 2:
            # 라이다를 off 시키는 신호 파라미터를 받으면 lidar 값 보내지 말고 대기 또는 무시
            print("lidar off")
            lidar_stream_enabled = False
            lidar.pause()   # 라이다 스트림 중지
            # conn.sendall(b"ACK:LIDAR_OFF\n")
            last_status_call = 2
        elif status_call == 3 and last_status_call != 3:
            print("lidar reset")
            lidar.reset_baseline()  # baseline 갱신 → relative 모드
            # conn.sendall(b"ACK:LIDAR_RESET\n")
            last_status_call = 3

        if sbc_power == 1:
            # print("[DEBUG] Power Off command received → shutting down Jetson")
            conn.sendall(b"ACK:AGV down\n")
            os.system('sudo shutdown now')
            if last_adc is not None:
                soc = pin_voltage_to_soc(last_adc)
                resp = bytes([0xA1, 2, 0x00, 0x00])
                conn.sendall(resp)
                if soc <= 2:
                    conn.sendall(b"WARNING: Battery level is at or below 20%\n")
            else:
                conn.sendall(b"ACK:ADC:ERR\n")
        else:
            pass
        time.sleep(0.001)

def lidar_loop(conn):
    global lidar_stream_enabled
    while True:
        with lidar_lock:
            if not lidar_stream_enabled:
                break
        try:
            lidar_distance = lidar.read_frame()
            if lidar_distance:
                conn.sendall(lidar_distance)
        except (BrokenPipeError, ConnectionResetError):
            print("[LIDAR] Client disconnected, stopping stream")
            with lidar_lock:
                lidar_stream_enabled = False
            break
        except Exception as e:
            print(f"[LIDAR] Streaming error: {e}")
        time.sleep(0.5)

def camera_loop():
    cap = cv2.VideoCapture(CAMERA_DEVICE)
    if not cap.isOpened():
        print("[CAM] Cannot open camera")
        return
    while True:
        ret, frame = cap.read()
        if tracer.auto_move == 1:
            if not ret:
                print("[CAM] Frame grab failed")
                break
            vis, _ = tracer.process_frame(frame)
        else:
            if tracer.motor.get_mode == 'auto':
                tracer.motor.send_speeds(0, 0)
            vis, _ = tracer.process_frame(frame)

        #3) 화면 표시 및 키 이벤트 처리
        #print(f"[SERVER] line_tracer.auto_move = {tracer.auto_move}")
        #cv2.imshow("LineTracer", vis)
        # # 1ms 대기, 'q' 누르면 루프 탈출
        #if cv2.waitKey(1) & 0xFF == ord('q'):
           #break
    cap.release()

def send_state_on_command(conn, drive):
    """
    UI 명령을 처리하는 시점에만, 모터 상태 변화가 있으면
    - 0xA2: (정지=0, 전진=1, 후진=2)
    - 0xA1: 배터리 바이트(0x02, 0x00, soc)
    를 전송한다.
    """
    global last_sent_state, last_adc
    if drive not in (0, 1, 2):
        return
    state = 0 if drive == 0 else (1 if drive == 1 else 2)
    if state == last_sent_state:
        return  # 변화 없으면 아무것도 안 보냄

    try:
        # 모터 상태
        conn.sendall(bytes([0xA2, state, 0x00, 0x00]))
        # 배터리 바이트 (상태가 바뀔 때만 보냄)
        if last_adc is not None:
            soc = pin_voltage_to_soc(last_adc)
            soc = int(max(0, min(9, soc)))
            conn.sendall(bytes([0xA1, 0x02, 0x00, soc]))
    except Exception:
        pass

    last_sent_state = state

def start_ctrl_server():
    global sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 서버 코드를 종료하고 다시 실행해도 서버 주소를 이미 쓰고 있다는 오류를 없애기 위함
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 3) # 로봇의 속도가 저속일때면 5초정도가 괜찮은데 혹여나 고속 일때 끊길 수 있으니 3초로 
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 1)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 2)

    sock.bind((JETSON_HOST, JETSON_CTRL_PORT))
    sock.listen(1)
    print(f"[CTRL] Listening on {JETSON_CTRL_PORT}")
    while True:
        conn, addr = sock.accept()
        threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()

if __name__ == '__main__':
    # 1) 제어·라이다 서버 기동
    threading.Thread(target=start_ctrl_server, daemon=True).start()
    threading.Thread(target=dispatcher_loop, daemon=True).start()
    threading.Thread(target=encoder.main, daemon=True).start()
    threading.Thread(target=camera_loop, daemon=True).start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down servers")
        if sock:
            sock.close()
        motor.stop()
        motor.servo_off()
        encoder.stop()
        lidar.close()
