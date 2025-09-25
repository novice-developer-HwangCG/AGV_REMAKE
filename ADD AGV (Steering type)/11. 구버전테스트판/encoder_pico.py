#!/usr/bin/env python3
import time
import serial
import threading

class AGVENCODER:
    def __init__(self, tracer, motor, adc_callback=None, port="/dev/ttyPICO", baudrate=115200):
        """
        :param tracer: linetracing.LineTracer 의 인스턴스. 
                       AGVENCODER가 Pico로부터 받은 펄스 수를 이 객체에 전달함.
        :param port:  Pico가 연결된 UART 포트 (예: '/dev/ttyUSB1')
        :param baudrate: 직렬 통신 보드 레이트
        """
        # LineTracer 인스턴스를 저장
        self.tracer = tracer
        # motor 인스턴스를 저장
        self.motor = motor

        # adc 인스턴스
        self.adc_callback = adc_callback

        self.left_counter  = 0
        self.right_counter = 0
        self.adc_value     = 0.0

        # Pico와 통신할 직렬 포트 초기화
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )

        # 살아 있는지 여부 플래그
        self.serial_alive = True
        self.reset_encoder = False
        self.lock = threading.Lock()

    def tx_pico(self):
        try:
            # 0xFF 한 바이트 전송
            self.ser.write(bytes([0xFF]))
            time.sleep(0.01)  # 전송 후 약간 대기
        except Exception as e:
            print(f"##ERROR: UART write fail: {e}")
            time.sleep(0.1)

    def reset_counter(self):
        self.reset_encoder = True

    def reset_call(self):
        try:
            self.ser.write(bytes([0x00]))
            time.sleep(0.01)
        except Exception as e:
            print(f"##ERROR: UART write fail: {e}")
            time.sleep(0.1)

    def main(self):
        while self.serial_alive:
            self.tx_pico()

            if self.reset_encoder == True:
                self.reset_call()
                self.reset_encoder = False

            # Pico로부터 돌아오는 엔코더 데이터 읽기 (한 번에 읽기 pico에서 보내는 형식'sprintf(buffer, "%d,%d\n", l_encoder, r_encoder);')
            raw = self.ser.readline()
            if raw:
                try:
                    # 바이트를 문자열로 디코딩하고, 맨 끝 개행(\n) 제거
                    line = raw.decode('utf-8', errors='ignore').strip()
                    # 쉼표로 구분된 두 정수(좌/우 펄스) 분리
                    parts = line.split(',')
                    if len(parts) == 3:
                        left_pulse  = int(parts[0])
                        right_pulse = int(parts[1])
                        adc = float(parts[2])
                        self.left_counter = left_pulse
                        self.right_counter = right_pulse
                        self.adc_value = adc
                        # print(self.left_counter, self.right_counter)
                        # print(self.adc_value)
                        # LineTracer에 펄스 값을 전달
                        if self.tracer.auto_move != 0:
                            self.tracer.update_encoder(self.left_counter, self.right_counter)
                        # Motor에게 펄스 값을 전달
                        if self.motor.manual_drive != 0:
                            self.motor.update_encoder_manual(self.left_counter, self.right_counter)
                        if self.adc_callback:
                            self.adc_callback(adc)
                    else:
                        # 형식이 예상과 다를 경우 무시
                        print(f"##WARNING: Unexpected encoder format: '{line}'")
                except ValueError:
                    print(f"##WARNING: Cannot parse encoder data: '{line}'")
                except Exception as e:
                    print(f"##ERROR: Cannot deliver to LineTracer: {e}")
                except serial.SerialException as e:
                    print(f"[ENCODER] Serial Exception: {e}")
                    break   # 루프 탈출
            time.sleep(0.01)

    def read_counts(self):
        with self.lock:
            return self.left_counter, self.right_counter

    def stop(self):
        self.serial_alive = False
        # try:
        #     if self.ser.is_open:
        #         self.ser.close()
        # except Exception as e:
        #     print(f"[ENCODER] Error closing serial: {e}")

if __name__ == "__main__":
    # 테스트용 스텁: LineTracer 대신 간단히 print만 하는 더미 객체
    # class DummyTracer:
    #     def update_encoder(self, pulse):
    #         print(f"[DummyTracer] Pulse received: {pulse}")

    # tracer = DummyTracer()
    agv_encoder = AGVENCODER()
    try:
        agv_encoder.main()
    except KeyboardInterrupt:
        agv_encoder.stop()
        print("Encoder loop stopped.")
