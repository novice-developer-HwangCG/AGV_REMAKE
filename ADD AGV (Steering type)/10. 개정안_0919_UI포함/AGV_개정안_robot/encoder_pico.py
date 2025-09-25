#!/usr/bin/env python3
# encoder_pico.py
# from __future__ import annotations

import time
import serial
import threading
import re
from typing import Optional, Tuple


class AGVENCODER:
    """
    Pico로부터 엔코더/ADC를 읽어 LineTracer와 MotorController에 전달.

    공개 API(서버에서 사용):
      - main()                      # 블로킹 루프 (외부 스레드로 실행)
      - reset_counter()             # Pico 엔코더 카운터 리셋 요청
      - read_counts() -> (L, R)     # 스레드세이프 카운터 조회
      - stop()                      # 루프/시리얼 종료

    동작:
      - 주기적으로 0xFF를 송신해 한 줄 응답을 요청.
      - 응답 포맷: "left,right,adc" 또는 "left,right,adc,T=123456 us" 지원.
      - auto(라인트레이싱) 주행 중이면 tracer.update_encoder(), 수동 주행이면 motor.update_encoder_manual() 호출.
      - ADC는 읽을 때마다 adc_callback(adcV) 호출(상위에서 SoC 변환/이벤트 제어).
    """

    RE_FLOAT = re.compile(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?")

    def __init__(
        self,
        tracer,
        motor,
        adc_callback=None,
        port: str = "/dev/ttyPICO",
        baudrate: int = 115200,
        timeout: float = 0.1,
        debug: bool = False,
    ):
        self.tracer = tracer
        self.motor = motor
        self.adc_callback = adc_callback
        self.debug = debug

        # 상태
        self.left_counter: int = 0
        self.right_counter: int = 0
        self.adc_value: float = 0.0

        self._alive = threading.Event()
        self._alive.set()
        self._need_reset = threading.Event()
        self._lock = threading.Lock()

        # 시리얼
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )

    # ── public helpers ───────────────────────────────────────────────────────
    def reset_counter(self):
        """다음 루프에서 리셋 커맨드(0x00)를 전송."""
        self._need_reset.set()

    def read_counts(self) -> Tuple[int, int]:
        with self._lock:
            return self.left_counter, self.right_counter

    # ── core loop ────────────────────────────────────────────────────────────
    def main(self):
        """블로킹 루프 — 서버에서 daemon thread로 실행."""
        try:
            while self._alive.is_set():
                # 1) 데이터 요청
                self._write_byte(0xFF)

                # 2) 필요 시 리셋
                if self._need_reset.is_set():
                    self._write_byte(0x00)
                    self._flush_and_zero_local()
                    self._need_reset.clear()

                # 3) 한 줄 수신 & 파싱
                line = self._readline()
                if line is not None:
                    parsed = self._parse_line(line)
                    if parsed is not None:
                        l, r, adc = parsed
                        # 공유 상태 갱신 (락 보호)
                        with self._lock:
                            self.left_counter = l
                            self.right_counter = r
                            self.adc_value = adc

                        # 상위 콜백(SoC 변환은 상위에서)
                        if self.adc_callback:
                            try:
                                self.adc_callback(adc)
                            except Exception:
                                pass

                        # 주행 모드에 따라 전달
                        try:
                            if getattr(self.tracer, "auto_move", 0) != 0:
                                self.tracer.update_encoder(l, r)
                        except Exception as e:
                            if self.debug:
                                print(f"[ENC] tracer.update_encoder error: {e}")

                        try:
                            if getattr(self.motor, "manual_drive", 0) != 0:
                                self.motor.update_encoder_manual(l, r)
                        except Exception as e:
                            if self.debug:
                                print(f"[ENC] motor.update_encoder_manual error: {e}")

                time.sleep(0.01)  # 100Hz 주기
        except serial.SerialException as e:
            print(f"[ENCODER] Serial exception: {e}")
        except Exception as e:
            print(f"[ENCODER] Loop error: {e}")
        finally:
            self.stop()

    # ── io helpers ───────────────────────────────────────────────────────────
    def _write_byte(self, b: int):
        try:
            self.ser.write(bytes([b & 0xFF]))
        except Exception as e:
            if self.debug:
                print(f"[ENC] UART write fail: {e}")
            time.sleep(0.05)

    def _flush_and_zero_local(self):
        """리셋 직후 입력 버퍼를 비우고 로컬 카운터를 0으로 동기."""
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        with self._lock:
            self.left_counter = 0
            self.right_counter = 0
        # 트레이서/모터 쪽은 서버에서 prev_* 초기화 수행

    def _readline(self) -> Optional[str]:
        try:
            raw = self.ser.readline()
            if not raw:
                return None
            return raw.decode("utf-8", errors="ignore").strip()
        except Exception as e:
            if self.debug:
                print(f"[ENC] readline error: {e}")
            return None

    def _parse_line(self, line: str) -> Optional[Tuple[int, int, float]]:
        """
        허용 포맷:
          - "123,456,2.73"
          - "123,456,2.73,T=123456 us"
        """
        try:
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 3:
                if self.debug:
                    print(f"[ENC] unexpected format: '{line}'")
                return None

            l = int(parts[0], 10)
            r = int(parts[1], 10)

            # 3번째 토큰에서 부동소수 추출(단위/문자 혼입 대비)
            m = self.RE_FLOAT.search(parts[2])
            if not m:
                if self.debug:
                    print(f"[ENC] adc parse fail: '{parts[2]}' in '{line}'")
                return None
            adc = float(m.group(0))

            return l, r, adc
        except Exception:
            if self.debug:
                print(f"[ENC] parse error: '{line}'")
            return None

    # ── shutdown ─────────────────────────────────────────────────────────────
    def stop(self):
        self._alive.clear()
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception as e:
            print(f"[ENCODER] Error closing serial: {e}")


if __name__ == "__main__":
    # 단독 테스트 시, 더미 tracer/motor를 만들어도 됨.
    pass
