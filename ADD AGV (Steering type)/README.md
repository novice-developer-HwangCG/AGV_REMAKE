<--- 통신 프로토콜 (구) --->

노트북 → 젯슨
Byte0 │ Mode       │ 0(Manual), 1(Auto), 2(Dist), 3(Dist reset)
Byte1 │ Drive      │ 0(Stop), 1(FWD), 2(BWD) / if Byte0 == 2 (hi)
Byte2 │ SpeedSel   │ 0 ~ 9                   / if Byte0 == 2 (mid)
Byte3 │ Rotate     │ 0(None), 1(LFT), 2(RHT) / if Byte0 == 2 (lo)
Byte4 │ Status_Call│ 0(AGV), 1(LiDAR on), 2(LiDAR off), 3(LiDAR reset)
Byte5 │ Power      │ —  0(Null), 1(SBC Power off), 2(ADC/BAT Call) 
Byte6 │ Reserved3  │ —  (예비 우선은 0)
Byte7 │ Checksum   │ sum[0..6] & 0xFF 

젯슨 → 노트북 (AGV)
Byte0 │ Frame Header  │ AGV=0xA1, 
Byte1 │ Status_AGV    │ 0=정상, 1=AGV 상태불량, 2=BAT
Byte2 │ Error case    │ 1(Pico), 2(motor), 3(LiDAR), 4(None line)
Byte3 │ None          │ 0

Byte0 │ Frame Header  │ LIDAR=0x5A
Byte1 │ Status_AGV    │ 0=정상, 2=LiDAR 상태불량
Byte2 │ Dist H   │ 거리 값(mm) 앞 두 자리 (예. 00XXXX)
Byte3 │ Dist H   │ 거리 값(mm) 중간 두 자리 (예. XX00XX)
Byte4 │ Dist L   │ 거리 값(mm) 뒤 두 자리 (예. XXXX00)

<--- 통신 프로토콜 (신 250924) --->

노트북 → 젯슨
Byte0 │ Mode       │ 0(Manual), 1(Auto), 2(Dist), 3(Dist reset)
Byte1 │ Drive      │ 0(Stop), 1(FWD), 2(BWD) / if Byte0 == 2 (hi)
Byte2 │ SpeedSel   │ 0 ~ 9                   / if Byte0 == 2 (lo)
Byte3 │ Rotate     │ 0(None), 1(LFT), 2(RHT)
Byte4 │ Status_Call│ 0(AGV), 1(LiDAR on), 2(LiDAR off), 3(LiDAR reset)
Byte5 │ Power      │ —  0(Null), 1(SBC Power off), 2(ADC/BAT Call) 
Byte6 │ Reserved3  │ —  (예비 우선은 0)
Byte7 │ Checksum   │ sum[0..6] & 0xFF 

젯슨 → 노트북 (AGV)
Byte0 │ Frame Header  │ AGV=0xA1
Byte1 │ Status_AGV    │ 0=정상, 1=AGV 상태불량, 2=BAT
Byte2 │ Error case    │ 1(Pico), 2(motor), 3(LiDAR), 4(None line)
Byte3 │ Battery       │ 0(None), 1(20% 이하), 2(20%), ... ,9(90% 이상)

Byte0 │ Frame Header  │ LIDAR=0x5A
Byte1 │ Status_AGV    │ 0=정상, 2=LiDAR 상태불량
Byte2 │ Dist H   │ 거리 값(mm) 앞 두 자리 (예. 00XXXX)
Byte3 │ Dist H   │ 거리 값(mm) 중간 두 자리 (예. XX00XX)
Byte4 │ Dist L   │ 거리 값(mm) 뒤 두 자리 (예. XXXX00)
Byte5 │ Sign     │ 0(양수 +), 1(음수 -)

Byte0 │ Frame Header  │ AGV MOTOR=0xA2
Byte1 │ Motor status  │ 0=정지, 1=전진, 2=후진
Byte2 │ None          │ 0(None)
Byte3 │ None          │ 0(None)

<--- 각 코드 별 역할 --->
agv_server.py = 서버 코드 main 역할 (ui에 받은 파라미터 값 처리)
encoder_pico.py = pico에게로 부터 ADC 값 및 엔코더 값 받기
lidar.py = lidar 데이터 받기
linetracing.py = 카메라 처리 담당 linetracing
rs485_motor.py = 모터에게 속도값을 전달하는 코드


<--- 참고 --->
- 엔코더 값 출력 (좌측 바퀴와 우측 바퀴는 반대 방향으로 배치 되어 있음 즉, 좌측 모터는 속도 값을 음수, 우측 모터 속도 값은 양수여야 로봇은 앞으로 전진 함)
void encoder_irq_handler(uint gpio, uint32_t events)
{
    if (gpio == L_ENCODER_A)
        L_encoderPos += (gpio_get(L_ENCODER_A) == gpio_get(L_ENCODER_B) ? 1 : -1);
    else if (gpio == R_ENCODER_A)
        R_encoderPos += (gpio_get(R_ENCODER_A) == gpio_get(R_ENCODER_B) ? -1 : 1);
}

- 엔코더 코드를 적용 로봇을 전진 시켰을 시 출력된 엔코더 값
encoder L, R: 58, 85
encoder L, R: 73, 101
encoder L, R: 91, 116
encoder L, R: 108, 129
encoder L, R: 128, 145
encoder L, R: 148, 161
encoder L, R: 168, 180
encoder L, R: 186, 200
encoder L, R: 203, 221 ...

- 바퀴 직경 0.12m
- 기어비 10:1
- 엔코더 해상도 1000ppr
- 모터 최대 속도 1.2308m/s (기어비 적용 250rpm)
- 차륜 궤간 0.57m
- 바퀴 두께 0.05m
- 좌측 모터 정방향 회전 → 후진, 역뱡향 회전 → 전진
- 우측 모터 역뱡향 회전 → 후진, 정방향 회전 → 전진

모터 구간별 속도 (이론상)
250 = 0.123m/s = 0.4431km/h
500 = 0.2462m/s = 0.8862km/h
750 = 0.3692m/s = 1.3293km/h
1000 = 0.4923m/s = 1.7724km/h
1250 = 0.6154m/s = 2.2154km/h
1500 = 0.7385m/s = 2.6585km/h
1750 = 0.8616m/s = 3.1016km/h
2000 = 0.9846m/s = 3.5447km/h
2250 = 1.1077m/s = 3.9878km/h

2500rpm 주었을 때 최대 속도 실측 결과 약 5.67km/h

<--- 로직 우선 순위 --->
- 수동 모드 일 경우 회전 우선 처리 (전진 후진을 원할 시 항상 회전은 None = 0 으로)
- 자동 모드 일 경우 목표 이동 거리 값 없다면 기본 라인트레이싱


*** PICO 코드 테스트 방법 ***

- .uf2 파일을 업로드할 pico에 boostel 버튼을 누른 상태에서 USB 연결

- PICO 저장 장치 창이 나타나면 해당 창에 .uf2 파일을 복붙(.uf2 파일 옮기기)

- PICO에 전원 공급

- PICO와 UART 통신 중인 장비에서 PICO에게 0xFF를 지속적으로 전달해야 엔코더 값과 ADC 값을 받을 수 있음

- 0x00을 받을 경우 엔코더 값 0으로 만듦

*** 라이다 테스트 방법 ***

- LPB40B 라이다를 USB to TTL에 연결
(선 연결 - 흑 = GND, 백 = RXD, 녹 = TXD, 적 = 5V)

- 테스트하고자 하는 SBC에 라이다가 연결된 USB to TTL을 연결

- baud.py에 PORT 변수에 연결된 포트 번호로 수정

- python3 baud.py 실행

*** line을 구분 테스트 방법 ***

- linetracer.py 코드만 그대로 사용하면 됨

*** 250924 추가 수정 ***

- 수정된 코드는 10. AGV 개정안 확인

- Manual 모드에서 목표 이동 거리 값 추가

- Auto 모드에서 후진 이동 추가 (Manual 모드 또한 목표 이동 거리 값이 있을 시 후진 포함)

- 모터 상태 정보 추가 (엔코더 값 확인)

- 배터리 상태 정보 추가 (모터 상태 변화 할 때만 전달)

*** 250929 추가 수정 ***

- 모터 드라이버 알람 상태 값 읽기 추가

