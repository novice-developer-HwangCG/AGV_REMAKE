해당 폴더안에 'W5500_EVB_PICO-20260406-v1.28.0.uf2' 파일을 pico에 업로드 (pico bootsel 버튼을 누른채 usb 연결)
파일 없다면 https://micropython.org/download/W5500_EVB_PICO/ 해당 링크 들어가서 Firmware 다운 받기

이후 thoony 개발툴로 pico 포트 연결

agv_rail_pico_260528.py 코드를 pico에 main.py로 저장

agv_rail_lidar_260528.py 코드를 pico에 lidar.py로 저장

* 주의점 rst핀 연결해야 함 pico 코드에는 rst 핀 8번으로 연결되어 있는데 배선 수정 시 참고할 것 (추천하는 핀은 우측 모터 핀 중 아무거나 또는 asdd용 6핀 옆에 빈 4pin에 pico에 사용하지 않는 핀 아무거나 연결)