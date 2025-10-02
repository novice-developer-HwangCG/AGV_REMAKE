<--- pico ↔ W5500 테스트--->

1. 간단 핑 테스트는 아래 사이트 내용대로 따라하면 됨
https://wiznxt.tistory.com/863?category=508895

핀 연결 (pico SPI 통신 핀 기준)
SCK = GP2
TX = G3
RX = GP4
CSn = GP5
- 주의점 SPI 버스를 맞춰야 함
I2C 0이라면 gp 0, gp 1, gp 2, gp 3, gp 4, gp 5, gp 6, gp 7 + gp 16, gp 17, gp18, gp 19
I2C 1이라면 gp 8, gp 9, gp 10, gp 11, gp 12, gp 13, gp 14, gp 15


w5500에 연결된 랜선을 연결하고 싶은 장비에 연결
장비에 이더넷 설정을 pico에서 설정한 코드에 맞추면 끝

중요 pico lib 폴더안에 아래 세가지는 무조건 넣어놔야 함
- adafruit_wiznet5k
- adafruit_bus_device
- adafruit_requests.mpy

라이브러리 업데이트 되면서 혹시나 모를 최신 버전으로 업데이트 해야 할 수 도 있음 주의


2. C SDK 테스트 (시도 안해봄) 해당 사이트 내용 따라 하면 될 것 같음
https://github.com/WIZnet-ioNIC/WIZnet-PICO-C

cd ~/projects
git clone --recurse-submodules https://github.com/WIZnet-ioNIC/WIZnet-PICO-C.git -> 서브 모듈과 함께 나머지까지 함께 설치

cd WIZnet-PICO-C
mkdir build && cd build
cmake ..
make -j

C 예제 코드
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pico_wiznet5k/wiznet5k.h"  // 경로는 프로젝트 구조에 맞게 조정

// SPI0 사용: GP18=SCK, GP19=MOSI, GP16=MISO, CS=GP17
#define PIN_SPI_SCK  18
#define PIN_SPI_MOSI 19
#define PIN_SPI_MISO 16
#define PIN_SPI_CS   17
#define PIN_W5500_RST 15

int main() {
    stdio_init_all();
    spi_init(spi0, 2 * 1000 * 1000);  // 2 MHz
    gpio_set_function(PIN_SPI_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MISO, GPIO_FUNC_SPI);
    gpio_init(PIN_SPI_CS);
    gpio_set_dir(PIN_SPI_CS, GPIO_OUT);
    gpio_put(PIN_SPI_CS, 1);

    // 리셋 핀
    gpio_init(PIN_W5500_RST);
    gpio_set_dir(PIN_W5500_RST, GPIO_OUT);
    gpio_put(PIN_W5500_RST, 0);
    sleep_ms(100);
    gpio_put(PIN_W5500_RST, 1);
    sleep_ms(100);

    // W5500 초기화
    wiznet5k_init(spi0, PIN_SPI_CS);
    wiznet5k_dhcp_start();

    // IP 할당 대기
    while (!wiznet5k_is_link_up()) {
        printf("Waiting for link...\n");
        sleep_ms(500);
    }
    printf("IP: %s\n", wiznet5k_get_ip_string());

    // 메인 루프
    while (true) {
        // 예: 링크 상태 LED 토글 등
        sleep_ms(1000);
    }
    return 0;
}
