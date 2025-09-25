#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/adc.h"
#include "hardware/irq.h"

#define LED             PICO_DEFAULT_LED_PIN

#define L_ENCODER_A     4
#define L_ENCODER_B     5
#define R_ENCODER_A     7
#define R_ENCODER_B     8

#define PWR_CHECK_LED   2

#define ADC_PIN         28

#define UART_TX_PIN_NUMBER              (1)
#define UART_RX_PIN_NUMBER              (0)

#define UART_ID                         (uart0)
#define BAUD_RATE                       (115200)
#define DATA_BITS                       (8)
#define STOP_BITS                       (1)
#define PARITY                          (UART_PARITY_NONE)

#define UART_START_FLAG                 (0xff)

void UART_Rx_Handler(void);
void ConfigureUart(void);
void ConfigureGpio(void);
void Configure_adc(void);
uint16_t read_adc_value(void);
void tx_uart_data(int l_encoder, int r_encoder, float voltage);
void StartSignal(void);
void encoder_irq_handler(uint gpio, uint32_t events);

volatile static uint        Receive_Done       = 0;
volatile static uint8_t     Receive_Char       = 0;
volatile static uint        Frame_Count        = 0;

static char buffer[64];

volatile int L_encoderPos = 0;
volatile int R_encoderPos = 0;

int main()
{
    stdio_init_all();
    sleep_ms(10);

    ConfigureUart();
    sleep_ms(10);

    ConfigureGpio();
    sleep_ms(10);

    Configure_adc();
    sleep_ms(10);
    // StartSignal();
    // sleep_ms(10);

    gpio_put(LED, 1);
    gpio_put(PWR_CHECK_LED, 1);

    while (true)
    {
        tight_loop_contents();

        if (Receive_Done > 0)
        {
            const float conversion_factor = 3.3f / (1 << 12);
            uint16_t raw = read_adc_value();
            float voltage = raw * conversion_factor;

            tx_uart_data(L_encoderPos, R_encoderPos, voltage);
            Receive_Done = 0;
        }

        Frame_Count++;
        busy_wait_ms(1);
    }
    return 0;
}

void UART_Rx_Handler(void)
{
    irq_set_enabled(UART0_IRQ, false);
    if (uart_is_readable_within_us(UART_ID, 100))
    {
        Receive_Char = uart_getc(UART_ID);
    }

    if (Receive_Char == UART_START_FLAG) {
        Receive_Done = 1;
    }
    
    if (Receive_Char == 0x00){
        L_encoderPos = 0;
        R_encoderPos = 0;
    }

    irq_set_enabled(UART0_IRQ, true);
}

void ConfigureUart(void)
{
    gpio_set_function(UART_TX_PIN_NUMBER, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN_NUMBER, GPIO_FUNC_UART);

    uart_init(UART_ID, BAUD_RATE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID, false);

    irq_set_exclusive_handler(UART0_IRQ, UART_Rx_Handler);
    irq_set_enabled(UART0_IRQ, true);

    uart_set_irq_enables(UART_ID, true, false);
}

void ConfigureGpio(void)
{
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);

    gpio_init(PWR_CHECK_LED);
    gpio_set_dir(PWR_CHECK_LED, GPIO_OUT);

    gpio_init(L_ENCODER_A);
    gpio_set_dir(L_ENCODER_A, GPIO_IN);
    gpio_set_irq_enabled_with_callback(L_ENCODER_A, GPIO_IRQ_EDGE_RISE, true, &encoder_irq_handler);
    // gpio_set_irq_enabled_with_callback(L_ENCODER_A, GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL, true, &encoder_irq_handler);

    gpio_init(L_ENCODER_B);
    gpio_set_dir(L_ENCODER_B, GPIO_IN);

    gpio_init(R_ENCODER_A);
    gpio_set_dir(R_ENCODER_A, GPIO_IN);
    gpio_set_irq_enabled(R_ENCODER_A, GPIO_IRQ_EDGE_RISE, true);
    // gpio_set_irq_enabled(R_ENCODER_A, GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL, true);

    gpio_init(R_ENCODER_B);
    gpio_set_dir(R_ENCODER_B, GPIO_IN);
}

void Configure_adc() {
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(2); 
}

uint16_t read_adc_value() {
    return adc_read();
}

void StartSignal(void)
{
    gpio_put(LED, 1);
    sleep_ms(3000);
    gpio_put(LED, 0);
}

void encoder_irq_handler(uint gpio, uint32_t events)
{
    if (gpio == L_ENCODER_A)
        L_encoderPos += (gpio_get(L_ENCODER_A) == gpio_get(L_ENCODER_B) ? 1 : -1);
    else if (gpio == R_ENCODER_A)
        R_encoderPos += (gpio_get(R_ENCODER_A) == gpio_get(R_ENCODER_B) ? -1 : 1);
}

void tx_uart_data(int l_encoder, int r_encoder, float voltage)
{
    sprintf(buffer, "%d,%d,%.2f\n", l_encoder, r_encoder, voltage);
    uart_puts(UART_ID, buffer);
}
