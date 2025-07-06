#include "spi_collector.h"
#include "../periphs/spi.h"
#include "../periphs/uart.h"
#include "../periphs/spi_async.h"
#include "../drivers/delay.h"
#include "core_cm4.h" 

#define COLLECTOR_ENCODER_A_INDEX 0
#define COLLECTOR_ENCODER_B_INDEX 1
#define COLLECTOR_TENSION_A_INDEX 2
#define COLLECTOR_TENSION_B_INDEX 3

static uint16_t current_index = 0;

static const SPIConfig* configs[4] = {
    &SPI_CONF_MTR_ENCODER_A,
    &SPI_CONF_MTR_ENCODER_B,
    &SPI_CONF_TENSION_ARM_A,
    &SPI_CONF_TENSION_ARM_B,
};

static uint16_t clean_bits[4] = {0, 0, 0, 0};
static uint16_t dirty_bits[4] = {0, 0, 0, 0};

static uint8_t tx_data[2] = {0, 0};
static uint8_t rx_data[2] = {0, 0};

uint16_t spi_collector_get_encoder_a() {
    return clean_bits[COLLECTOR_ENCODER_A_INDEX];
}

uint16_t spi_collector_get_encoder_b() {
    return clean_bits[COLLECTOR_ENCODER_B_INDEX];
}

uint16_t spi_collector_get_tension_a() {
    return clean_bits[COLLECTOR_TENSION_A_INDEX];
}

uint16_t spi_collector_get_tension_b() {
    return clean_bits[COLLECTOR_TENSION_B_INDEX];
}

void spi_collector_init() {
    spi_async_init(&SPI_CONF_TENSION_ARM_A); 
    spi_async_init(&SPI_CONF_TENSION_ARM_B); 
    spi_async_init(&SPI_CONF_MTR_ENCODER_B); 
    spi_async_init(&SPI_CONF_MTR_ENCODER_A); 
}

void spi_collector_callback() {
    //uart_print("Collector callback index="); 
    //uart_println_int(current_index);

    // Process rx data
    uint16_t msb = rx_data[0];
    uint16_t lsb = rx_data[1];
    uint16_t full = ((msb << 8) | lsb);
    dirty_bits[current_index] = full;
    //uart_println_int_base(full & 0x3FFF, 16);

    // Done processing all 4? Post to clean
    if (current_index == 3) {
        __disable_irq();
        clean_bits[0] = dirty_bits[0];
        clean_bits[1] = dirty_bits[1];
        clean_bits[2] = dirty_bits[2];
        clean_bits[3] = dirty_bits[3];
        __enable_irq();
        /*
        uart_print_int(clean_bits[0] & 0x3FFF);
        uart_put(' ');
        uart_print_int(clean_bits[1] & 0x3FFF);
        uart_put(' ');
        uart_print_int(clean_bits[2] >> 6);
        uart_put(' ');
        uart_print_int(clean_bits[3] >> 6);
        uart_put('\n');
        */
    } else {
        // Send the next index
        current_index++;
        if (current_index == 2) {
            spi_change_mode(&SPI_CONF_TENSION_ARM_A);
        }
        //uart_println("Queuing another callback");
        spi_async_start_transfer(configs[current_index], tx_data, rx_data, 2, spi_collector_callback);
    }
    //delay(0x1);
}

void spi_collector_start_service() {
    current_index = 0; 
    spi_change_mode(&SPI_CONF_MTR_ENCODER_A);
    //uart_println("Queuing another callback");
    spi_async_start_transfer(configs[current_index], tx_data, rx_data, 2, spi_collector_callback);
}

