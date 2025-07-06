#include "spi_collector.h"
#include "../periphs/spi.h"
#include "../periphs/uart.h"
#include "../periphs/gpio.h"
#include "../periphs/spi_async.h"
#include "../drivers/delay.h"
#include "board.h"
#include "tension_arm.h"
#include "motor_unit.h"
#include "motor_encoder.h"
#include "core_cm4.h" 
#include "../foc/foc_math_fpu.h"

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

static uint16_t dirty_bits[4] = {0, 0, 0, 0};

static uint8_t tx_data[2] = {0, 0};
static uint8_t rx_data[2] = {0, 0};

// Additional float processing
static volatile float encoder_a_pos = 0.0f;
static volatile float encoder_a_pole_pos = 0.0f;
static volatile float encoder_b_pos = 0.0f;
static volatile float encoder_b_pole_pos = 0.0f;
static volatile float tension_a_pos = 0.0f;
static volatile float tension_b_pos = 0.0f;

// Calibrations
static const float encoder_a_offset = 1.38105f;
static const float encoder_b_offset = 4.59148f;
static const float tension_a_top_position = 466;
static const float tension_a_bottom_position = 588;
static const float tension_b_top_position = 725;
static const float tension_b_bottom_position = 606;
static const float motor_poles = 4.0f;

// Motor torques
static volatile float torque_a = 0.0f;
static volatile float torque_b = 0.0f;

float spi_collector_get_encoder_a() {
    return encoder_a_pos;
}

float spi_collector_get_encoder_a_pole() {
    return encoder_a_pole_pos;
}

float spi_collector_get_encoder_b() {
    return encoder_b_pos;
}

float spi_collector_get_encoder_b_pole() {
    return encoder_b_pole_pos;
}

float spi_collector_get_tension_a() {
    return tension_a_pos;
}

float spi_collector_get_tension_b() {
    return tension_b_pos;
}

void spi_collector_init() {
    spi_async_init(&SPI_CONF_TENSION_ARM_A); 
    spi_async_init(&SPI_CONF_TENSION_ARM_B); 
    spi_async_init(&SPI_CONF_MTR_ENCODER_B); 
    spi_async_init(&SPI_CONF_MTR_ENCODER_A); 
}

static void process_floats() {
    encoder_a_pos = TWOPI * ((float) (dirty_bits[0]&0x3FFF) / ((float) 0x3FFF));
    encoder_b_pos = TWOPI * ((float) (dirty_bits[1]&0x3FFF) / ((float) 0x3FFF));

    // Process pole position a
	encoder_a_pole_pos = encoder_a_pos - encoder_a_offset;
	encoder_a_pole_pos *= motor_poles;
	while (encoder_a_pole_pos < 0) encoder_a_pole_pos += TWOPI;
	while (encoder_a_pole_pos > TWOPI) encoder_a_pole_pos -= TWOPI;

    // Process pole position b
	encoder_b_pole_pos = encoder_b_pos - encoder_b_offset;
	encoder_b_pole_pos *= motor_poles;
	while (encoder_b_pole_pos < 0) encoder_b_pole_pos += TWOPI;
	while (encoder_b_pole_pos > TWOPI) encoder_b_pole_pos -= TWOPI;

    tension_a_pos = (float) dirty_bits[2];
    tension_b_pos = (float) dirty_bits[3];
}

static void process_foc() {
    motor_unit_set_torque(&MOTOR_UNIT_A, torque_a, encoder_a_pole_pos);
    motor_unit_set_torque(&MOTOR_UNIT_B, torque_b, encoder_b_pole_pos);
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
        gpio_set_pin(DEBUG_PIN);
        process_floats();
        process_foc();
        gpio_clear_pin(DEBUG_PIN);
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

void spi_collector_set_torque_a(float torque) {
    torque_a = torque;
}

void spi_collector_set_torque_b(float torque) {
    torque_b = torque;
}

