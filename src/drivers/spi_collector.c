#include "spi_collector.h"
#include "../periphs/spi.h"
#include "../periphs/uart.h"
#include "../periphs/gpio.h"
#include "../periphs/spi_async.h"
#include "../control/controller.h"
#include "../drivers/delay.h"
#include "board.h"
#include "tension_arm.h"
#include "motor_unit.h"
#include "motor_encoder.h"
#include "core_cm4.h" 
#include "../foc/foc_math_fpu.h"
#include "../periphs/timer.h"
#include "uha_motor_driver.h"

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
static const float tension_a_top_position = 470;
static const float tension_a_bottom_position = 591;
static const float tension_b_top_position = 746;
static const float tension_b_bottom_position = 627;
static const float motor_poles = 4.0f;

// Motor torques
static volatile bool enable_motors = false;
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
    uha_motor_driver_init(&UHA_MTR_DRVR_CONF_A);
    uha_motor_driver_init(&UHA_MTR_DRVR_CONF_B);
    torque_a = 0.0f;
    torque_b = 0.0f;
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

    // Process Tension Arm Positions
	float interpolation_a = ((float) (tension_a_bottom_position - (dirty_bits[2] >> 6))) / ((float) (tension_a_top_position - tension_a_bottom_position));
	if (interpolation_a < 0) {
		interpolation_a = -interpolation_a;
	}
    tension_a_pos = interpolation_a;

	float interpolation_b = ((float) (tension_b_bottom_position - (dirty_bits[3] >> 6))) / ((float) (tension_b_top_position - tension_b_bottom_position));
	if (interpolation_b < 0) {
		interpolation_b = -interpolation_b;
	}

    tension_b_pos = interpolation_b;
}

static void process_foc() {
    motor_unit_set_torque(&MOTOR_UNIT_A, torque_a, encoder_a_pole_pos);
    motor_unit_set_torque(&MOTOR_UNIT_B, torque_b, encoder_b_pole_pos);
}

static void foc_off() {
    uha_motor_driver_set_high_z(&UHA_MTR_DRVR_CONF_A);
    uha_motor_driver_set_high_z(&UHA_MTR_DRVR_CONF_A);
    //motor_unit_set_torque(&MOTOR_UNIT_A, 0.0f, encoder_a_pole_pos);
    //motor_unit_set_torque(&MOTOR_UNIT_B, 0.0f, encoder_b_pole_pos);
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
        //__disable_irq();
        //gpio_set_pin(DEBUG_PIN);
        process_floats();
        controller_run_iteration();
        if (enable_motors) {
            process_foc();
        } else {
           foc_off(); 
        }
        //gpio_clear_pin(DEBUG_PIN);
        //__enable_irq();
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

static void spi_collector_service_entry() {
    current_index = 0; 
    spi_change_mode(&SPI_CONF_MTR_ENCODER_A);
    //uart_println("Queuing another callback");
    spi_async_start_transfer(configs[current_index], tx_data, rx_data, 2, spi_collector_callback);
}

void spi_collector_enable_service() {
    timer_schedule(TIMER_ID_SPI_COLLECTOR, TIMER_SAMPLE_RATE_SPI_COLLECTOR, TIMER_PRIORITY_SPI_COLLECTOR, spi_collector_service_entry);
}

void spi_collector_disable_service() {
    timer_deschedule(TIMER_ID_SPI_COLLECTOR);
}

void spi_collector_enable_motors() {
    torque_a = 0.0f;
    torque_b = 0.0f;
    enable_motors = true;
}

void spi_collector_disable_motors() {
    torque_a = 0.0f;
    torque_b = 0.0f;
    enable_motors = false;
}

void spi_collector_set_torque_a(float torque) {
    torque_a = torque;
}

void spi_collector_set_torque_b(float torque) {
    torque_b = torque;
}

