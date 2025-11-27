#include "samd51j20a.h"
#include <stdint.h>
#include "gpio.h"
#include "i2c_slave.h"
#include "../board.h"

// SDA = PB12, SCL = PB13
#define I2C_SLAVE_SERCOM SERCOM4

static volatile i2c_slave_data_t recent_data = {0, 0};
static volatile uint8_t byte_count = 0;

// Initialize SERCOM4 as I2C slave
void i2c_slave_init(uint8_t slave_address) {
    // Configure pins as input with pull-up (open-drain I2C)
    gpio_init_pin(I2C_SLAVE_SDA, GPIO_DIR_IN, GPIO_ALTERNATE_C_SERCOM);

    gpio_init_pin(I2C_SLAVE_SCL, GPIO_DIR_IN, GPIO_ALTERNATE_C_SERCOM);

    // Enable APB clock for SERCOM4
    MCLK->APBDMASK.bit.SERCOM4_ = 1;

    // Enable GCLK for SERCOM4
    GCLK->PCHCTRL[SERCOM4_GCLK_ID_CORE].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;
    while (!GCLK->PCHCTRL[SERCOM4_GCLK_ID_CORE].bit.CHEN);

    // Software reset
    I2C_SLAVE_SERCOM->I2CS.CTRLA.bit.SWRST = 1;
    while (I2C_SLAVE_SERCOM->I2CS.SYNCBUSY.bit.SWRST);

    // Configure as I2C slave
    I2C_SLAVE_SERCOM->I2CS.CTRLA.bit.MODE = 0x4; // I2C slave

    // Set slave address
    I2C_SLAVE_SERCOM->I2CS.ADDR.bit.ADDR = slave_address;
    I2C_SLAVE_SERCOM->I2CS.ADDR.bit.ADDRMASK = 0;

    // Enable smart mode (auto ACK)
    I2C_SLAVE_SERCOM->I2CS.CTRLB.bit.SMEN = 1;

    // Enable interrupts
    I2C_SLAVE_SERCOM->I2CS.INTENSET.bit.AMATCH = 1; // Address match
    I2C_SLAVE_SERCOM->I2CS.INTENSET.bit.DRDY = 1;   // Data ready
    I2C_SLAVE_SERCOM->I2CS.INTENSET.bit.PREC = 1;   // Stop condition

    // Enable peripheral
    I2C_SLAVE_SERCOM->I2CS.CTRLA.bit.ENABLE = 1;
    while (I2C_SLAVE_SERCOM->I2CS.SYNCBUSY.bit.ENABLE);

    // Enable NVIC interrupts
    NVIC_EnableIRQ(SERCOM4_0_IRQn);
    NVIC_EnableIRQ(SERCOM4_1_IRQn);
    NVIC_EnableIRQ(SERCOM4_3_IRQn);
}

// Return last received 2-byte data
i2c_slave_data_t i2c_slave_get_recent_data(void) {
    return recent_data;
}

// ------------------- Interrupt Handlers -------------------

// Address match interrupt
void SERCOM4_0_Handler(void) {
    if (I2C_SLAVE_SERCOM->I2CS.INTFLAG.bit.AMATCH) {
        byte_count = 0;

        // If master wants to read, CMD can be handled here (optional)
        I2C_SLAVE_SERCOM->I2CS.CTRLB.bit.CMD = 0x3; // ACK + continue

        // Clear interrupt
        I2C_SLAVE_SERCOM->I2CS.INTFLAG.bit.AMATCH = 1;
    }
}

// Data ready interrupt
void SERCOM4_1_Handler(void) {
    if (I2C_SLAVE_SERCOM->I2CS.INTFLAG.bit.DRDY) {
        if (!I2C_SLAVE_SERCOM->I2CS.STATUS.bit.DIR) { // Master write
            uint8_t data = I2C_SLAVE_SERCOM->I2CS.DATA.reg; // Must read first

            if (byte_count == 0) recent_data.address_byte = data;
            else if (byte_count == 1) recent_data.data_byte = data;

            byte_count++;

            // ACK + continue
            I2C_SLAVE_SERCOM->I2CS.CTRLB.bit.CMD = 0x3;
        }
        // Clear DRDY flag
        I2C_SLAVE_SERCOM->I2CS.INTFLAG.bit.DRDY = 1;
    }
}

// Stop condition interrupt
void SERCOM4_3_Handler(void) {
    if (I2C_SLAVE_SERCOM->I2CS.INTFLAG.bit.PREC) {
        I2C_SLAVE_SERCOM->I2CS.INTFLAG.bit.PREC = 1; // clear flag
    }
}
