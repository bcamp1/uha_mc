/*
 * i2c_slave.h
 *
 * Created: 11/26/2025
 *  Author: brans
 */

#ifndef I2C_SLAVE_H_
#define I2C_SLAVE_H_

#include <stdint.h>
#include <stdbool.h>

#define I2C_SLAVE_SERCOM	SERCOM4
#define I2C_SLAVE_SDA		PIN_PB12
#define I2C_SLAVE_SCL		PIN_PB13

typedef struct {
	uint8_t address_byte;
	uint8_t data_byte;
} i2c_slave_data_t;

void i2c_slave_init(uint8_t slave_address);
i2c_slave_data_t i2c_slave_get_recent_data(void);

#endif /* I2C_SLAVE_H_ */
