/*
 * i2c.h
 *
 * Created: 8/17/2024 12:06:45 PM
 *  Author: brans
 */ 


#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include <stdbool.h>

#define I2C_SERCOM	SERCOM0
#define I2C_SDA		PIN_PA08
#define I2C_SCL		PIN_PA09

#define I2C_BUS_BUSY	(0b11)
#define I2C_BUS_IDLE	(0b01)
#define I2C_BUS_OWNER	(0b10)

#define I2C_CMD_RS			(0x1)
#define I2C_CMD_READBYTE	(0x2)
#define I2C_CMD_STOP		(0x3)
#define I2C_CMD_ACK			(0)
#define I2C_CMD_NACK		(1)

#define I2C_W	(0)
#define I2C_R	(1)

void i2c_init(void);

int i2c_wait_MB_timeout(uint32_t cycles);
void i2c_wait_busy(void);
void i2c_syncbusy(void);
void i2c_send_cmd(bool ack, uint8_t cmd);

int i2c_send_data(uint8_t data);
int i2c_send_addr_write(uint8_t addr);
int i2c_send_addr_read(uint8_t addr);



#endif /* I2C_H_ */