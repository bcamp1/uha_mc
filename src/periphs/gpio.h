#ifndef GPIO_H
#define GPIO_H

#include <stdbool.h>
#include <stdint.h>
#include <samd51j20a.h>

#define GPIO_DIR_IN			(false)
#define GPIO_DIR_OUT		(true)

#define GPIO_DRIVE_WEAK		(0)
#define GPIO_DRIVE_STRONG	(1)

#define GPIO_PORT_A			(0)
#define GPIO_PORT_B			(1)

#define GPIO_ALTERNATE_NONE				(8)
#define GPIO_ALTERNATE_A_EIC			(0)
#define GPIO_ALTERNATE_B_ADC			(1)
#define GPIO_ALTERNATE_C_SERCOM			(2)
#define GPIO_ALTERNATE_D_SERCOM_ALT		(3)
#define GPIO_ALTERNATE_E_TC				(4)
#define GPIO_ALTERNATE_F_TCC			(5)
#define GPIO_ALTERNATE_G_TCC_PDEC		(6)
#define GPIO_ALTERNATE_H_GCLK			(7)

//void gpio_init();
void gpio_init_pin(uint8_t pin, bool direction, uint8_t alternate_function);
void gpio_set_pin(uint8_t pin);
void gpio_clear_pin(uint8_t pin);
void gpio_toggle_pin(uint8_t pin);
bool gpio_get_pin(uint8_t pin);
void gpio_set_drivestrength(uint8_t pin, uint8_t drivestrength);

#endif
