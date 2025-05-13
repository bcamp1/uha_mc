/*
 * ems22a.h
 *
 * Created: 3/20/2025 12:35:38 AM
 *  Author: brans
 */ 

#include "../periphs/spi.h"


#ifndef EMS22A_H_
#define EMS22A_H_

typedef struct {
	const SPIConfig* spi;
	uint16_t bottom_position;
	uint16_t top_position;
} TensionArmConfig;

extern const TensionArmConfig TENSION_ARM_A;
extern const TensionArmConfig TENSION_ARM_B;

float tension_arm_get_position(const TensionArmConfig* config);
void tension_arm_init(const TensionArmConfig* config);

#endif /* EMS22A_H_ */
