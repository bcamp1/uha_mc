// UHA Motherboard Scheduling Priorities and Frequencies
#pragma once

// Priorities
#define PRIO_ROLLER_ENCODER (0)
#define PRIO_STATE_MACHINE_TICK (1)
#define PRIO_SLAVE_I2C (2)

// Frequencies
#define FREQUENCY_STATE_MACHINE_TICK (1000.0f)

// Timer IDs
#define ID_STATE_MACHINE_TICK (0)

