// UHA Motherboard Scheduling Priorities and Frequencies
#pragma once

// Priorities (lower number = higher priority on Cortex-M)
#define PRIO_ROLLER_ENCODER (0)
#define PRIO_RS422 (1)
#define PRIO_RS485 (1)
#define PRIO_STATE_MACHINE_TICK (2)
#define PRIO_SLAVE_I2C (3)

// Frequencies
#define FREQUENCY_STATE_MACHINE_TICK (1000.0f)

// Timer IDs
#define ID_STATE_MACHINE_TICK (0)

