// UHA Motherboard Pin Configurations
#pragma once

// Debug Pins
#define PIN_DEBUG1 PIN_PB08
#define PIN_DEBUG2 PIN_PB07
#define DEBUG_PIN PIN_PA14

// SPI
#define PIN_MOSI PIN_PA17
#define PIN_MISO PIN_PA18
#define PIN_SCK PIN_PA16

// Roller Encoder Pins
#define PIN_ROLLER_INDEX PIN_PA09
#define PIN_ROLLER_PULSE PIN_PA07
#define PIN_ROLLER_DIR   PIN_PA06
#define INDEX_ROLLER_PULSE (7)
#define IRQ_ROLLER EIC_7_IRQn
#define HANDLER_ROLLER EIC_7_Handler

// Tension Arm Encoders
#define PIN_TENSION_TAKEUP_CS PIN_PA25
#define PIN_TENSION_SUPPLY_CS PIN_PA24

// Pinch Solenoid
#define PIN_PINCH_SOLENOID PIN_PB22

