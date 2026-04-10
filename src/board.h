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
#define PIN_ROLLER_DIR   PIN_PA08
#define INDEX_ROLLER_PULSE (7)
#define IRQ_ROLLER EIC_7_IRQn
#define HANDLER_ROLLER EIC_7_Handler

// Tension Arm Encoders
#define PIN_TENSION_A_CS PIN_PA25
#define PIN_TENSION_B_CS PIN_PA24

// Solenoids
#define PIN_PINCH_SOLENOID PIN_PB22

// ------------------------DEPRICATED AS OF REV E-----------------------
// BLDC Driver A Pins
#define PIN_BLDC_A_CS       PIN_PA21
#define PIN_BLDC_A_CONFIG1  PIN_PA24
#define PIN_BLDC_A_CONFIG2  PIN_PA23
#define PIN_BLDC_A_ENABLE   PIN_PA25
#define PIN_BLDC_A_FAULT    PIN_PA22

// BLDC Driver B Pins
#define PIN_BLDC_B_CS       PIN_PA12
#define PIN_BLDC_B_CONFIG1  PIN_PB17
#define PIN_BLDC_B_CONFIG2  PIN_PB16
#define PIN_BLDC_B_ENABLE   PIN_PA20
#define PIN_BLDC_B_FAULT    PIN_PA19

// BLDC Driver C Pins
#define PIN_BLDC_C_CS       PIN_PB11
#define PIN_BLDC_C_CONFIG1  PIN_PB13
#define PIN_BLDC_C_CONFIG2  PIN_PB14
#define PIN_BLDC_C_ENABLE   PIN_PB12
#define PIN_BLDC_C_FAULT    PIN_PB15

// Solenoid
#define PIN_LIFT_SOLENOID PIN_PB23
