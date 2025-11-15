// UHA Motherboard Pin Configurations
#pragma once

// Debug Pins
#define PIN_DEBUG1 PIN_PA14
#define PIN_DEBUG2 PIN_PB15
#define DEBUG_PIN PIN_PA14

// UART
#define PIN_TX PIN_PA04 
#define PIN_RX PIN_PA05 

// SPI
#define PIN_MOSI PIN_PA17
#define PIN_MISO PIN_PA18
#define PIN_SCK PIN_PA16

// I2C
#define SLAVE_SDA PIN_PB12
#define SLAVE_SCL PIN_PB13
#define MOTOR_SDA PIN_PA12
#define MOTOR_SCL PIN_PA13

// BLDC Driver A Pins
#define PIN_BLDC_A_CS       PIN_PA25
#define PIN_BLDC_A_IDENT1   PIN_PA23
#define PIN_BLDC_A_IDENT0   PIN_PA20
#define PIN_BLDC_A_ENABLE   PIN_PA08

// BLDC Driver B Pins
#define PIN_BLDC_B_CS       PIN_PA24
#define PIN_BLDC_B_IDENT1   PIN_PA22
#define PIN_BLDC_B_IDENT0   PIN_PB17
#define PIN_BLDC_B_ENABLE   PIN_PA09

// BLDC Driver C Pins
#define PIN_BLDC_C_CS       PIN_PA19
#define PIN_BLDC_C_IDENT1   PIN_PA21
#define PIN_BLDC_C_IDENT0   PIN_PB16
#define PIN_BLDC_C_ENABLE   PIN_PA10

// Stepper Driver 1 Pins
#define PIN_STP1_DIR    PIN_PB06
#define PIN_STP1_STEP   PIN_PB07
#define PIN_STP1_ENABLE PIN_PB08

// Stepper Driver 2 Pins
#define PIN_STP2_DIR    PIN_PB09
#define PIN_STP2_STEP   PIN_PA06
#define PIN_STP2_ENABLE PIN_PA07

// Roller Encoder Pins
#define PIN_ROLLER_INDEX PIN_PB05
#define PIN_ROLLER_PULSE PIN_PB07
#define PIN_ROLLER_DIR   PIN_PB06

// Tension Arm Encoders
#define PIN_TENSION_A_CS PIN_PB30
#define PIN_TENSION_B_CS PIN_PB31

