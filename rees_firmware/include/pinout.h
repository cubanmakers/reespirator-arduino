#ifndef PINOUT_H
#define PINOUT_H

// Stepper driver (FlexyStepper)
#define PIN_STEPPER_STEP 6
#define PIN_STEPPER_DIRECTION 7
#define PIN_EN 8

// Buzzer
#define PIN_BUZZ 11

// Stepper homing
#define PIN_ENDSTOP 2

// Solenoid pine
#define PIN_SOLENOID 39

// Display
/*
#ifdef I2C
#define I2C_DIR 0x3F
#else
#define PIN_LCD_RS A0
#define PIN_LCD_RW A1
#define PIN_LCD_E  A2
#define PIN_LCD_D4 A3
#define PIN_LCD_D5 A4
#define PIN_LCD_D6 A5
#define PIN_LCD_D7 A6
#endif*/

// BME280 SPI for Arduino Nano or Mega 128
// #define PIN_BME_SCK  13
// #define PIN_BME_MISO 12
// #define PIN_BME_MOSI 11
// #define PIN_BME_CS1  10 // sensor de presion 1
// #define PIN_BME_CS2  4  // sensor de presion 2

// BME280 SPI for Arduino Mega 256
#define PIN_BME_SCK  52
#define PIN_BME_MISO 50
#define PIN_BME_MOSI 51
#define PIN_BME_CS1  53 // sensor de presion 1
#define PIN_BME_CS2  49 // sensor de presion 2

#endif // ENCODER_H