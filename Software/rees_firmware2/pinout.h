#ifndef PINOUT_H
#define PINOUT_H

#include <Arduino.h>

// Display
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
#endif

// Rotary encoder
#define PIN_ENCODER_CLK 2
#define PIN_ENCODER_DT 3
#define PIN_ENCODER_SW 9

// Stepper driver
#define PIN_STEPPER_ENABLE 8
#define PIN_STEPPER_STEP 6
#define PIN_STEPPER_DIRECTION 7

// Buzzer
#define PIN_BUZZ 11

// Sensor hall
#define PIN_ENDSTOP 21

// Electrovalve
#define PIN_VALVE 22

// Relay
#define PIN_RELAY 25

// BME280 SPI al menos para Arduino Nano o Mega 128
#if 0
#define PIN_BME_SCK  13
#define PIN_BME_MISO 12
#define PIN_BME_MOSI 11
#define PIN_BME_CS1  10 // sensor de presion 1
#define PIN_BME_CS2  4  // sensor de presion 2
#endif

// BME280 SPI para Arduino Mega 256
#define PIN_BME_SCK  52
#define PIN_BME_MISO 50
#define PIN_BME_MOSI 51
#define PIN_BME_CS1  53 // sensor de presion 1
#define PIN_BME_CS2  49 // sensor de presion 2

#endif // ENCODER_H
