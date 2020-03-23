#ifndef PINOUT_H
#define PINOUT_H

#include "Arduino.h"

// Display

#define LCD_ROWS 2
#define LCD_COLS 16

#ifdef I2C  //i2c

//D irección de LCD I2C
#define I2C_DIR 0x3F

#else

// pines Display parallel  // parallel
#define LCD_RS A0
#define LCD_RW A1
#define LCD_E  A2
#define LCD_D4 A3
#define LCD_D5 A4
#define LCD_D6 A5
#define LCD_D7 A6

#endif // I2C

// Rotary encoder
#define ENCODER_CLK_PIN 2
#define ENCODER_DT_PIN 3
#define ENCODER_SW_PIN 9

// Stepper driver (FlexyStepper)
#define MOTOR_ENABLE_PIN 8
#define MOTOR_STEP_PIN 6
#define MOTOR_DIRECTION_PIN 7

// Buzzer
#define BUZZ_PIN 11

// Sensor hall
#define ENDSTOP_PIN 21

// BME280 SPI al menos para Arduino Nano o Mega 128
// #define BME_SCK  13
// #define BME_MISO 12
// #define BME_MOSI 11
// #define BME_CS1  10 // sensor de presion 1
// #define BME_CS2  4  // sensor de presion 2

// BME280 SPI para Arduino Mega 256
#define BME_SCK  52
#define BME_MISO 50
#define BME_MOSI 51
#define BME_CS1  53 // sensor de presion 1
#define BME_CS2  49 // sensor de presion 2

#endif // ENCODER_H
