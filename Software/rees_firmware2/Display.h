#ifndef PANTALLA_H
#define PANTALLA_H

#ifdef I2C
#include "src/LiquidCrystal_I2C/LiquidCrystal_I2C.h"
#else
#include "src/LiquidCrystal/LiquidCrystal.h"
#endif

#include "pinout.h"
#include "defaults.h"

class Display
{
public:
  Display();
  void writeLine(int line, String message = "", int offsetLeft = 0);
  void clear();

private:
#ifdef I2C
  LiquidCrystal_I2C lcd = LiquidCrystal_I2C(I2C_DIR, 20, 4);
#else
  LiquidCrystal lcd = LiquidCrystal(PIN_LCD_RS, PIN_LCD_E, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
#endif
};

#endif // PANTALLA_H
