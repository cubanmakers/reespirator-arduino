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
  LiquidCrystal lcd = LiquidCrystal(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
#endif
};

#endif // PANTALLA_H
