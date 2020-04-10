#include "pinout.h"
#include "defaults.h"

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("hola");
}

void loop() {
    static unsigned long lastRead = millis();
    if (millis() > lastRead + 50) {
        ventilation_control.update();
                
        lastRead = millis();
    }
}