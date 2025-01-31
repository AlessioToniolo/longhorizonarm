#include "ServoEasing.hpp"
#include <ESP32Servo.h>

ServoEasing base;

void setup() {
    base.attach(13, 90);
}

void loop() {
    base.setEasingType(EASE_CUBIC_IN_OUT);
    base.easeTo(175, 30);
    delay(2000);
}