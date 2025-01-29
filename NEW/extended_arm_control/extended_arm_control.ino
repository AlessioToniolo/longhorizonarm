#include <ServoEasing.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>

ServoEasing* ServoArray[4];
ServoEasing base;
ServoEasing shoulder1;
ServoEasing shoulder2;
ServoEasing wrist;


void setup() {
    Serial.begin(115200); // baudrate

    // Attach servos
    base.attach(13, 90);
    shoulder1.attach(12, 90);
    shoulder2.attach(14, 90);
    wrist.attach(26, 90);

    // Initialize servo array
    ServoArray[0] = &base;
    ServoArray[1] = &shoulder1;
    ServoArray[2] = &shoulder2;
    ServoArray[3] = &wrist;

    // Set easing type for all servos
    for (int i = 0; i < NUMBER_OF_SERVOS; i++) {
        ServoArray[i]->setEasingType(EASE_CUBIC_IN_OUT);
    }
}

void loop() {
    // Example synchronized movement
    setEaseToForAllServosSynchronizeAndWaitForAllServosToStop(); // 2000ms movement duration
    
    // Add your desired movement patterns here using the array
    // Example: synchronously move all servos to different positions
    ServoArray[0]->setEaseTo(120); // base
    ServoArray[1]->setEaseTo(45);  // shoulder1
    ServoArray[2]->setEaseTo(90);  // shoulder2
    ServoArray[3]->setEaseTo(60);  // wrist
    
    synchronizeAllServosStartAndWaitForAllServosToStop();
    
    delay(1000); // Wait before next movement
}