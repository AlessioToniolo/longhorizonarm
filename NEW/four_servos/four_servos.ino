#include <Arduino.h>
#include "ServoEasing.hpp"

// Create four servo objects using ServoEasing
ServoEasing Servo1;
ServoEasing Servo2;
ServoEasing Servo3;
ServoEasing Servo4;

// Define servo pins - adjust for your Arduino
#define SERVO1_PIN 9
#define SERVO2_PIN 10
#define SERVO3_PIN 11
#define SERVO4_PIN 12

// Starting position for all servos
#define START_POSITION 0

void setup() {
    Serial.begin(115200);
    
    // Attach all servos and set to start position
    Servo1.attach(SERVO1_PIN, START_POSITION);
    Servo2.attach(SERVO2_PIN, START_POSITION);
    Servo3.attach(SERVO3_PIN, START_POSITION);
    Servo4.attach(SERVO4_PIN, START_POSITION);

    // Wait for servos to reach start position
    delay(500);
}

// Move all servos to specified positions synchronously
void moveServos(int pos1, int pos2, int pos3, int pos4, uint16_t speed) {
    // Set target positions
    Servo1.setEaseTo(pos1);
    Servo2.setEaseTo(pos2);
    Servo3.setEaseTo(pos3);
    Servo4.setEaseTo(pos4);
    
    // Start synchronized movement
    setSpeedForAllServos(speed);
    synchronizeAllServosAndStartInterrupt();
    
    // Wait for movement to complete
    while (ServoEasing::areInterruptsActive()) {
        delay(5);
    }
}

void loop() {
    uint16_t speed = 20; // degrees per second
    
    // Move to first position (different angles)
    moveServos(90, 120, 150, 180, speed);
    delay(1000);
    
    // Move to second position (all same angle)
    moveServos(180, 180, 180, 180, speed);
    delay(1000);
    
    // Return to start position
    moveServos(0, 0, 0, 0, speed);
    delay(1000);
}