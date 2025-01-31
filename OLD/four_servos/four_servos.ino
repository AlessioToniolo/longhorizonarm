#include <Arduino.h>
#include <FABRIK2D.h>
#include "ServoEasing.hpp"
#include <math.h>

// Following Single Responsibility Principle - each servo controls one joint
ServoEasing Servo1;
ServoEasing Servo2;
ServoEasing Servo3;

const uint16_t SPEED = 20; // degrees per second
const float BASE_HEIGHT = 145.1;
int lengths[] = {187, 162}; // Only two members now

// Create FABRIK2D solver for two joints plus end effector
Fabrik2D fabrik2D(3, lengths);

void setup() {
    Serial.begin(115200);
    
    // Initialize servos with default position
    Servo1.attach(13, 90);
    Servo2.attach(12, 90);
    Servo3.attach(14, 90);

    delay(500);
    moveServos(90, 90, 90); // Home position
    delay(3000);
}

void moveServos(int pos1, int pos2, int pos3) {
    Servo1.setEaseTo(pos1);
    Servo2.setEaseTo(pos2);
    Servo3.setEaseTo(pos3);
    
    setSpeedForAllServos(SPEED);
    synchronizeAllServosAndStartInterrupt();
    
    while (ServoEasing::areInterruptsActive()) {
        delay(5);
    }
}

bool moveToPosition(float x, float y, float z) {
    float baseAngle = calculateBaseAngle(x, y);
    float servoBase = constrain(baseAngle, 0, 180);

    float r, adjusted_z;
    transform3Dto2D(x, y, z, r, adjusted_z);
    
    if (!fabrik2D.solve(r, adjusted_z, lengths)) {
        Serial.println("Position unreachable");
        return false;
    }
    
    float shoulderAngle = fabrik2D.getAngle(0) * RAD_TO_DEG;
    float elbowAngle = fabrik2D.getAngle(1) * RAD_TO_DEG;
    
    float servoShoulder = constrain(90 - shoulderAngle, 0, 180);
    float servoElbow = constrain(elbowAngle + 90, 0, 180);
  
    moveServos(servoBase, servoShoulder, servoElbow);
    return true;
}

float calculateBaseAngle(float x, float y) {
    return fmod((atan2(y, x) * RAD_TO_DEG + 360), 360);
}

void transform3Dto2D(float x, float y, float z, float& r, float& new_z) {
    r = sqrt(x*x + y*y);
    new_z = z - BASE_HEIGHT;
}

void loop() {
  moveToPosition(100, 0, 200);
}