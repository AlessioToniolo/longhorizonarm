#include <Arduino.h>
#include <FABRIK2D.h>
#include "ServoEasing.hpp"
#include <math.h>

ServoEasing Servo1;
ServoEasing Servo2;
ServoEasing Servo3;
ServoEasing Servo4;

uint16_t speed = 20; // degrees per second

void setup() {
    Serial.begin(115200);
    
    Servo1.attach(13, 90);
    Servo2.attach(12, 90);
    Servo3.attach(14, 90);
    Servo4.attach(27, 90);

    delay(500);

    // home
    moveServos(90, 90, 90, 90);

    millis(3000);
}

// Move all servos to specified positions synchronously
void moveServos(int pos1, int pos2, int pos3, int pos4) {
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
    delay(1000);
    moveToPosition(100, 0, 175);
    delay(1000);
}




// IK STUFFFF
const float BASE_HEIGHT = 145.1;
int lengths[] = {187, 162, 86}; // Remove 'const' from here

// Create FABRIK2D solver for the arm segments (4 joints including end effector)
Fabrik2D fabrik2D(4, lengths);

bool moveToPosition(float x, float y, float z) {
    // Calculate base rotation with proper angle wrapping
    float baseAngle = calculateBaseAngle(x, y);
    baseAngle = fmod(baseAngle + 360, 360); // Wrap to 0-360
    float servoBase = constrain(baseAngle, 0, 180);

    // Transform to 2D problem
    float r, adjusted_z;
    transform3Dto2D(x, y, z, r, adjusted_z);
    
    // Solve inverse kinematics using library-compatible integer lengths
    if (!fabrik2D.solve(r, adjusted_z, lengths)) {
        Serial.println("Position unreachable");
        return false;
    }
    
    // Get joint angles with proper conversion
    float shoulderAngle = fabrik2D.getAngle(0) * RAD_TO_DEG;
    float elbowAngle = fabrik2D.getAngle(1) * RAD_TO_DEG;
    float wristAngle = fabrik2D.getAngle(2) * RAD_TO_DEG;
    
    // Convert angles to servo positions
    float servoShoulder = constrain(90 - shoulderAngle, 0, 180);
    float servoElbow = constrain(elbowAngle + 90, 0, 180);
    float servoWrist = constrain(wristAngle + 90, 0, 180);
  
    moveServos(servoBase, servoShoulder, servoElbow, servoWrist);
}

float calculateBaseAngle(float x, float y) {
    return atan2(y, x) * RAD_TO_DEG;
}

void transform3Dto2D(float x, float y, float z, float& r, float& new_z) {
    r = sqrt(x*x + y*y);
    new_z = z - BASE_HEIGHT;
}