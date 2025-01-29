#include <FABRIK2D.h>
#include <ServoEasing.hpp>
#include <ESP32Servo.h>
#include <math.h>

// Arm segment lengths in mm
const float BASE_HEIGHT = 145.1;
const float lengths[] = {186.9, 162.0, 150.0}; // shoulder, elbow, wrist lengths

// Create FABRIK2D solver for the arm segments (excluding base rotation)
Fabrik2D fabrik2D(4, lengths); // 4 joints including end effector

// Servos
ServoEasing base;
ServoEasing shoulder1;
ServoEasing shoulder2;
ServoEasing wrist;

void setup() {
    Serial.begin(115200);

    // Attach servos
    base.attach(13, 90);
    shoulder1.attach(12, 90);
    shoulder2.attach(14, 90);
    wrist.attach(26, 90);

    // Set servo easing
    for (int i = 0; i < 4; i++) {
        ServoEasing::ServoEasingArray[i]->setEasingType(EASE_CUBIC_IN_OUT);
    }

    // Set FABRIK tolerance
    fabrik2D.setTolerance(0.5);
}

// Calculate base rotation angle from X,Y coordinates
float calculateBaseAngle(float x, float y) {
    return atan2(y, x) * RAD_TO_DEG;
}

// Transform 3D point to 2D plane for FABRIK solving
void transform3Dto2D(float x, float y, float z, float& r, float& new_z) {
    r = sqrt(x*x + y*y);      // Distance from base in XY plane
    new_z = z - BASE_HEIGHT;   // Adjust Z for base height
}

// Move arm to specified XYZ position
bool moveToPosition(float x, float y, float z, uint16_t speed = 30) {
    // Calculate base rotation
    float baseAngle = calculateBaseAngle(x, y);
    
    // Transform to 2D problem
    float r, adjusted_z;
    transform3Dto2D(x, y, z, r, adjusted_z);
    
    // Solve inverse kinematics in 2D
    if (!fabrik2D.solve(r, adjusted_z, lengths)) {
        Serial.println("Position unreachable");
        return false;
    }
    
    // Get joint angles
    float shoulderAngle = fabrik2D.getAngle(0) * RAD_TO_DEG;
    float elbowAngle = fabrik2D.getAngle(1) * RAD_TO_DEG;
    float wristAngle = fabrik2D.getAngle(2) * RAD_TO_DEG;
    
    // Adjust angles for servo orientation
    float servoBase = constrain(baseAngle, 0, 180);
    float servoShoulder = constrain(90 - shoulderAngle, 0, 180);
    float servoElbow = constrain(elbowAngle + 90, 0, 180);
    float servoWrist = constrain(wristAngle + 90, 0, 180);
    
    // Set movement speed
    setSpeedForAllServos(speed);
    
    // Move all servos
    ServoEasing::ServoEasingArray[0]->setEaseTo(servoBase);
    ServoEasing::ServoEasingArray[1]->setEaseTo(servoShoulder);
    ServoEasing::ServoEasingArray[2]->setEaseTo(servoElbow);
    ServoEasing::ServoEasingArray[3]->setEaseTo(servoWrist);
    
    // Start synchronized movement
    synchronizeAllServosAndStartInterrupt(true);
    
    return true;
}

void loop() {
    // Example movement sequence
    moveToPosition(200, 0, 100);  // Move to point in front
    
    // Wait for movement to complete
    while (ServoEasing::areInterruptsActive()) {
        // Can do other things here while servos are moving
    }
    delay(1000);
    
    moveToPosition(150, 150, 50);  // Move to point to the right
    while (ServoEasing::areInterruptsActive()) {}
    delay(1000);
}