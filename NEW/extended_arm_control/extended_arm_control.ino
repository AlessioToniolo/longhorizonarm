#include <FABRIK2D.h>
#include <ServoEasing.hpp>
#include <ESP32Servo.h>
#include <math.h>

// Arm segment lengths in mm (converted to integers for library compatibility)
const float BASE_HEIGHT = 145.1;
int lengths[] = {187, 162, 150}; // Remove 'const' from here

// Create FABRIK2D solver for the arm segments (4 joints including end effector)
Fabrik2D fabrik2D(4, lengths);

// Servos with corrected naming
ServoEasing base;
ServoEasing shoulder;
ServoEasing elbow;
ServoEasing wrist;

// Home position angles
const int HOME_POSITION = 90;
const int STARTUP_DELAY = 5000; // 5 second delay
const uint16_t HOME_SPEED = 10;  // Slower speed for home position
const uint16_t MOVE_SPEED = 10;  // Gentler movement speed

void setSpeedForAllServos(uint16_t speed) {
    for (int i = 0; i < 4; i++) {
        ServoEasing::ServoEasingArray[i]->setSpeed(speed);
    }
}

void moveToHome() {
    setSpeedForAllServos(HOME_SPEED);
    
    // Move all servos to 90 degrees
    base.setEaseTo(HOME_POSITION);
    shoulder.setEaseTo(HOME_POSITION);
    elbow.setEaseTo(HOME_POSITION);
    wrist.setEaseTo(HOME_POSITION);
    
    // Start synchronized movement
    synchronizeAllServosAndStartInterrupt(true);
    
    // Wait for movement to complete
    while (ServoEasing::areInterruptsActive()) {
        delay(10);
    }
}

float calculateBaseAngle(float x, float y) {
    return atan2(y, x) * RAD_TO_DEG;
}

void transform3Dto2D(float x, float y, float z, float& r, float& new_z) {
    r = sqrt(x*x + y*y);
    new_z = z - BASE_HEIGHT;
}

bool moveToPosition(float x, float y, float z, uint16_t speed = MOVE_SPEED) {
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
    
    // Set movement speed
    setSpeedForAllServos(speed);
    
    // Assign angles to correct servos
    base.setEaseTo(servoBase);
    shoulder.setEaseTo(servoShoulder);
    elbow.setEaseTo(servoElbow);
    wrist.setEaseTo(servoWrist);
    
    // Start synchronized movement
    synchronizeAllServosAndStartInterrupt(true);
    
    return true;
}

void setup() {
    Serial.begin(115200);

    // Attach servos with proper naming
    base.attach(13, 90);
    shoulder.attach(12, 90);
    elbow.attach(14, 90);
    wrist.attach(27, 90);

    // Set servo easing
    for (int i = 0; i < 4; i++) {
        ServoEasing::ServoEasingArray[i]->setEasingType(EASE_CUBIC_IN_OUT);
    }

    // Set FABRIK tolerance
    fabrik2D.setTolerance(0.5);

    // Move to home position
    moveToHome();
    
    // Wait for the specified startup delay
    delay(STARTUP_DELAY);
/*
    elbow.detach();
    delay(1000);
    elbow.attach(14, 90);

    delay(1000);
    wrist.detach();
    delay(1000);
    wrist.attach(27, 90);

      

    delay(STARTUP_DELAY);
    */
}

void loop() {
    // Test movement with a less aggressive position
    /*moveToPosition(150, 0, 120); // More conservative test position
    while (ServoEasing::areInterruptsActive()) { 
        delay(10); // Non-blocking wait with some delay to prevent tight loop
    }
    */
    /*
    delay(1000);
    base.detach();
    delay(1000);
    base.attach(13, 90);

    delay(2000);

*/

/*
    base.setEaseTo(HOME_POSITION);
    shoulder.setEaseTo(HOME_POSITION);
    elbow.setEaseTo(HOME_POSITION);
    wrist.setEaseTo(HOME_POSITION);

    delay(2000);  // Wait 2 seconds before next movement

*/
delay(5\000);
moveToPosition(150, 0, 200); // More conservative test position
    while (ServoEasing::areInterruptsActive()) { 
        delay(10); // Non-blocking wait with some delay to prevent tight loop
    }

    
}