#include <FABRIK2D.h>
#include <ServoEasing.hpp>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <math.h>

// Arm segment lengths in mm
const float BASE_HEIGHT = 145.1;
const float lengths[] = {186.9, 162.0, 150.0};

// Create FABRIK2D solver
Fabrik2D fabrik2D(4, lengths);

// Servos
ServoEasing base;
ServoEasing shoulder1;
ServoEasing shoulder2;
ServoEasing wrist;

void setup() {
    Serial.begin(9600);  // Match Python baud rate
    
    // Attach servos
    base.attach(13, 90);
    shoulder1.attach(12, 90);
    shoulder2.attach(14, 90);
    wrist.attach(26, 90);

    // Set servo easing
    for (int i = 0; i < 4; i++) {
        ServoEasing::ServoEasingArray[i]->setEasingType(EASE_CUBIC_IN_OUT);
    }

    fabrik2D.setTolerance(0.5);
}

// Previous helper functions remain unchanged
float calculateBaseAngle(float x, float y) {
    return atan2(y, x) * RAD_TO_DEG;
}

void transform3Dto2D(float x, float y, float z, float& r, float& new_z) {
    r = sqrt(x*x + y*y);
    new_z = z - BASE_HEIGHT;
}

bool moveToPosition(float x, float y, float z, uint16_t speed = 30) {
    // Previous moveToPosition implementation remains the same
    // ... (keep existing implementation)
}

void processJsonCommand(const JsonDocument& doc) {
    // Check for position command
    if (doc.containsKey("pos")) {
        // Parse the position string (format: "x, y, z")
        String posStr = doc["pos"].as<String>();
        
        // Split the string into x, y, z components
        int firstComma = posStr.indexOf(',');
        int secondComma = posStr.indexOf(',', firstComma + 1);
        
        float x = posStr.substring(0, firstComma).toFloat();
        float y = posStr.substring(firstComma + 1, secondComma).toFloat();
        float z = posStr.substring(secondComma + 1).toFloat();
        
        // Move to position
        moveToPosition(x, y, z);
        
        // Wait for movement to complete
        while (ServoEasing::areInterruptsActive()) {
            yield();  // Allow ESP32 to handle system tasks
        }
    }
    // Check for delay command
    else if (doc.containsKey("delay")) {
        int delayMs = doc["delay"].as<int>();
        delay(delayMs);
    }
}

void loop() {
    if (Serial.available()) {
        // Allocate the JSON document
        StaticJsonDocument<200> doc;  // Adjust size as needed
        
        // Read the JSON string from Serial
        String jsonString = Serial.readStringUntil('\n');
        
        // Parse JSON
        DeserializationError error = deserializeJson(doc, jsonString);
        
        // Check for parsing errors
        if (error) {
            Serial.print("deserializeJson() failed: ");
            Serial.println(error.c_str());
            return;
        }
        
        // Process the command
        processJsonCommand(doc);
    }
}