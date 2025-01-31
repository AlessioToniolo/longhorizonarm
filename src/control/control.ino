#include <Arduino.h>
#include <ArduinoJson.h>
#include "ServoEasing.hpp"

ServoEasing base;
ServoEasing shoulder1;
ServoEasing shoulder2;
ServoEasing wrist;
ServoEasing twist;
// ServoEasing gripper;

// Increased buffer size to handle array of commands
const size_t JSON_BUFFER_SIZE = 1024;

void setup() {
    Serial.begin(115200);
    
    base.attach(13, 90);
    shoulder1.attach(12, 90);
    shoulder2.attach(14, 90);
    wrist.attach(27, 90);
    twist.attach(26, 90);  // Added twist servo on pin 26
    // gripper.attach(25, 90);  // Gripper servo would use pin 25

    delay(500);
    moveServos(90, 90, 90, 90, 90); // Home position (added twist)
    // moveServos(90, 90, 90, 90, 90, 90); // Home position with gripper
    delay(3000);
}

void moveServos(int pos1, int pos2, int pos3, int pos4, int pos5) {
    base.setEaseTo(pos1);
    shoulder1.setEaseTo(pos2);
    shoulder2.setEaseTo(pos3);
    wrist.setEaseTo(pos4);
    twist.setEaseTo(pos5);
    
    setSpeedForAllServos(40);
    synchronizeAllServosAndStartInterrupt();
    
    while (ServoEasing::areInterruptsActive()) {
        delay(5);
    }
}

/* Commented out extended moveServos function for gripper
void moveServos(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6) {
    base.setEaseTo(pos1);
    shoulder1.setEaseTo(pos2);
    shoulder2.setEaseTo(pos3);
    wrist.setEaseTo(pos4);
    twist.setEaseTo(pos5);
    gripper.setEaseTo(pos6);
    
    setSpeedForAllServos(40);
    synchronizeAllServosAndStartInterrupt();
    
    while (ServoEasing::areInterruptsActive()) {
        delay(5);
    }
}
*/

void processCommand(JsonObject command) {
    int basePos = command["base"] | 90;
    int shoulder1Pos = command["shoulder1"] | 90;
    int shoulder2Pos = command["shoulder2"] | 90;
    int wristPos = command["wrist"] | 90;
    int twistPos = command["twist"] | 90;
    // int gripperPos = command["gripper"] | 90;
    int delayAfter = command["delayAfter"] | 0;  // Default 0ms delay if not specified
    
    moveServos(basePos, shoulder1Pos, shoulder2Pos, wristPos, twistPos);
    // moveServos(basePos, shoulder1Pos, shoulder2Pos, wristPos, twistPos, gripperPos);
    
    if (delayAfter > 0) {
        delay(delayAfter);
    }
}

void loop() {
    if (Serial.available()) {
        StaticJsonDocument<JSON_BUFFER_SIZE> doc;
        String jsonString = Serial.readStringUntil('\n');
        DeserializationError error = deserializeJson(doc, jsonString);
        
        if (!error) {
            if (doc.is<JsonArray>()) {
                JsonArray commands = doc.as<JsonArray>();
                for (JsonObject command : commands) {
                    processCommand(command);
                }
            } else {
                processCommand(doc.as<JsonObject>());
            }
        } else {
            Serial.println("Error parsing JSON");
        }
    }
}