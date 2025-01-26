#include <ESP32Servo.h>
#include <ArduinoJson.h>

#define NUM_SERVOS 6
const int SERVO_PINS[NUM_SERVOS] = {12, 13, 14, 15, 16, 17};

const int SERVO_MIN_PULSE_WIDTH = 500;
const int SERVO_MAX_PULSE_WIDTH = 2500;

Servo servos[NUM_SERVOS];
StaticJsonDocument<1024> doc;

void setup() {
    Serial.begin(115200);
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].attach(SERVO_PINS[i], SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
    }
    Serial.println("Multi-servo control initialized.");
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        DeserializationError error = deserializeJson(doc, input);
        
        if (error) {
            Serial.println("{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
            return;
        }

        if (doc["command"] == "move_multiple") {
            JsonArray movements = doc["movements"];
            bool success = true;
            
            for (JsonObject movement : movements) {
                int servo_id = movement["servo_id"];
                int position = movement["position"];
                
                if (servo_id >= 0 && servo_id < NUM_SERVOS) {
                    position = constrain(position, 0, 270);
                    servos[servo_id].write(position);
                } else {
                    success = false;
                    break;
                }
            }
            
            if (success) {
                Serial.println("{\"status\":\"success\",\"message\":\"All servos moved\"}");
            } else {
                Serial.println("{\"status\":\"error\",\"message\":\"Invalid servo ID\"}");
            }
        }
    }
    delay(20);
}