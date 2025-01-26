#include <ESP32Servo.h>
#include <ArduinoJson.h>

#define NUM_SERVOS 6          // Number of servos you're using
#define BUFFER_SIZE 512       // Increased buffer size for larger JSON commands

// Define the GPIO pins connected to the servos
const int servoPins[NUM_SERVOS] = {13, 12, 14, 27, 26, 25}; // Adjust these pins as per your setup

Servo servos[NUM_SERVOS];           // Array to hold Servo objects
StaticJsonDocument<BUFFER_SIZE> doc; // JSON document for parsing commands

// Function to find the index of a servo based on its GPIO pin
int findServoIndex(int pin) {
    for(int i = 0; i < NUM_SERVOS; i++) {
        if(servoPins[i] == pin) {
            return i;
        }
    }
    return -1; // Return -1 if the pin is not found
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; } // Wait for serial port to connect (only needed for some boards)

    // Attach each servo to its designated pin and initialize to center position (90 degrees)
    for(int i = 0; i < NUM_SERVOS; i++) {
        servos[i].attach(servoPins[i], 500, 2500); // Attach with min and max pulse widths
        servos[i].write(90); // Move to center position
        Serial.printf("Servo %d attached to pin %d and initialized to 90 degrees.\n", i, servoPins[i]);
    }

    Serial.println("Servo control initialized and ready.");
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n'); // Read incoming command
        DeserializationError error = deserializeJson(doc, input); // Parse JSON
        
        if (error) {
            Serial.println("{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
            return;
        }

        String command = doc["command"].as<String>();
        
        if (command == "move") {
            // Handle single servo movement
            if (!doc.containsKey("servo_id") || !doc.containsKey("position")) {
                Serial.println("{\"status\":\"error\",\"message\":\"Missing parameters\"}");
                return;
            }

            int servo_id = doc["servo_id"];
            int position = doc["position"];
            
            int index = findServoIndex(servo_id);
            if (index != -1) {
                position = constrain(position, 0, 270);
                servos[index].write(position);
                Serial.printf("{\"status\":\"success\",\"message\":\"Servo on pin %d moved to %d degrees\"}\n", servo_id, position);
            } else {
                Serial.printf("{\"status\":\"error\",\"message\":\"Invalid servo ID: %d\"}\n", servo_id);
            }
            return;
        }
        
        if (command == "move_multiple") {
            // Handle multiple servo movements
            if (!doc.containsKey("movements")) {
                Serial.println("{\"status\":\"error\",\"message\":\"Missing movements array\"}");
                return;
            }

            JsonArray movements = doc["movements"];
            bool success = true;
            String errorMessage = "";

            for (JsonObject movement : movements) {
                if (!movement.containsKey("servo_id") || !movement.containsKey("position")) {
                    success = false;
                    errorMessage = "Missing servo_id or position in movement";
                    break;
                }

                int servo_id = movement["servo_id"];
                int position = movement["position"];
                
                int index = findServoIndex(servo_id);
                if (index != -1) {
                    position = constrain(position, 0, 270);
                    servos[index].write(position);
                } else {
                    success = false;
                    errorMessage = "Invalid servo ID in movements";
                    break;
                }
            }
            
            if (success) {
                Serial.println("{\"status\":\"success\",\"message\":\"Servos moved\"}");
            } else {
                Serial.printf("{\"status\":\"error\",\"message\":\"%s\"}\n", errorMessage.c_str());
            }
            return;
        }
        
        // Handle unknown commands
        Serial.println("{\"status\":\"error\",\"message\":\"Unknown command\"}");
    }
}
