#include <ESP32Servo.h>
#include <ArduinoJson.h>

#define MAX_SERVOS 6
Servo servos[MAX_SERVOS];

// Define the GPIO pins connected to the servos
const int servoPins[MAX_SERVOS] = {13, 12, 14, 27, 26, 25};

// Current positions of each servo
float current_positions[MAX_SERVOS];

StaticJsonDocument<1024> doc;

void setup() {
    Serial.begin(115200);

    // Attach all servos to their pins and initialize to center position
    for(int i = 0; i < MAX_SERVOS; i++) {
        servos[i].attach(servoPins[i], 500, 2500);
        servos[i].write(90); // Initialize to 90 degrees
        current_positions[i] = 90.0;
        Serial.printf("Servo %d attached to pin %d, initialized to 90 degrees.\n", i, servoPins[i]);
    }

    Serial.println("Servo control initialized.");
}

void loop() {
    // Handle incoming serial commands
    while (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        handleCommand(input);
    }

    // No need to update motion profiles since movements are immediate
}

void handleCommand(String input) {
    DeserializationError error = deserializeJson(doc, input);
    if (error) {
        sendError("Invalid JSON");
        return;
    }

    if (!doc.containsKey("command")) {
        sendError("Missing command");
        return;
    }

    String command = doc["command"].as<String>();

    if (command == "move") {
        handleMoveCommand(doc);
    }
    else if (command == "move_multiple") {
        handleMoveMultipleCommand(doc);
    }
    else if (command == "status") {
        sendStatus();
    }
    else {
        sendError("Unknown command");
    }
}

void handleMoveCommand(JsonDocument& doc) {
    if (!doc.containsKey("servo_id") || !doc.containsKey("target")) {
        sendError("Missing parameters for move");
        return;
    }

    int servo_id = doc["servo_id"];
    float target = doc["target"];

    if (servo_id < 0 || servo_id >= MAX_SERVOS) {
        sendError("Invalid servo_id");
        return;
    }

    if (target < 0 || target > 270) {
        sendError("Target position out of range (0-270)");
        return;
    }

    servos[servo_id].write(round(target));
    current_positions[servo_id] = target;

    Serial.printf("Moved Servo %d to %.2f degrees.\n", servo_id, target);
    sendSuccess();
}

void handleMoveMultipleCommand(JsonDocument& doc) {
    if (!doc.containsKey("movements")) {
        sendError("Missing 'movements' for move_multiple");
        return;
    }

    JsonArray movements = doc["movements"].as<JsonArray>();
    for(JsonObject movement : movements) {
        if (!movement.containsKey("servo_id") || !movement.containsKey("target")) {
            sendError("Missing parameters in one of the movements");
            return;
        }

        int servo_id = movement["servo_id"];
        float target = movement["target"];

        if (servo_id < 0 || servo_id >= MAX_SERVOS) {
            sendError("Invalid servo_id in one of the movements");
            return;
        }

        if (target < 0 || target > 270) {
            sendError("Target position out of range (0-270) in one of the movements");
            return;
        }

        servos[servo_id].write(round(target));
        current_positions[servo_id] = target;

        Serial.printf("Moved Servo %d to %.2f degrees.\n", servo_id, target);
    }

    sendSuccess();
}

void sendSuccess() {
    StaticJsonDocument<100> response;
    response["status"] = "success";
    serializeJson(response, Serial);
    Serial.println();
}

void sendError(const String& message) {
    StaticJsonDocument<100> response;
    response["status"] = "error";
    response["message"] = message;
    serializeJson(response, Serial);
    Serial.println();
}

void sendStatus() {
    StaticJsonDocument<1024> response;
    JsonArray statusArray = response.createNestedArray("statuses");

    for(int i = 0; i < MAX_SERVOS; i++) {
        JsonObject servoStatus = statusArray.createNestedObject();
        servoStatus["servo_id"] = i;
        servoStatus["current_position"] = current_positions[i];
    }

    serializeJson(response, Serial);
    Serial.println();
}
