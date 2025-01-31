#include <ESP32Servo.h>
#include <ArduinoJson.h>

#define MAX_SERVOS 6
Servo servos[MAX_SERVOS];

// Define the GPIO pins connected to the servos
const int servoPins[MAX_SERVOS] = {13, 12, 14, 27, 26, 25};

// Profile parameters for each servo
struct ProfileState {
    bool in_motion;
    float current_pos;
    float target_pos;
    float current_vel;
    float max_vel;
    float max_accel;
    unsigned long last_update_time;
} profiles[MAX_SERVOS];

// JSON document
StaticJsonDocument<1024> doc;

void setup() {
    Serial.begin(115200);
    
    // Initialize all profiles
    for(int i = 0; i < MAX_SERVOS; i++) {
        profiles[i].in_motion = false;
    }

    // Attach all servos to their pins and initialize to center position
    for(int i = 0; i < MAX_SERVOS; i++) {
        servos[i].attach(servoPins[i], 500, 2500);
        servos[i].write(90);
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

    // Update motion profiles if in motion
    for(int i = 0; i < MAX_SERVOS; i++) {
        if (profiles[i].in_motion) {
            updateProfiledMove(i);
        }
    }
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

    if (command == "move_profiled") {
        // Handle profiled move
        if (!doc.containsKey("servo_id") || !doc.containsKey("target") || 
            !doc.containsKey("max_vel") || !doc.containsKey("max_accel")) {
            sendError("Missing parameters for move_profiled");
            return;
        }

        int servo_id = doc["servo_id"];
        float target = doc["target"];
        float max_vel = doc["max_vel"];
        float max_accel = doc["max_accel"];

        if (servo_id < 0 || servo_id >= MAX_SERVOS) {
            sendError("Invalid servo_id");
            return;
        }

        // If another motion is in progress for this servo, reject the new command
        if (profiles[servo_id].in_motion) {
            sendError("Another motion is in progress for this servo");
            return;
        }

        startProfiledMove(servo_id, target, max_vel, max_accel);
    }
    else if (command == "move_multiple_profiled") {
        // Handle multiple profiled moves
        if (!doc.containsKey("movements")) {
            sendError("Missing 'movements' for move_multiple_profiled");
            return;
        }

        JsonArray movements = doc["movements"].as<JsonArray>();
        for(JsonObject movement : movements) {
            if (!movement.containsKey("servo_id") || !movement.containsKey("target") ||
                !movement.containsKey("max_vel") || !movement.containsKey("max_accel")) {
                sendError("Missing parameters in one of the movements");
                return;
            }

            int servo_id = movement["servo_id"];
            float target = movement["target"];
            float max_vel = movement["max_vel"];
            float max_accel = movement["max_accel"];

            if (servo_id < 0 || servo_id >= MAX_SERVOS) {
                sendError("Invalid servo_id in one of the movements");
                return;
            }

            if (profiles[servo_id].in_motion) {
                sendError("Another motion is in progress for servo_id " + String(servo_id));
                return;
            }

            startProfiledMove(servo_id, target, max_vel, max_accel);
        }

        sendSuccess();
    }
    else if (command == "status") {
        // Handle status
        sendStatus();
    }
    else {
        sendError("Unknown command");
    }
}

void startProfiledMove(int servo_id, float target, float max_vel, float max_accel) {
    profiles[servo_id].in_motion = true;
    profiles[servo_id].current_pos = servos[servo_id].read();
    profiles[servo_id].target_pos = target;
    profiles[servo_id].current_vel = 0;
    profiles[servo_id].max_vel = max_vel;
    profiles[servo_id].max_accel = max_accel;
    profiles[servo_id].last_update_time = millis();

    Serial.printf("Starting profiled move for Servo %d to %.2f degrees with max_vel=%.2f, max_accel=%.2f\n",
                  servo_id, target, max_vel, max_accel);
    sendSuccess();
}

void updateProfiledMove(int servo_id) {
    unsigned long current_time = millis();
    float dt = (current_time - profiles[servo_id].last_update_time) / 1000.0; // seconds

    // Prevent dt from being too large or too small
    if (dt <= 0 || dt > 1) {
        dt = 0.02; // Assume 20ms if timing is off
    }

    profiles[servo_id].last_update_time = current_time;

    float distance = profiles[servo_id].target_pos - profiles[servo_id].current_pos;
    float direction = (distance > 0) ? 1.0 : -1.0;
    float abs_distance = abs(distance);

    // Calculate the maximum possible velocity considering acceleration
    float accel_time = profiles[servo_id].max_vel / profiles[servo_id].max_accel;
    float accel_dist = 0.5 * profiles[servo_id].max_accel * accel_time * accel_time;

    float new_pos = profiles[servo_id].current_pos;
    float new_vel = profiles[servo_id].current_vel;

    if (abs_distance < 2 * accel_dist) {
        // Triangular profile
        float peak_time = sqrt(abs_distance / profiles[servo_id].max_accel);
        if (dt < peak_time) {
            // Accelerating
            new_vel += profiles[servo_id].max_accel * dt;
            if (new_vel > profiles[servo_id].max_vel) new_vel = profiles[servo_id].max_vel;
            new_pos += direction * new_vel * dt;
        }
        else if (dt < 2 * peak_time) {
            // Decelerating
            new_vel -= profiles[servo_id].max_accel * dt;
            if (new_vel < 0) new_vel = 0;
            new_pos += direction * new_vel * dt;
        }
        else {
            // Motion complete
            new_pos = profiles[servo_id].target_pos;
            new_vel = 0;
            profiles[servo_id].in_motion = false;
        }
    }
    else {
        // Trapezoidal profile
        float const_dist = abs_distance - 2 * accel_dist;
        float const_time = const_dist / profiles[servo_id].max_vel;
        float total_time = 2 * accel_time + const_time;

        if (dt < accel_time) {
            // Accelerating
            new_vel += profiles[servo_id].max_accel * dt;
            if (new_vel > profiles[servo_id].max_vel) new_vel = profiles[servo_id].max_vel;
            new_pos += direction * new_vel * dt;
        }
        else if (dt < accel_time + const_time) {
            // Constant velocity
            new_vel = profiles[servo_id].max_vel;
            new_pos += direction * new_vel * dt;
        }
        else if (dt < total_time) {
            // Decelerating
            new_vel -= profiles[servo_id].max_accel * dt;
            if (new_vel < 0) new_vel = 0;
            new_pos += direction * new_vel * dt;
        }
        else {
            // Motion complete
            new_pos = profiles[servo_id].target_pos;
            new_vel = 0;
            profiles[servo_id].in_motion = false;
        }
    }

    // Clamp new_pos to target_pos
    if ((direction > 0 && new_pos > profiles[servo_id].target_pos) ||
        (direction < 0 && new_pos < profiles[servo_id].target_pos)) {
        new_pos = profiles[servo_id].target_pos;
        new_vel = 0;
        profiles[servo_id].in_motion = false;
    }

    // Update servo position
    servos[servo_id].write(round(new_pos));
    profiles[servo_id].current_pos = new_pos;
    profiles[servo_id].current_vel = new_vel;

    // Debugging information
    Serial.printf("Servo %d: pos=%.2f, target=%.2f, vel=%.2f, in_motion=%s\n",
                  servo_id, profiles[servo_id].current_pos, profiles[servo_id].target_pos,
                  profiles[servo_id].current_vel, profiles[servo_id].in_motion ? "true" : "false");
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
        servoStatus["status"] = profiles[i].in_motion ? "moving" : "complete";
        if (profiles[i].in_motion) {
            servoStatus["current_position"] = profiles[i].current_pos;
            servoStatus["target_position"] = profiles[i].target_pos;
            servoStatus["current_velocity"] = profiles[i].current_vel;
        }
    }

    serializeJson(response, Serial);
    Serial.println();
}
