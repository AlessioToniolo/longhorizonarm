#include <ESP32Servo.h>
#include <ArduinoJson.h>

#define MAX_SERVOS 6
Servo servos[MAX_SERVOS];

// Define the GPIO pins connected to the servos
const int servoPins[MAX_SERVOS] = {13, 12, 14, 27, 26, 25};

// Profile parameters for a single servo
struct ProfileState {
    int servo_id;
    float current_pos;
    float target_pos;
    float current_vel;
    float max_vel;
    float max_accel;
    unsigned long last_update_time;
    bool in_motion;
} profile;

// JSON document
StaticJsonDocument<512> doc;

void setup() {
    Serial.begin(115200);
    profile.in_motion = false;

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
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        handleCommand(input);
    }

    // Update motion profile if in motion
    if (profile.in_motion) {
        updateProfiledMove();
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

        // If another motion is in progress, reject the new command
        if (profile.in_motion) {
            sendError("Another motion is in progress");
            return;
        }

        startProfiledMove(servo_id, target, max_vel, max_accel);
    }
    else if (command == "move") {
        // Handle regular move
        if (!doc.containsKey("servo_id") || !doc.containsKey("position")) {
            sendError("Missing parameters for move");
            return;
        }

        int servo_id = doc["servo_id"];
        int position = doc["position"];

        if (servo_id < 0 || servo_id >= MAX_SERVOS) {
            sendError("Invalid servo_id");
            return;
        }

        servos[servo_id].write(position);
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
    profile.servo_id = servo_id;
    profile.current_pos = servos[servo_id].read();
    profile.target_pos = target;
    profile.current_vel = 0;
    profile.max_vel = max_vel;
    profile.max_accel = max_accel;
    profile.last_update_time = millis();
    profile.in_motion = true;

    Serial.printf("Starting profiled move for Servo %d to %0.2f degrees with max_vel=%0.2f, max_accel=%0.2f\n",
                  servo_id, target, max_vel, max_accel);
    sendSuccess();
}

void updateProfiledMove() {
    unsigned long current_time = millis();
    float dt = (current_time - profile.last_update_time) / 1000.0; // Convert to seconds

    // Prevent dt from being too large or too small
    if (dt <= 0 || dt > 1) {
        dt = 0.02; // Assume 20ms if timing is off
    }

    profile.last_update_time = current_time;

    float distance = profile.target_pos - profile.current_pos;
    float direction = (distance > 0) ? 1.0 : -1.0;
    float abs_distance = abs(distance);

    // Calculate the maximum possible velocity considering acceleration
    float accel_time = profile.max_vel / profile.max_accel;
    float accel_dist = 0.5 * profile.max_accel * accel_time * accel_time;

    float new_pos = profile.current_pos;
    float new_vel = profile.current_vel;

    if (abs_distance < 2 * accel_dist) {
        // Triangular profile
        float peak_time = sqrt(abs_distance / profile.max_accel);
        if (dt < peak_time) {
            // Accelerating
            new_vel += profile.max_accel * dt;
            if (new_vel > profile.max_vel) new_vel = profile.max_vel;
            new_pos += direction * new_vel * dt;
        }
        else if (dt < 2 * peak_time) {
            // Decelerating
            new_vel -= profile.max_accel * dt;
            if (new_vel < 0) new_vel = 0;
            new_pos += direction * new_vel * dt;
        }
        else {
            // Motion complete
            new_pos = profile.target_pos;
            new_vel = 0;
            profile.in_motion = false;
        }
    }
    else {
        // Trapezoidal profile
        float const_dist = abs_distance - 2 * accel_dist;
        float const_time = const_dist / profile.max_vel;
        float total_time = 2 * accel_time + const_time;

        if (dt < accel_time) {
            // Accelerating
            new_vel += profile.max_accel * dt;
            if (new_vel > profile.max_vel) new_vel = profile.max_vel;
            new_pos += direction * new_vel * dt;
        }
        else if (dt < accel_time + const_time) {
            // Constant velocity
            new_vel = profile.max_vel;
            new_pos += direction * new_vel * dt;
        }
        else if (dt < total_time) {
            // Decelerating
            new_vel -= profile.max_accel * dt;
            if (new_vel < 0) new_vel = 0;
            new_pos += direction * new_vel * dt;
        }
        else {
            // Motion complete
            new_pos = profile.target_pos;
            new_vel = 0;
            profile.in_motion = false;
        }
    }

    // Clamp new_pos to target_pos
    if ((direction > 0 && new_pos > profile.target_pos) ||
        (direction < 0 && new_pos < profile.target_pos)) {
        new_pos = profile.target_pos;
        new_vel = 0;
        profile.in_motion = false;
    }

    // Update servo position
    servos[profile.servo_id].write(round(new_pos));
    profile.current_pos = new_pos;
    profile.current_vel = new_vel;

    // Debugging information
    Serial.printf("Servo %d: pos=%.2f, target=%.2f, vel=%.2f, in_motion=%s\n",
                  profile.servo_id, profile.current_pos, profile.target_pos,
                  profile.current_vel, profile.in_motion ? "true" : "false");
}

void sendSuccess() {
    StaticJsonDocument<100> response;
    response["status"] = "success";
    serializeJson(response, Serial);
    Serial.println();
}

void sendError(const char* message) {
    StaticJsonDocument<100> response;
    response["status"] = "error";
    response["message"] = message;
    serializeJson(response, Serial);
    Serial.println();
}

void sendStatus() {
    StaticJsonDocument<200> response;
    response["status"] = profile.in_motion ? "moving" : "complete";
    if (profile.in_motion) {
        response["servo_id"] = profile.servo_id;
        response["current_position"] = profile.current_pos;
        response["target_position"] = profile.target_pos;
    }
    serializeJson(response, Serial);
    Serial.println();
}
