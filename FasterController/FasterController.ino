#include <ESP32Servo.h>
#include <ArduinoJson.h>

#define MAX_SERVOS 6
Servo servos[MAX_SERVOS];

// Define the GPIO pins connected to the servos
const int servoPins[MAX_SERVOS] = {13, 12, 14, 27, 26, 25};

// Timer for consistent updates
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool updateFlag = false;

// Profile parameters for each servo
struct ProfileState {
    bool in_motion;
    float current_pos;
    float target_pos;
    float current_vel;
    float max_vel;
    float max_accel;
    float start_pos;
    float move_duration;
    float move_start_time;
    bool profile_calculated;
    float peak_vel;
    float accel_time;
    float decel_time;
    float cruise_time;
} profiles[MAX_SERVOS];

// JSON document
StaticJsonDocument<1024> doc;

// Timer ISR - Keep it minimal
void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    updateFlag = true;
    portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
    Serial.begin(115200);
    
    // Initialize all profiles
    for(int i = 0; i < MAX_SERVOS; i++) {
        profiles[i].in_motion = false;
        profiles[i].profile_calculated = false;
    }

    // Attach all servos to their pins and initialize to center position
    for(int i = 0; i < MAX_SERVOS; i++) {
        servos[i].attach(servoPins[i], 500, 2500);
        servos[i].write(90);
        profiles[i].current_pos = 90;
        Serial.printf("Servo %d attached to pin %d, initialized to 90 degrees.\n", i, servoPins[i]);
    }

    // Setup timer for 50Hz updates (20ms)
    timer = timerBegin(0, 80, true);  // 80MHz / 80 = 1MHz timer
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 20000, true);  // 20ms intervals (50Hz)
    timerAlarmEnable(timer);

    Serial.println("Servo control initialized.");
}

void loop() {
    // Handle incoming serial commands
    while (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        handleCommand(input);
    }

    // Check if it's time to update profiles
    if (updateFlag) {
        portENTER_CRITICAL(&timerMux);
        updateFlag = false;
        portEXIT_CRITICAL(&timerMux);
        
        // Update all active profiles
        for(int i = 0; i < MAX_SERVOS; i++) {
            if (profiles[i].in_motion) {
                updateProfiledMove(i);
            }
        }
    }
}

void calculateProfile(int servo_id) {
    ProfileState& p = profiles[servo_id];
    float distance = abs(p.target_pos - p.start_pos);
    
    // Calculate minimum time needed for the move
    float min_time_to_peak = sqrt(distance / p.max_accel);
    float peak_vel_possible = min_time_to_peak * p.max_accel;
    
    if (peak_vel_possible <= p.max_vel) {
        // Triangular profile
        p.peak_vel = peak_vel_possible;
        p.accel_time = min_time_to_peak;
        p.decel_time = min_time_to_peak;
        p.cruise_time = 0;
    } else {
        // Trapezoidal profile
        p.peak_vel = p.max_vel;
        p.accel_time = p.max_vel / p.max_accel;
        float cruise_distance = distance - (p.max_vel * p.accel_time);
        p.cruise_time = cruise_distance / p.max_vel;
        p.decel_time = p.accel_time;
    }
    
    p.move_duration = p.accel_time + p.cruise_time + p.decel_time;
    p.profile_calculated = true;
}

float getProfilePosition(int servo_id, float elapsed_time) {
    ProfileState& p = profiles[servo_id];
    float direction = (p.target_pos > p.start_pos) ? 1.0 : -1.0;
    float pos = p.start_pos;
    
    if (elapsed_time >= p.move_duration) {
        return p.target_pos;
    }
    
    if (elapsed_time < p.accel_time) {
        // Acceleration phase
        pos += direction * (0.5 * p.max_accel * elapsed_time * elapsed_time);
    } else if (elapsed_time < (p.accel_time + p.cruise_time)) {
        // Cruise phase
        float cruise_elapsed = elapsed_time - p.accel_time;
        pos += direction * (0.5 * p.max_accel * p.accel_time * p.accel_time +
               p.peak_vel * cruise_elapsed);
    } else {
        // Deceleration phase
        float total_cruise = 0.5 * p.max_accel * p.accel_time * p.accel_time +
                           p.peak_vel * p.cruise_time;
        float decel_elapsed = elapsed_time - p.accel_time - p.cruise_time;
        float decel_distance = p.peak_vel * decel_elapsed -
                             0.5 * p.max_accel * decel_elapsed * decel_elapsed;
        pos += direction * (total_cruise + decel_distance);
    }
    
    return pos;
}

void startProfiledMove(int servo_id, float target, float max_vel, float max_accel) {
    profiles[servo_id].in_motion = true;
    profiles[servo_id].start_pos = profiles[servo_id].current_pos;
    profiles[servo_id].target_pos = target;
    profiles[servo_id].max_vel = max_vel;
    profiles[servo_id].max_accel = max_accel;
    profiles[servo_id].move_start_time = millis() / 1000.0;  // Convert to seconds
    profiles[servo_id].profile_calculated = false;
    
    calculateProfile(servo_id);

    Serial.printf("Starting profiled move for Servo %d to %.2f degrees with max_vel=%.2f, max_accel=%.2f\n",
                  servo_id, target, max_vel, max_accel);
    sendSuccess();
}

void updateProfiledMove(int servo_id) {
    ProfileState& p = profiles[servo_id];
    
    if (!p.profile_calculated) {
        calculateProfile(servo_id);
    }
    
    float current_time = millis() / 1000.0;  // Convert to seconds
    float elapsed_time = current_time - p.move_start_time;
    
    float new_pos = getProfilePosition(servo_id, elapsed_time);
    
    // Check if move is complete
    if (elapsed_time >= p.move_duration) {
        new_pos = p.target_pos;
        p.in_motion = false;
        p.profile_calculated = false;
    }
    
    // Update servo position
    servos[servo_id].write(round(new_pos));
    p.current_pos = new_pos;
}

// Your existing handleCommand, sendSuccess, sendError, and sendStatus functions remain the same
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

        if (profiles[servo_id].in_motion) {
            sendError("Another motion is in progress for this servo");
            return;
        }

        startProfiledMove(servo_id, target, max_vel, max_accel);
    }
    else if (command == "move_multiple_profiled") {
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
        sendStatus();
    }
    else {
        sendError("Unknown command");
    }
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
        }
    }

    serializeJson(response, Serial);
    Serial.println();
}