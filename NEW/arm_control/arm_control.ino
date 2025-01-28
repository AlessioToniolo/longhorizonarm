#include <ESP32Servo.h>
#include <ArduinoJson.h>

// Servo configuration
struct ServoConfig {
    const int pin;
    const float max_vel;
    const float max_accel;
};

// any pins from 13 to 33


const ServoConfig SERVOS[] = {
    {13, 90.0, 45.0},  // Base rotation
    {12, 60.0, 30.0},  // Shoulder 1
    {14, 60.0, 30.0},  // Shoulder 2
    {26, 60.0, 30.0}   // Shoulder 3
};
const int NUM_SERVOS = 4;

struct MotionState {
    Servo servo;
    bool moving;
    float current_pos;
    float target_pos;
    float start_pos;
    float move_duration;
    float move_start_time;
    float max_vel;
    float accel_time;
    float cruise_time;
} servos[NUM_SERVOS];

hw_timer_t* timer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool update_needed = false;

void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    update_needed = true;
    portEXIT_CRITICAL_ISR(&timerMux);
}

void calculateProfile(int id) {
    MotionState& s = servos[id];
    const float distance = abs(s.target_pos - s.start_pos);
    const ServoConfig& config = SERVOS[id];
    
    const float min_time_to_peak = sqrt(distance / config.max_accel);
    const float peak_vel_possible = min_time_to_peak * config.max_accel;
    
    if (peak_vel_possible <= config.max_vel) {
        s.max_vel = peak_vel_possible;
        s.accel_time = min_time_to_peak;
        s.cruise_time = 0;
    } else {
        s.max_vel = config.max_vel;
        s.accel_time = config.max_vel / config.max_accel;
        s.cruise_time = (distance - (config.max_vel * s.accel_time)) / config.max_vel;
    }
    
    s.move_duration = 2 * s.accel_time + s.cruise_time;
}

float getProfilePosition(const MotionState& s, float elapsed_time) {
    if (elapsed_time >= s.move_duration) return s.target_pos;
    
    const float direction = (s.target_pos > s.start_pos) ? 1.0 : -1.0;
    const ServoConfig& config = SERVOS[&s - servos];
    
    if (elapsed_time < s.accel_time) {
        return s.start_pos + direction * (0.5 * config.max_accel * elapsed_time * elapsed_time);
    } 
    else if (elapsed_time < (s.accel_time + s.cruise_time)) {
        const float cruise_elapsed = elapsed_time - s.accel_time;
        return s.start_pos + direction * (0.5 * config.max_accel * s.accel_time * s.accel_time +
               s.max_vel * cruise_elapsed);
    } 
    else {
        const float decel_elapsed = elapsed_time - s.accel_time - s.cruise_time;
        const float cruise_dist = s.max_vel * s.cruise_time;
        const float decel_dist = s.max_vel * decel_elapsed - 
                               0.5 * config.max_accel * decel_elapsed * decel_elapsed;
        return s.start_pos + direction * (cruise_dist + decel_dist);
    }
}

void startMove(int id, float target, bool use_profile) {
    if (target < 0 || target > 180) return;
    
    MotionState& s = servos[id];
    
    if (use_profile) {
        s.moving = true;
        s.start_pos = s.current_pos;
        s.target_pos = target;
        s.move_start_time = millis() / 1000.0;
        calculateProfile(id);
    } else {
        s.moving = false;
        s.current_pos = target;
        s.servo.write(round(target));
    }
}

void setup() {
    Serial.begin(115200);
    delay(250);

    // Attach all servos normally and move them to 90°
    for(int i = 0; i < NUM_SERVOS; i++) {
        int channel = servos[i].servo.attach(SERVOS[i].pin, 500, 2500);
        
        // Store initial position as 90
        servos[i].current_pos = 90;
        servos[i].moving      = false;
        servos[i].servo.write(90);

        // Optional: debug output so you can see channels
        // Serial.print("Servo #");
        // Serial.print(i);
        // Serial.print(" attached on channel: ");
        // Serial.println(channel);
    }

    // Give the system a short delay, then detach & re-attach servo #2 (pin 14).
    // This often helps if pin 14 starts up in a conflicting mode.
    delay(200);
    servos[2].servo.detach();  
    servos[2].servo.attach(SERVOS[2].pin, 500, 2500);
    servos[2].servo.write(90);
    servos[2].current_pos = 90;
    servos[2].moving      = false;

    // Setup 50Hz timer (20ms) and attach interrupt
    timer = timerBegin(50);  // 50Hz frequency
    timerAttachInterrupt(timer, onTimer);
}


void loop() {
    if (Serial.available()) {
        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, Serial);
        
        if (!error && doc.containsKey("servo") && doc.containsKey("position")) {
            int id = doc["servo"];
            float pos = doc["position"];
            bool use_profile = doc.containsKey("profile") ? doc["profile"].as<bool>() : true;
            
            if (id >= 0 && id < NUM_SERVOS) {
                startMove(id, pos, use_profile);
            }
        }
    }
    
    if (update_needed) {
        portENTER_CRITICAL(&timerMux);
        update_needed = false;
        portEXIT_CRITICAL(&timerMux);
        
        float current_time = millis() / 1000.0;
        
        for(int i = 0; i < NUM_SERVOS; i++) {
            MotionState& s = servos[i];
            if (s.moving) {
                float elapsed_time = current_time - s.move_start_time;
                float new_pos = getProfilePosition(s, elapsed_time);
                s.servo.write(round(new_pos));
                s.current_pos = new_pos;
                
                if (elapsed_time >= s.move_duration) {
                    s.moving = false;
                }
            }
        }
    }
}