// Include the Servo library
#include <ESP32Servo.h>

// Define the GPIO pin connected to the servo
#define SERVO_PIN 13 // Replace with the pin connected to your servo

// Servo specifications (adjust based on your servo's datasheet)
const int SERVO_MIN_PULSE_WIDTH = 500; // Minimum pulse width in microseconds (0 degrees)
const int SERVO_MAX_PULSE_WIDTH = 2500; // Maximum pulse width in microseconds (270 degrees)

Servo servo; // Create a Servo object

void setup() {
  Serial.begin(115200); // Initialize serial communication for debugging

  // Attach the servo to the specified pin
  servo.attach(SERVO_PIN, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);

  Serial.println("Servo control initialized.");
}

void loop() {
  // Prompt the user to input an angle
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int angle = input.toInt(); // Convert input to an integer

    // Ensure the angle is within the valid range (0-270 degrees)
    angle = constrain(angle, 0, 270);

    // Write the angle to the servo
    servo.write(angle);

    // Output the angle for debugging
    Serial.print("Servo angle set to: ");
    Serial.println(angle);
  }

  delay(20); // Small delay to allow the servo to move
}
