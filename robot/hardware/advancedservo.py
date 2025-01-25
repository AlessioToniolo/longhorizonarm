import time
import Jetson.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
SERVO_PIN = 32  # Physical pin 32

GPIO.setup(SERVO_PIN, GPIO.OUT)

def send_pulses(pulse_width, duration):
    """Send consistent pulses for a specified duration"""
    end_time = time.time() + duration
    while time.time() < end_time:
        GPIO.output(SERVO_PIN, GPIO.HIGH)
        time.sleep(pulse_width)
        GPIO.output(SERVO_PIN, GPIO.LOW)
        time.sleep(0.02 - pulse_width)  # Complete 20ms cycle

try:
    print("Testing with very slow, conservative movements...")
    while True:
        # Start position
        print("Position 1")
        send_pulses(0.0015, 3)  # Hold for 3 seconds
        time.sleep(2)  # Long pause
        print("Starting slow movement...")
        
        # Move through middle extremely slowly
        for pw in range(150, 170, 1):  # Small steps
            pulse_width = pw / 100000.0
            print(f"Pulse width: {pulse_width:.6f}")
            send_pulses(pulse_width, 0.2)  # Longer duration per step
            time.sleep(0.5)  # Pause between each tiny movement
        
        print("Reached Position 2")
        send_pulses(0.0017, 3)  # Hold for 3 seconds
        time.sleep(2)  # Long pause
        print("Starting return movement...")
        
        # Move back extremely slowly
        for pw in range(170, 150, -1):
            pulse_width = pw / 100000.0
            print(f"Pulse width: {pulse_width:.6f}")
            send_pulses(pulse_width, 0.2)
            time.sleep(0.5)  # Pause between each tiny movement
        
        print("Completed one cycle")
        time.sleep(3)  # Long pause between cycles

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    GPIO.cleanup()