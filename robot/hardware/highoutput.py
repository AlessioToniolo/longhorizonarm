import time
import Jetson.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
SERVO_PIN = 32  # Physical pin 32

GPIO.setup(SERVO_PIN, GPIO.OUT)

try:
    print("Testing simple HIGH signal...")
    while True:
        # Just try to hold HIGH like the 3V3 pin
        GPIO.output(SERVO_PIN, GPIO.HIGH)
        time.sleep(5)
        print("Holding HIGH")
        
        # Brief reset
        GPIO.output(SERVO_PIN, GPIO.LOW)
        time.sleep(1)
        print("Reset LOW")

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    GPIO.cleanup()