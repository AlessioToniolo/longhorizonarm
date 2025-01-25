import time
import Jetson.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
SERVO_PIN = 32  # Physical pin 32

GPIO.setup(SERVO_PIN, GPIO.OUT)

try:
    print("Testing HIGH-biased pulses...")
    while True:
        GPIO.output(SERVO_PIN, GPIO.HIGH)
        time.sleep(0.002)  # 2ms high pulse
        GPIO.output(SERVO_PIN, GPIO.LOW)
        time.sleep(0.0001)  # very brief low pulse
        print("Pulse")

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    GPIO.cleanup()