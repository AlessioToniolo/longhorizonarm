import time
import Jetson.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
SERVO_PIN = 32  # Physical pin 32

GPIO.setup(SERVO_PIN, GPIO.OUT)

def send_pulses(pulse_width, duration):
    """Send pulses at specified width for given duration"""
    end_time = time.time() + duration
    while time.time() < end_time:
        GPIO.output(SERVO_PIN, GPIO.HIGH)
        time.sleep(pulse_width)
        GPIO.output(SERVO_PIN, GPIO.LOW)
        time.sleep(0.02 - pulse_width)  # Complete 20ms cycle

try:
    print("Testing known good pulse widths...")
    while True:
        # Position 1
        print("Position 1 (1.59ms)")
        send_pulses(0.00159, 3)
        time.sleep(2)
        
        # Position 2
        print("Position 2 (1.62ms)")
        send_pulses(0.00162, 3)
        time.sleep(2)

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    GPIO.cleanup()