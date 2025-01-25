import time
import Jetson.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
SERVO_PIN = 32  # Physical pin 32

# Set up GPIO with initial state
GPIO.setup(SERVO_PIN, GPIO.OUT, initial=GPIO.LOW)
time.sleep(1)  # Let it settle at LOW first

# Initialize PWM with servo standard frequency
pwm = GPIO.PWM(SERVO_PIN, 50)

try:
    print("Starting initialization sequence...")
    # Start with 0 duty cycle
    pwm.start(0)
    time.sleep(1)
    
    # Send a clear "first position" signal
    print("Setting initial position...")
    pwm.ChangeDutyCycle(7.5)  # Center position
    time.sleep(2)
    
    print("Beginning main sequence...")
    while True:
        # Move to first position
        print("Position 1")
        pwm.ChangeDutyCycle(5)
        time.sleep(3)
        
        # Brief pause at current position
        pwm.ChangeDutyCycle(0)  # Stop sending pulses briefly
        time.sleep(1)
        
        # Move to second position
        print("Position 2")
        pwm.ChangeDutyCycle(10)
        time.sleep(3)
        
        # Brief pause at current position
        pwm.ChangeDutyCycle(0)
        time.sleep(1)

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    pwm.stop()
    GPIO.cleanup()