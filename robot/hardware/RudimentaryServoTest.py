import time

import Jetson.GPIO as GPIO

# Set up GPIO using BCM numbering
GPIO.setmode(GPIO.BOARD)

# Servo pin (physical pin 32)
SERVO_PIN = 32

# Set up the GPIO pin for PWM
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Create PWM instance with 50Hz frequency
pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz is standard for servos

try:
    # Start PWM with neutral position (7.5% duty cycle)
    pwm.start(7.5)
    print("Servo test starting...")
    
    while True:
        # Move to 0 degrees (2.5% duty cycle)
        print("Moving to 0 degrees")
        pwm.ChangeDutyCycle(2.5)
        time.sleep(1)
        
        # Move to 90 degrees (7.5% duty cycle)
        print("Moving to 90 degrees")
        pwm.ChangeDutyCycle(7.5)
        time.sleep(1)
        
        # Move to 180 degrees (12.5% duty cycle)
        print("Moving to 180 degrees")
        pwm.ChangeDutyCycle(12.5)
        time.sleep(1)

except KeyboardInterrupt:
    print("\nStopping servo test...")
    
finally:
    pwm.stop()
    GPIO.cleanup()
    print("GPIO cleanup completed")