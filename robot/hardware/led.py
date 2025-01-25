import time
import Jetson.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
LED_PIN = 33  # Changed to pin 33

GPIO.setup(LED_PIN, GPIO.OUT)
pwm = GPIO.PWM(LED_PIN, 100)

try:
    print("Testing PWM fade on pin 33...")
    pwm.start(0)
    
    while True:
        # Fade in
        print("Fading in...")
        for duty in range(0, 101, 5):
            pwm.ChangeDutyCycle(duty)
            time.sleep(0.1)
        
        # Fade out
        print("Fading out...")
        for duty in range(100, -1, -5):
            pwm.ChangeDutyCycle(duty)
            time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    pwm.stop()
    GPIO.cleanup()