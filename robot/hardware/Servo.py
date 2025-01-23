import Jetson.GPIO as GPIO
import time
import numpy as np

class Servo:
    def __init__(self, pin, min_angle, max_angle, home, delay=0.2):
        self.pin = pin
        self.delay = delay
        self.freq = 50
        self.min_pulse = 500
        self.max_pulse = 2500
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.home = home
        self.current_angle = 0

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, self.freq)
        self.pwm.start(0)

    def set_angle(self, angle):
        angle = np.clip(angle, self.min_angle, self.max_angle)
        self.current_angle = angle
        pulse_width = self.min_pulse + (angle / self.max_angle) * (self.max_pulse - self.min_pulse) # linear interpolation (slope * range + ymin)
        duty_cycle = (pulse_width / 1000000) * self.freq * 100 # pulse_width / period * 100 (percent)
        self.pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(self.delay)

    def get_angle(self):
        return self.current_angle

    def get_min_angle(self):
        return self.min_angle
    
    def get_max_angle(self):
        return self.max_angle
    
    def get_home(self):
        return self.home

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup(self.pin)