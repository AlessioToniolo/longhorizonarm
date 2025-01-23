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
        self.max_velocity = 60 # deg/sec
        self.max_acceleration = 45 # deg/sec^2

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, self.freq)
        self.pwm.start(0)

    def set_angle(self, angle):
        angle = np.clip(angle, self.min_angle, self.max_angle)
        self.current_angle = angle
        duty_cycle = self.angle_to_duty(angle)
        self.pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(self.delay)

    def set_angle_with_profile(self, target_angle):
        current_angle = self.current_angle
        angle_diff = target_angle - current_angle
        
        [t_accel, t_cruise, t_decel] = self.calculate_profile(angle_diff)
        
        step_time = 1.0 / self.freq
        total_time = t_accel + t_cruise + t_decel
        num_steps = int(total_time * self.freq)
        
        start_time = time.time()
        sign = np.sign(angle_diff)
        
        for step in range(num_steps):
            current_time = step * step_time
            
            if current_time <= t_accel:
                dist = 0.5 * self.max_acceleration * current_time**2
            elif current_time <= t_accel + t_cruise:
                dist = (0.5 * self.max_acceleration * t_accel**2) + \
                      self.max_velocity * (current_time - t_accel)
            else:
                t = current_time - t_accel - t_cruise
                dist = (0.5 * self.max_acceleration * t_accel**2) + \
                      (self.max_velocity * t_cruise) + \
                      (self.max_velocity * t - 0.5 * self.max_acceleration * t**2)
            
            new_angle = current_angle + sign * dist
            duty = self.angle_to_duty(new_angle)
            self.pwm.ChangeDutyCycle(duty)
            
            time.sleep(step_time)
            
        final_duty = self.angle_to_duty(target_angle)
        self.pwm.ChangeDutyCycle(final_duty)
        self.current_angle = target_angle

    def angle_to_duty(self, angle):
        pulse_width = self.min_pulse + (angle / self.max_angle) * (self.max_pulse - self.min_pulse) # linear interpolation (slope * range + ymin)
        duty_cycle = (pulse_width / 1000000) * self.freq * 100 # pulse_width / period * 100 (percent)
        return duty_cycle

    def reset_profile(self):
        self.max_velocity = 60
        self.max_acceleration = 45

    def get_angle(self):
        return self.current_angle

    def get_min_angle(self):
        return self.min_angle
    
    def get_max_angle(self):
        return self.max_angle
    
    def get_home(self):
        return self.home

    def calculate_profile(self, angle_diff):
        t_accel = self.max_velocity / self.max_acceleration

        distance_accel = .5 * self.max_acceleration * t_accel**2

        if 2*distance_accel > np.abs(angle_diff):
            t_accel = np.sqrt(abs(angle_diff) / self.max_acceleration)
            cruise = 0
        else:
            t_cruise = (np.abs(angle_diff) - 2*distance_accel) / self.max_velocity

        return [t_accel, t_cruise, t_accel]
    
    def calculate_profile_total_time(self, angle_diff):
        [t_accel, t_cruise, t_decel] = self.calculate_profile(angle_diff)
        return t_accel + t_cruise + t_decel

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup(self.pin)