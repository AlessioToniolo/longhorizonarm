from .Servo import Servo
test = Servo(7, 0, 270, 90)
test.set_angle(90)
test.delay(2)
test.set_angle(180)