# RYAN BROWN: 730320130
# BETHANY NEWCOMB: 730311643

#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor, GyroSensor
from pybricks.parameters import Port, Button
# from pybricks.tools import wait

from tools import turn_angle, drive_until_touch

ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
ultrasonic = UltrasonicSensor(Port.S1)
touch = TouchSensor(Port.S2)
gyro = GyroSensor(Port.S3)


# PART 1
while(not Button.CENTER in ev3.buttons.pressed()):
    wait(100)

drive_until_touch(left_motor, right_motor, touch)

left_motor.run_angle(200, -270, wait=False)
right_motor.run_angle(200, -270, wait=True)

turn_angle(left_motor, right_motor, gyro, 90)


# # PART 2
# SET_POINT = ultrasonic.distance()
# TARGET = 4270

# avg = 0