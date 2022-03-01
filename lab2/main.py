#!/usr/bin/env pybricks-micropython

# RYAN BROWN: 730320130
# BETHANY NEWCOMB: 730311643

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor, GyroSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait

def turn_angle(left, right, gyro, angle, speed=50):
    initial = gyro.angle()

    left.run(speed if angle > 0 else -speed)
    right.run(-speed if angle > 0 else speed)

    while (abs(initial + angle - gyro.angle()) > 2):
        wait(100)

    left.hold()
    right.hold()

def drive_until_touch(left, right, touch, speed=200):
    left.run(speed)
    right.run(speed)

    while (not touch.pressed()):
        wait(100)

    left.hold()
    right.hold()

ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
ultrasonic = UltrasonicSensor(Port.S1)
touch = TouchSensor(Port.S2)
gyro = GyroSensor(Port.S3)


# PART 1
while(not Button.CENTER in ev3.buttons.pressed()):
    wait(100)

drive_until_touch(left_motor, right_motor, touch, speed=250)

left_motor.run_angle(200, -360, wait=False)
right_motor.run_angle(200, -360, wait=True)

turn_angle(left_motor, right_motor, gyro, 90)


# PART 2
SET_POINT = ultrasonic.distance()
TARGET = 4270

avg = 0

left_motor.reset_angle(0)
right_motor.reset_angle(0)

left_motor.run(200)
right_motor.run(200)

while (avg < TARGET):
    wait(50)
    distance = ultrasonic.distance()

    if abs(distance - SET_POINT) > 5:
        if distance < SET_POINT:
            left_motor.run(300)
            right_motor.run(100)
        else:
            left_motor.run(100)
            right_motor.run(300)
    else:
        left_motor.run(200)
        right_motor.run(200)

    # left_motor.run_angle(300, 180, wait=False)
    # right_motor.run_angle(300, 180, wait=True)

    # if (abs(distance - SET_POINT) > 3):
    #     if distance < SET_POINT:
    #         turn_angle(left_motor, right_motor, gyro, 7.5)
    #     else:
    #         turn_angle(left_motor, right_motor, gyro, -7.5)

    avg = (left_motor.angle() + right_motor.angle()) / 2

left_motor.hold()
right_motor.hold()

ev3.speaker.beep(1000, 500)
