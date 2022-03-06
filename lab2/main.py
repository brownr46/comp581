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

left_motor.run_angle(200, -270, wait=False)
right_motor.run_angle(200, -270, wait=True)

turn_angle(left_motor, right_motor, gyro, 90)


# PART 2
SET_POINT = ultrasonic.distance()
TARGET = 4000
SPEED = 125

avg = 0

left_motor.reset_angle(0)
right_motor.reset_angle(0)

left_motor.run(SPEED)
right_motor.run(SPEED)

while (avg < TARGET):
    distance = ultrasonic.distance()
    delta = distance - SET_POINT
    ev3.screen.clear()
    ev3.screen.draw_text(0,0, str(distance))

    scale = 0.0034 if delta > 0 else 0.01

    left_factor = max(0, min(1 - (delta * scale), 1.3)) 
    right_factor = max(0, min(1 + (delta * scale), 1.3))
    
    ev3.screen.draw_text(0, 30, str(left_factor))
    ev3.screen.draw_text(0, 60, str(right_factor))

    left_motor.run(SPEED * left_factor)
    right_motor.run(SPEED * right_factor)

    avg = (left_motor.angle() + right_motor.angle()) / 2
    wait(100)

left_motor.hold()
right_motor.hold()

ev3.speaker.beep(1000, 500)
