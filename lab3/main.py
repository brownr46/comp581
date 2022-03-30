#!/usr/bin/env pybricks-micropython

# RYAN BROWN: 730320130
# BETHANY NEWCOMB: 730311643

import math
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

def run_angle(left, right, speed, angle):
    left.run_angle(speed, angle, wait=False)
    right.run_angle(speed, angle, wait=True)


ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
ultrasonic = UltrasonicSensor(Port.S1)
touch = TouchSensor(Port.S2)
gyro = GyroSensor(Port.S3)


# PART 1
while(not Button.CENTER in ev3.buttons.pressed()):
    wait(100)

left_motor.reset_angle(0)
right_motor.reset_angle(0)

drive_until_touch(left_motor, right_motor, touch, speed=250)

INITIAL_DISTANCE = left_motor.angle()

ev3.speaker.beep(1000, 500)

run_angle(left_motor, right_motor, 200, -275)

turn_angle(left_motor, right_motor, gyro, 90)


# PART 2
SET_POINT = ultrasonic.distance()
SPEED = 150
L = 12
RADIUS = 2.8

x = y = theta = distance = cycle = 0

left_motor.reset_angle(0)
right_motor.reset_angle(0)
gyro.reset_angle(0)

left_motor.run(SPEED)
right_motor.run(SPEED)

while (distance > 13 or cycle < 100):
    if touch.pressed():
        run_angle(left_motor, right_motor, 200, -250)

        ev3.speaker.say('Oh no I have hit a wall')

        x = x - 12.2 * math.cos(theta)
        y = y - 12.2 * math.sin(theta)

        turn_angle(left_motor, right_motor, gyro, 75)

    distance = ultrasonic.distance()
    delta = distance - SET_POINT
    ev3.screen.clear()
    ev3.screen.draw_text(0,0, str(distance))
    ev3.screen.draw_text(0,60, str(round(x)))
    ev3.screen.draw_text(0,80, str(round(y)))
    ev3.screen.draw_text(0,100, str(round(theta * 180 / math.pi)))

    scale = 0.0034 if delta > 0 else 0.02

    left_factor = max(.7, min(1 - (delta * scale), 1.15)) 
    right_factor = max(.7, min(1 + (delta * scale), 1.15))
    
    ev3.screen.draw_text(0, 20, str(left_factor))
    ev3.screen.draw_text(0, 40, str(right_factor))

    left_motor.run(SPEED * left_factor)
    right_motor.run(SPEED * right_factor)
    
    vl = (left_motor.speed() * math.pi / 180) * RADIUS
    vr = (right_motor.speed() * math.pi / 180) * RADIUS

    if not vl == vr:
        omega = (vr - vl) / L
        R = L / 2 * (vl + vr) / (vr - vl) 

        ICCx = x - R * math.sin(theta)
        ICCy = y + R * math.cos(theta)

        x = (math.cos(omega * .1) * (x - ICCx)) + (-math.sin(omega * .1) * (y - ICCy)) + ICCx
        y = (math.sin(omega * .1) * (x - ICCx)) + (math.cos(omega * .1) * (y - ICCy)) + ICCy
        theta = -gyro.angle() * math.pi / 180

    else:
        x = x + vl * math.cos(theta) * .1
        y = y + vl * math.sin(theta) * .1

    distance = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
    cycle += 1
    wait(100)

ev3.speaker.beep(1000, 500)

run_angle(left_motor, right_motor, SPEED, 150)

turn_angle(left_motor, right_motor, gyro, -90)

run_angle(left_motor, right_motor, 400, -INITIAL_DISTANCE + 275)

left_motor.hold()
right_motor.hold()

ev3.speaker.beep(1000, 500)
