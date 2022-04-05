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

def run_angle(left, right, speed, angle):
    left.run_angle(speed, angle, wait=False)
    right.run_angle(speed, angle, wait=True)

def distance_to_line(x, y, a=-2.25, b=1, c=0):
    return abs(a*x + b*y + c) / math.sqrt(math.pow(a, 2) + math.pow(b, 2))

def distance_to_goal(x, y):
    return math.sqrt(math.pow((80 - x), 2) + math.pow(180 - y, 2))

def can_turn_to_goal(x, y, theta): # TODO: FIX
    return theta > 1.15272351794


ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
ultrasonic = UltrasonicSensor(Port.S1)
touch = TouchSensor(Port.S2)
gyro = GyroSensor(Port.S3)


while(not Button.CENTER in ev3.buttons.pressed()):
    wait(100)

SET_POINT = 220
SPEED = 150
L = 12
RADIUS = 33.26

x = y = 0
theta = math.pi / 2
hit_point_distance = distance_from_goal = float('inf')
on_line = True

left_motor.reset_angle(0)
right_motor.reset_angle(0)
gyro.reset_angle(-90)

turn_angle(left_motor, right_motor, gyro, 24)

left_motor.run(SPEED)
right_motor.run(SPEED)

while (distance_from_goal > 3):
    if Button.CENTER in ev3.buttons.pressed():
        break

    distance = ultrasonic.distance()

    if not on_line and distance_to_line(x, y) < 2 and can_turn_to_goal(x, y, theta) and hit_point_distance - distance_from_goal > 5:
        turn_angle(left_motor, right_motor, gyro, theta * 180 / math.pi - 66)

        left_motor.run(SPEED)
        right_motor.run(SPEED)

        on_line = True        

    if touch.pressed() or (distance < SET_POINT and on_line):
        if touch.pressed():
            run_angle(left_motor, right_motor, 200, -250)

            x = x - 12.2 * math.cos(theta)
            y = y - 12.2 * math.sin(theta)

            turn_angle(left_motor, right_motor, gyro, 75)
            theta = -gyro.angle() * math.pi / 180

        hit_point_distance = distance_to_goal(x, y)
        on_line = False

    ev3.screen.clear()
    ev3.screen.draw_text(0,0, str(distance))
    ev3.screen.draw_text(0,60, str(round(x)))
    ev3.screen.draw_text(0,80, str(round(y)))
    ev3.screen.draw_text(0,100, str(round(theta * 180 / math.pi)))
    ev3.screen.draw_text(50, 60, str(round(distance_to_line(x,y))))
    ev3.screen.draw_text(50, 80, str(round(distance_from_goal)))
    ev3.screen.draw_text(50, 100, str(on_line))

    if not on_line:
        delta = distance - SET_POINT

        scale = 0.0034 if delta > 0 else 0.02

        left_factor = max(.7, min(1 - (delta * scale), 1.15)) 
        right_factor = max(.7, min(1 + (delta * scale), 1.15))
        
        ev3.screen.draw_text(0, 20, str(round(left_factor, 2)))
        ev3.screen.draw_text(0, 40, str(round(right_factor, 2)))

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

    distance_from_goal = distance_to_goal(x, y)
    wait(100)

left_motor.hold()
right_motor.hold()

ev3.speaker.beep(1000, 500)
