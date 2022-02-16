#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait

ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
ultrasonic = UltrasonicSensor(Port.S1)
touch = TouchSensor(Port.S2)


# PART 1
while(not Button.CENTER in ev3.buttons.pressed()):
    wait(100)

DISTANCE = 2455.525
left_motor.run_target(500, DISTANCE, wait=False)
right_motor.run_target(500, DISTANCE, wait=False)

ev3.speaker.beep(1000, 500)


# PART 2
while(not Button.CENTER in ev3.buttons.pressed()):
    wait(100)

left_motor.run(250)
right_motor.run(250)

while(ultrasonic.distance() > 525):
    wait(100)

left_motor.hold()
right_motor.hold()

ev3.speaker.beep(1000, 500)


# PART 3
while(not Button.CENTER in ev3.buttons.pressed()):
    wait(100)

left_motor.run(100)
right_motor.run(100)

while(not touch.pressed()):
    wait(100)

left_motor.run(-200)
right_motor.run(-200)

while(ultrasonic.distance() < 525):
    wait(100)

left_motor.hold()
right_motor.hold()

ev3.speaker.beep(1000, 500)