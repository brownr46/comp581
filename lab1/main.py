# RYAN BROWN: 730320130
# BETHANY NEWCOMB: 730311643

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

DISTANCE = 2557
left_motor.run_target(350, DISTANCE, wait=False)
right_motor.run_target(350, DISTANCE, wait=True)

ev3.speaker.beep(1000, 500)


# PART 2
while(not Button.CENTER in ev3.buttons.pressed()):
    wait(100)

left_motor.run(150)
right_motor.run(150)

while(ultrasonic.distance() > 537.5):
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

while(ultrasonic.distance() < 517.5):
    wait(100)

left_motor.hold()
right_motor.hold()

ev3.speaker.beep(1000, 500)