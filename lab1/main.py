#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
# Initialize the EV3 brick.
ev3 = EV3Brick()
# Initialize a motor at port B.

left_motor = Motor(Port.A)
right_motor = Motor(Port.B)


# Run the motor up to 500 degrees per second. To a target angle of 90 degrees.
left_motor.run_target(500, 1000, wait=False)
right_motor.run_target(500, 1000, wait=False)
# Play another beep sound.
ev3.speaker.beep(1000, 500)