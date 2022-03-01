#!/usr/bin/env pybricks-micropython

# from pybricks.tools import wait

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
