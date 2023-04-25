#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# This is a dumb pseudocode version of what his PID code on the whiteboard showed
def pid():
    # The default speed to go if the robot is moving straight forward
    normal = 100  # mm/s

    # Represents the speed difference for each wheel
    # It is applied to each wheel's speed oppositely
    deviation = 0  # mm/s

    # The big formula for PID that evaluates to "output" on his board
    output = 0

    # We need to figure out the output ranges to map to "turn left" and "turn right"
    if in_range(output, 0, 1):
        deviation = 5
    elif in_range(output, 1, 2):
        deviation = -5

    # Add deviation to one wheel's velocity,
    #  subtract it from the other
    robot.drive(normal + deviation, normal - deviation)


# Will need to determine the ranges for sensors and the output is a matrix, 
#   which would require a more complex function here
def in_range(output, range_start, range_stop):
    return range_start <= output <= range_stop
