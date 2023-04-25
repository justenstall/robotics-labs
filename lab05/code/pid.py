#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from itertools import cycle



# This is a dumb pseudocode version of what his PID code on the whiteboard showed
def pid():
    # The default speed to go if the robot is moving straight forward
    normal = 100  # mm/s

    # Set the number of errors to store
    # The error array is created as this size
    array_maxsize = 10

    # Create array to store errors 
    error_array = [None] * 10

    cycler = cycle(range(array_maxsize))

    for index in cycler:
        sensors = robot.get_sensors()
        error_n = calc_error(sensors)
        
        error_array[index] = error_n

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

def calc_error(sensors):
    # right = sensors.light_bumper.right
    # front_right = sensors.light_bumper.front_right
    # center_right = sensors.light_bumper.center_right
    center_left = sensors.light_bumper.center_left
    center_left_error = range_error(center_left, ranges['center_left'])

    front_left = sensors.light_bumper.front_left
    front_left_error = range_error(front_left, ranges['front_left'])

    left = sensors.light_bumper.left
    left_error = range_error(left, ranges['left'])

    total_error = center_left_error + front_left_error + left_error
    print("Total error: "+total_error)

    return total_error

# Will need to determine the ranges for sensors and the output is a matrix, 
#   which would require a more complex function here
def in_range(output, range_start, range_stop):
    return range_start <= output <= range_stop

# Calculate an error for "value" for the given range "range_tuple"
# "range_tuple": (range_start, range_stop)
def range_error(value, range_tuple):
    range_start = range_tuple[0]
    range_stop = range_tuple[1]
    if value < range_start:
        return -(range_start-value)
    elif value > range_stop:
        return value - range_stop
    else:
        return 0
