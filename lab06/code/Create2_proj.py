#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Version History
# v1.0: Python2.7 -- 2015//05/27
# v2.0: Update to Python3 -- 2020/04/01
# v2.1: Stiffler (bare) modifications -- 2022/02/02
# v3.0: Stiffler Quality of Life changes

###########################################################################
# Copyright (c) 2015-2020 iRobot Corporation#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#
#   Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.
#
#   Neither the name of iRobot Corporation nor the names
#   of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written
#   permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
###########################################################################

from math import pi
from tkinter import *
import tkinter.messagebox
import tkinter.simpledialog

import struct
import sys, glob  # for listing serial ports
import time
import math

from threading import Thread
from typing import Callable
from itertools import cycle

import docking
import wallfollow
from pid import Controller

# Create Library
import createlib as cl


try:
    import serial
except ImportError:
    tkinter.messagebox.showerror("Import error", "Please install pyserial.")
    raise


TEXTWIDTH = 100  # window width, in characters
TEXTHEIGHT = 24  # window height, in lines

VELOCITYCHANGE = 200
ROTATIONCHANGE = 300


class TetheredDriveApp(Tk):
    # static variables for keyboard callback -- I know, this is icky
    callbackKeyUp = False
    callbackKeyDown = False
    callbackKeyLeft = False
    callbackKeyRight = False
    callbackKeyLastDriveCommand = ""

    # Initialize a "Robot" object
    robot = None

    # Mapping of the supported keys (User should modify this to reflect key callbacks)
    supported_keys = {
        "P": "Passive",
        "S": "Safe",
        "F": "Full",
        "C": "Clean",
        "D": "Dock",
        "R": "Reset",
        "Space": "Beep",
        "Arrows": "Motion",
        "Escape": "Quick Shutdown",
        "O": "Print Sensors",
        "B": "Check buoy sensors",
        "W": "Find wall",  # Task 1
        "T": "Turn",  # Task 1
        "X": "Follow walls",  # Task 2
    }

    # Project variables appear below this comment

    def help_text(self, key_dict):
        """
        Function that generates "help" based on the supplied Dictionary
        """
        ret_str = "Supported Keys:"
        for key, value in key_dict.items():
            ret_str += f"\n{key}\t{value}"
        ret_str += "\n\nIf nothing happens after you connect, try pressing 'P' and then 'S' to get into safe mode.\n"
        return ret_str

    def _decorator(foo):
        def require_robot(self, *args, **kwargs):
            if self.robot is None:
                tkinter.messagebox.showinfo("Error", "Robot Not Connected!")
                return
            foo(self, *args, **kwargs)

        return require_robot

    def __init__(self):
        Tk.__init__(self)
        self.title("iRobot Create 2 Tethered Drive")
        self.option_add("*tearOff", FALSE)

        self.menubar = Menu()
        self.configure(menu=self.menubar)

        createMenu = Menu(self.menubar, tearoff=False)
        self.menubar.add_cascade(label="Create", menu=createMenu)

        createMenu.add_command(label="Connect", command=self.onConnect)
        createMenu.add_command(label="Help", command=self.onHelp)
        createMenu.add_command(label="Quit", command=self.onQuit)

        self.text = Text(self, height=TEXTHEIGHT, width=TEXTWIDTH, wrap=WORD)
        self.scroll = Scrollbar(self, command=self.text.yview)
        self.text.configure(yscrollcommand=self.scroll.set)
        self.text.pack(side=LEFT, fill=BOTH, expand=True)
        self.scroll.pack(side=RIGHT, fill=Y)

        self.text.insert(END, self.help_text(self.supported_keys))

        self.bind("<Key>", self.callbackKey)
        self.bind("<KeyRelease>", self.callbackKey)

        self.sensorDelay = 0.1  # delay to use everywhere sensors are read

    def prettyPrint(self, sensors):
        str = f"{'-'*70}\n"
        str += f"{'Sensor':>40} | {'Value':<5}\n"
        str += f"{'-'*70}\n"
        for k, v in sensors._asdict().items():
            str += f"{k}: {v}\n"
        return str

    @_decorator
    def callbackKey(self, event):
        """
        A handler for keyboard events. Feel free to add more!
        """
        k = event.keysym.upper()
        motionChange = False

        if event.type == "2":  # KeyPress; need to figure out how to get constant
            if k == "P":  # Passive
                print("Passive")
                self.robot.start()
            elif k == "S":  # Safe
                print("Safe")
                self.robot.safe()
            elif k == "F":  # Full
                print("Full")
                self.robot.full()
            elif k == "C":  # Clean
                self.robot.clean()
            elif k == "D":  # Dock
                self.robot.dock()
            elif k == "SPACE":  # Beep
                # self.sendCommandASCII('140 3 1 64 16 141 3')
                # setup beep as song 3
                beep_song = [64, 16]
                self.robot.createSong(3, beep_song)
                self.robot.playSong(3)
            elif k == "R":  # Reset
                self.robot.reset()
            elif k == "O":  # Print Sensors
                sensors = self.robot.get_sensors()
                sensor_str = self.prettyPrint(sensors)
                print(sensor_str)
            elif k == "B":  # Print Buoy Sensors
                sensors = self.robot.get_sensors()
                ir = docking.get_dock600_opcodes(sensors)
                docking.prettyPrint(ir)
            elif k == "UP":
                self.callbackKeyUp = True
                motionChange = True
            elif k == "DOWN":
                self.callbackKeyDown = True
                motionChange = True
            elif k == "LEFT":
                self.callbackKeyLeft = True
                motionChange = True
            elif k == "RIGHT":
                self.callbackKeyRight = True
                motionChange = True
            elif k == "ESCAPE":
                if self.robot is not None:
                    del self.robot
                self.destroy()
            elif k == "W":
                # Find wall
                print("Find wall")
                self.driveLightBumper()
            elif k == "T":
                # Rotate 90 degrees
                self.rotate_until(100, 90)
            elif k == "X":
                # Follow wall
                print("Follow wall")
                self.drive_to_dock()
            else:
                print("not handled", repr(k))
        elif event.type == "3":  # KeyRelease; need to figure out how to get constant
            if k == "UP":
                self.callbackKeyUp = False
                motionChange = True
            elif k == "DOWN":
                self.callbackKeyDown = False
                motionChange = True
            elif k == "LEFT":
                self.callbackKeyLeft = False
                motionChange = True
            elif k == "RIGHT":
                self.callbackKeyRight = False
                motionChange = True

        if motionChange == True:
            velocity = 0
            velocity += VELOCITYCHANGE if self.callbackKeyUp is True else 0
            velocity -= VELOCITYCHANGE if self.callbackKeyDown is True else 0
            rotation = 0
            rotation += ROTATIONCHANGE if self.callbackKeyLeft is True else 0
            rotation -= ROTATIONCHANGE if self.callbackKeyRight is True else 0

            # compute left and right wheel velocities
            vr = int(velocity + (rotation / 2))
            vl = int(velocity - (rotation / 2))

            # create drive command

            cmd = (vl, vr)
            if cmd != self.callbackKeyLastDriveCommand:
                self.robot.drive_direct(vl, vr)
                self.callbackKeyLastDriveCommand = cmd

    def onConnect(self):
        if self.robot is not None:
            tkinter.messagebox.showinfo(
                "Oops", "You're already connected to the robot!"
            )
            return

        try:
            ports = self.getSerialPorts()
            port = tkinter.simpledialog.askstring(
                "Port?",
                "Enter COM port to open.\nAvailable options:\n" + "\n".join(ports),
            )
        except EnvironmentError:
            port = tkinter.simpledialog.askstring("Port?", "Enter COM port to open.")

        if port is not None:
            print("Trying " + str(port) + "... ")
            try:
                self.robot = cl.Create2(port=port, baud=115200)
                print("Connected!")
                tkinter.messagebox.showinfo("Connected", "Connection succeeded!")
            except Exception as e:
                print(f"Failed. Exception - {e}")
                tkinter.messagebox.showinfo(
                    "Failed", "Sorry, couldn't connect to " + str(port)
                )

    def onHelp(self):
        """
        Display help text
        """
        tkinter.messagebox.showinfo("Help", self.help_text(self.supported_keys))

    def onQuit(self):
        """
        Confirm whether the user wants to quit.
        """
        if tkinter.messagebox.askyesno("Really?", "Are you sure you want to quit?"):
            if self.robot is not None:
                print("Robot object deleted")
                del self.robot
            self.destroy()

    def getSerialPorts(self):
        """
        Lists serial ports
        From http://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of available serial ports
        """
        if sys.platform.startswith("win"):
            ports = ["COM" + str(i + 1) for i in range(256)]

        elif sys.platform.startswith("linux") or sys.platform.startswith("cygwin"):
            # this is to exclude your current terminal "/dev/tty"
            ports = glob.glob("/dev/tty[A-Za-z]*")

        elif sys.platform.startswith("darwin"):
            ports = glob.glob("/dev/tty.*")

        else:
            raise EnvironmentError("Unsupported platform")

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result

    # ----------------------- Our functions ------------------------------

    def rotate_until(
        self,
        velocity=100,
        stop_degrees=0,
        stop_condition: Callable[[cl.Sensors], bool] = None,
        persist=True,
    ):
        """rotate_until
        Rotates the robot until xit reaches the stop_condition or has turned the specified degrees

        Args:
            velocity (int): wheel velocity in mm/s
            distance (int): distance limit in mm (only for same wheel speed)
            stop_condition (Callable(self) -> bool): robot will stop driving when this condition is true

        Returns:
            Distance travelled in mm
        """
        print(f"Rotating {stop_degrees} degrees at {velocity}mm/s")

        diameter = 235  # Create 2 wheel diameter
        circumference = diameter * math.pi

        # Will be 0 if stop_degrees is 0
        distance = circumference * (abs(stop_degrees) / 360)

        l_vel = velocity
        r_vel = -velocity
        if stop_degrees < 0:
            l_vel = -velocity
            r_vel = velocity

        return self.drive_until(
            l_vel=l_vel,
            r_vel=r_vel,
            stop_distance=distance,
            stop_condition=stop_condition,
            persist=persist,
        )

    def reverse_drive(self, distance=50):
        l_vel = -50
        r_vel = -50
        self.drive_until(l_vel=l_vel, r_vel=r_vel, stop_distance=distance)

    def drive_straight_until(
        self,
        velocity=200,
        stop_distance=0,
        stop_condition: Callable[[cl.Sensors], bool] = None,
    ):
        """drive_straight_until
        Drives the robot straight until it reaches the stop_condition or has travelled the specified distance in mm

        Args:
            velocity (int): wheel velocity in mm/s
            distance (int): distance limit in mm (only for same wheel speed)
            stop_condition (Callable(self) -> bool): robot will stop driving when this condition is true

        Returns:
            Distance travelled in mm
        """
        print(f"Driving {stop_distance}mm at {velocity}mm/s")
        return self.drive_until(velocity, velocity, stop_distance, stop_condition)

    def drive_until(
        self,
        l_vel: int,
        r_vel: int,
        stop_distance=0,
        stop_condition: Callable[[cl.Sensors], bool] = None,
        persist=True,
    ):
        """drive_until
        Drives the robot until it reaches the stop_condition or has travelled the specified distance in mm

        Args:
            l_vel (int): left wheel velocity in mm/s
            r_vel (int): right wheel velocity in mm/s
            distance (int): distance limit in mm (only for same wheel speed)
            stop_condition (Callable(self) -> bool): robot will stop driving when this condition is true
            persist (bool): whether to keep attempting to drive after a stop_condition

        Returns:
            Distance travelled in mm
        """

        # Initialized distance traveled
        traveled = 0

        # Start drive
        self.robot.drive_direct(l_vel, r_vel)

        # Initialize timing
        startTime = time.perf_counter()  # returns time in seconds
        elapsed = 0  # initialize elapsed time

        # Velocity to use for distance calcuations
        distance_velocity = max(abs(l_vel), abs(r_vel))

        while True:
            # Check traveled distance no matter what so elapsed time and traveled distance are accurate
            checkTime = time.perf_counter()  # get what time it is
            # difference between checkTime and startTime is how much time has passed
            elapsed = checkTime - startTime  # update elapsed time
            traveled = distance_velocity * elapsed  # update traveled distance

            if (stop_distance != 0) & (traveled >= stop_distance):
                print(f"Drove distance limit {stop_distance}mm, stopping")
                break

            # Exit loop if there has been a bump or wheeldrop
            if stop_condition != None:
                # Apply the sensor delay before next iteration
                time.sleep(self.sensorDelay)
                sensors = self.robot.get_sensors()
                if stop_condition(sensors):
                    print("Stop condition met")
                    break

        # Stop driving
        self.robot.drive_stop()

        # Print current values for traveled and elapsed
        # print(f"Robot drove {traveled:0.1f}mm in {elapsed:0.1f} seconds.")

        # Finish the distance if requested
        if (persist) & (traveled < stop_distance):
            return self.drive_until(
                l_vel=l_vel,
                r_vel=r_vel,
                stop_distance=(stop_distance - traveled),
                stop_condition=stop_condition,
                persist=persist,
            )

        return (traveled, elapsed)

    def dockRobot(self):
        """dockRobot
        Docks the robot using a PID controller on the robot's IR sensors that detect the dock's buoys.
        Assumes the robot has just encountered the dock's force field
        """
        # Rotate away from wall
        print("Rotating away from the wall")
        self.rotate_until(stop_degrees=30)

        # Drive until the omni sensor sees far buoy
        def omni_sees_far_buoy(sensors: cl.Sensors):
            ir = docking.get_dock600_opcodes(sensors=sensors)
            if ir.omni.red_buoy:
                print("Omni sensor saw the far buoy, stopping")
            return ir.omni.red_buoy | bump_or_wheeldrop(sensors)

        self.drive_until(
            l_vel=100,
            r_vel=100,
            stop_distance=0,
            stop_condition=omni_sees_far_buoy,
            persist=False,
        )

        # Rotate until the right IR sensor sees the green buoy
        def right_sees_green(sensors: cl.Sensors):
            if bump_or_wheeldrop(sensors):
                # Cancel the whole procedure
                return True
            ir = docking.get_dock600_opcodes(sensors=sensors)
            if ir.right.green_buoy:
                print("Right sensor saw the green buoy, stopping rotation")
            return ir.right.green_buoy

        self.rotate_until(
            velocity=30,
            stop_degrees=-360,
            stop_condition=right_sees_green,
            persist=False,
        )

        pid = Controller(
            name="docking",
            # Kp=1,
            # Ki=1,
            # Kd=1,
            Kp=8,
            Ki=0,
            Kd=0,
        )

        # iterate unless robot dies
        while self.robot is not None:
            # Read the sensors
            time.sleep(self.sensorDelay)
            sensors = self.robot.get_sensors()

            # First things first check if we completed the docking
            if is_docked(sensors):
                break

            # End on wheeldrop
            if wheeldrop(sensors):
                break

            # Back up if there is a bump
            if bump(sensors):
                print("Collision detected, reversing")
                self.robot.drive_stop()  # stop driving
                self.drive_until(
                    l_vel=-10,
                    r_vel=-10,
                    stop_distance=100,
                    stop_condition=is_docked,
                    persist=False,
                )
                continue
                # continue to next iteration so sensors are refreshed

            # Get PID controller's output for the current error value
            output = pid.iterate(docking.error(sensors))

            # the default speed to go if the robot is moving straight forward
            normal_velocity = 20

            # Turn amount
            # deviation = int(self.robot.limit(3*output, -40, 40))
            deviation = int(self.robot.limit(output, -30, 30))

            # RIGHT TURN: left wheel speeds up, right wheel slows down
            # LEFT TURN: left wheel slows down, right wheel speeds up
            # left_vel = normal_velocity + deviation
            # right_vel = normal_velocity - deviation
            left_vel = deviation + 20
            right_vel = -deviation + 20

            # Trigger the drive with the updated velocities
            print(f"Drive: L={left_vel:3} R={right_vel:3}")
            self.robot.drive_direct(left_vel, right_vel)

            # Apply the sensor delay before next iteration
            time.sleep(self.sensorDelay)

        print("Exited the docking drive loop")
        self.robot.drive_stop()

    # A PID implementation for following a wall
    # Input to the PID formula is the combined error from all of the left-facing light bumper sensors
    def drive_to_dock(self):
        pid = Controller(
            name="wall-follow",
            # Kp=.1,
            # Ki=.01,
            # Kd=.1,
        )

        while self.robot is not None:
            # Read the sensors
            time.sleep(self.sensorDelay)
            sensors = self.robot.get_sensors()

            # End on wheeldrop
            if wheeldrop(sensors):
                break

            # Back up if there is a bump
            if bump(sensors):
                print("Collision detected, reversing")
                self.robot.drive_stop()  # stop driving
                # time.sleep(self.sensorDelay)
                self.reverse_drive(distance=100)  # back away from the wall
                continue
                # continue to next iteration so sensors are refreshed

            ir = docking.get_dock600_opcodes(sensors)
            if ir.omni.force_field | ir.left.force_field | ir.right.force_field:
                print("Dock detected. Starting docking maneuver...")
                return self.dockRobot()

            # Turn amount
            deviation = 30
            # "normal_velocity" is the default speed to go if the robot is moving straight forward
            normal_velocity = 80
            # RIGHT TURN: left wheel speeds up, right wheel slows down
            # LEFT TURN: left wheel slows down, right wheel speeds up
            left_vel = normal_velocity
            right_vel = normal_velocity

            total_light = check_light(sensors)
            # if total_light < 50:
            #     print(f"Away from wall")
            #     # Do this here so we don't add to the error array?
            #     left_vel = normal_velocity
            #     right_vel = normal_velocity
            #     print(f"Drive: R={left_vel:3} L={right_vel:3}")
            #     self.robot.drive_direct(left_vel, right_vel)
            #     continue

            # Calculate the error
            # Negative error: light reading was too low, turn towards the wall (left)
            # Positive error: light reading was too high, turn away from the wall (right)
            output = pid.iterate(wallfollow.error(sensors))

            left_bound = -1000
            middle_bound = -50
            right_bound = 3000

            # We need to figure out the output ranges to map to "turn left" and "turn right"
            # if total_light < 15:
            #     print(f"Away from wall")
            #     left_vel = normal_velocity
            #     right_vel = normal_velocity
            if output <= left_bound:
                # if output < -100000:
                print("Sharp left")
                # left_vel = -normal_velocity
                # right_vel = normal_velocity
                left_vel = 10
                right_vel = 50
            elif left_bound <= output <= middle_bound:
                # elif in_range(output, left_bound, middle_bound): # TODO: determine the correct range for turning left
                print("Turn left")
                left_vel = normal_velocity - deviation
                right_vel = normal_velocity + deviation
            elif middle_bound <= output <= right_bound:
                # elif in_range(output, middle_bound, right_bound): # TODO: determine the correct range for turning right
                print("Turn right")
                left_vel = normal_velocity + deviation
                right_vel = normal_velocity - deviation
            elif (
                right_bound <= output
            ):  # TODO: determine the correct range for turning right
                print("Sharp right")
                # left_vel = normal_velocity
                # right_vel = -normal_velocity
                left_vel = 50
                right_vel = -50

            print(f"Drive: R={left_vel:3} L={right_vel:3}")
            self.robot.drive_direct(left_vel, right_vel)
        print("Exited the wall follow drive loop")
        self.robot.drive_stop()


# TODO: edit this based on how the light sensors work, the logic may be backwards if the light
def any_greater_than(threshold, list):
    for i in list:
        if i > threshold:
            return False
    return True


def ir_threshold(threshold, list):
    for i in list:
        if i > threshold:
            return True
    return False


def is_docked(sensors: cl.Sensors):
    if (sensors.charger_state != cl.CHARGING_STATE.CHARGING_FAULT) & (
        sensors.charger_state != cl.CHARGING_STATE.NOT_CHARGING
    ):
        print(30 * "-" + "\nDocked!\n" + 30 * "-")
        return True
    return False


def stop_if_buoy(sensors: cl.Sensors):
    ir = docking.get_dock600_opcodes(sensors=sensors)
    stop = ir.omni.green_buoy or ir.omni.red_buoy
    if stop:
        print("Buoy detected!")
    return stop | bump_or_wheeldrop(sensors=sensors)


def bump_or_wheeldrop(sensors: cl.Sensors):
    return bump(sensors) | wheeldrop(sensors)


def bump(sensors: cl.Sensors):
    bl = sensors.bumps_wheeldrops.bump_left
    br = sensors.bumps_wheeldrops.bump_right
    return bl | br


def wheeldrop(sensors: cl.Sensors):
    wl = sensors.bumps_wheeldrops.wheeldrop_left
    wr = sensors.bumps_wheeldrops.wheeldrop_right
    return wl | wr


def light_bumper(sensors: cl.Sensors):
    lr = sensors.light_bumper.right
    lfr = sensors.light_bumper.front_right
    lcr = sensors.light_bumper.center_right
    lcl = sensors.light_bumper.center_left
    lfl = sensors.light_bumper.front_left
    ll = sensors.light_bumper.left
    return lr | lfr | lcr | lcl | lfl | ll


def check_light(sensors: cl.Sensors):
    right = sensors.light_bumper_right
    front_right = sensors.light_bumper_front_right
    center_right = sensors.light_bumper_center_right
    center_left = sensors.light_bumper_center_left
    front_left = sensors.light_bumper_front_left
    left = sensors.light_bumper_left

    total = left + front_left + center_left + center_right + front_right + right

    print(
        f"Light = {left:5} {front_left:5} {center_left:5} {center_right:5} {front_right:5} {right:5} total={total}"
    )
    return total


# ----------------------- Main Driver ------------------------------
if __name__ == "__main__":
    app = TetheredDriveApp()
    app.mainloop()
