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
import sys, glob # for listing serial ports
import time
import math

from threading import Thread
from typing import Callable
from itertools import cycle

# Create Library
import createlib as cl

try:
    import serial
except ImportError:
    tkinter.messagebox.showerror('Import error', 'Please install pyserial.')
    raise


TEXTWIDTH = 100 # window width, in characters
TEXTHEIGHT = 24 # window height, in lines

VELOCITYCHANGE = 200
ROTATIONCHANGE = 300

class TetheredDriveApp(Tk):
    # static variables for keyboard callback -- I know, this is icky
    callbackKeyUp = False
    callbackKeyDown = False
    callbackKeyLeft = False
    callbackKeyRight = False
    callbackKeyLastDriveCommand = ''

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
                        "B": "Print Sensors",
                        "W": "Find wall", # Task 1
                        "T": "Turn", # Task 1
                        "X": "Follow walls" # Task 2
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
        def require_robot(self,*args, **kwargs):
            if self.robot is None:
                tkinter.messagebox.showinfo('Error', "Robot Not Connected!")
                return
            foo(self, *args, **kwargs)
        return require_robot


    def __init__(self):
        Tk.__init__(self)
        self.title("iRobot Create 2 Tethered Drive")
        self.option_add('*tearOff', FALSE)

        self.menubar = Menu()
        self.configure(menu=self.menubar)

        createMenu = Menu(self.menubar, tearoff=False)
        self.menubar.add_cascade(label="Create", menu=createMenu)

        createMenu.add_command(label="Connect", command=self.onConnect)
        createMenu.add_command(label="Help", command=self.onHelp)
        createMenu.add_command(label="Quit", command=self.onQuit)

        self.text = Text(self, height = TEXTHEIGHT, width = TEXTWIDTH, wrap = WORD)
        self.scroll = Scrollbar(self, command=self.text.yview)
        self.text.configure(yscrollcommand=self.scroll.set)
        self.text.pack(side=LEFT, fill=BOTH, expand=True)
        self.scroll.pack(side=RIGHT, fill=Y)

        self.text.insert(END, self.help_text(self.supported_keys))

        self.bind("<Key>", self.callbackKey)
        self.bind("<KeyRelease>", self.callbackKey)

        self.ledThread = cl.RepeatTimer(1, self.ledToggle, autostart=False)
        self.ledRunning = False
        self.ledStatus = False
        
        #self.driveThread = self.driveThread = Thread(target=self.driveBumpWheeldrop)
        self.driveThread = Thread(target=self.driveBumpWheeldrop)
        self.driving = False

        self.sensorDelay = .1 # delay to use everywhere sensors are read

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

        if event.type == '2': # KeyPress; need to figure out how to get constant
            if k == 'P':   # Passive
                print("Passive")
                self.robot.start()
            elif k == 'S': # Safe
                print("Safe")
                self.robot.safe()
            elif k == 'F': # Full
                print("Full")
                self.robot.full()
            elif k == 'C': # Clean
                self.robot.clean()
            elif k == 'D': # Dock
                self.robot.dock()
            elif k == 'SPACE': # Beep
                # self.sendCommandASCII('140 3 1 64 16 141 3')
                # setup beep as song 3
                beep_song = [64, 16]
                self.robot.createSong(3, beep_song)
                self.robot.playSong(3)
            elif k == 'R': # Reset
                self.robot.reset()
            elif k == 'B': # Print Sensors
                sensors = self.robot.get_sensors()
                sensor_str = self.prettyPrint(sensors)
                print(sensor_str)
            elif k == 'UP':
                self.callbackKeyUp = True
                motionChange = True
            elif k == 'DOWN':
                self.callbackKeyDown = True
                motionChange = True
            elif k == 'LEFT':
                self.callbackKeyLeft = True
                motionChange = True
            elif k == 'RIGHT':
                self.callbackKeyRight = True
                motionChange = True
            elif k == 'ESCAPE':
                if self.robot is not None:
                    del self.robot
                self.destroy()
            elif k == 'W':
                # Find wall
                print("Find wall")
                self.driveLightBumper()
            elif k == 'T':
                # Rotate 90 degrees
                self.rotate_until(100, 85)
            elif k == 'X':
                # Follow wall
                print("Follow wall")
                self.wall_follow_pid()
            else:
                print("not handled", repr(k))
        elif event.type == '3': # KeyRelease; need to figure out how to get constant
            if k == 'UP':
                self.callbackKeyUp = False
                motionChange = True
            elif k == 'DOWN':
                self.callbackKeyDown = False
                motionChange = True
            elif k == 'LEFT':
                self.callbackKeyLeft = False
                motionChange = True
            elif k == 'RIGHT':
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
            vr = int(velocity + (rotation/2))
            vl = int(velocity - (rotation/2))

            # create drive command

            cmd = (vl, vr)
            if cmd != self.callbackKeyLastDriveCommand:
                self.robot.drive_direct(vl, vr)
                self.callbackKeyLastDriveCommand = cmd

    def onConnect(self):
        if self.robot is not None:
            tkinter.messagebox.showinfo('Oops', "You're already connected to the robot!")
            return

        try:
            ports = self.getSerialPorts()
            port = tkinter.simpledialog.askstring('Port?', 'Enter COM port to open.\nAvailable options:\n' + '\n'.join(ports))
        except EnvironmentError:
            port = tkinter.simpledialog.askstring('Port?', 'Enter COM port to open.')

        if port is not None:
            print("Trying " + str(port) + "... ")
            try:
                self.robot = cl.Create2(port=port, baud=115200)
                print("Connected!")
                tkinter.messagebox.showinfo('Connected', "Connection succeeded!")
            except Exception as e:
                print(f"Failed. Exception - {e}")
                tkinter.messagebox.showinfo('Failed', "Sorry, couldn't connect to " + str(port))


    def onHelp(self):
        """
        Display help text
        """
        tkinter.messagebox.showinfo('Help', self.help_text(self.supported_keys))

    def onQuit(self):
        """
        Confirm whether the user wants to quit.
        """
        if tkinter.messagebox.askyesno('Really?', 'Are you sure you want to quit?'):
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
        if sys.platform.startswith('win'):
            ports = ['COM' + str(i + 1) for i in range(256)]

        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this is to exclude your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')

        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')

        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result



    # ----------------------- Custom functions ------------------------------
    def checkSensors(self):
        sensors = self.robot.get_sensors()
        print(self.prettyPrint(sensors))

        checkbit = lambda bit : 'Yes' if (bit & 1) == 1 else 'No'
        tkinter.messagebox.showinfo(
            "Wall and Cliff Sensors", 
            f"Wall: {checkbit(sensors.wall)}\nCliff left: {checkbit(sensors.cliff_left)}\nCliff front left: {checkbit(sensors.cliff_front_left)}\nCliff front right: {checkbit(sensors.cliff_front_right)}\nCliff right: {checkbit(sensors.cliff_right)}\n")
        tkinter.messagebox.showinfo(
            "Battery Information", 
            f"Charger state: {cl.CHARGING_STATE(sensors.charger_state).name}\nVoltage: {sensors.voltage} mV\nTemperature: {sensors.temperature} C\nCurrent: {sensors.current} mA\nBattery Charge: {sensors.battery_charge} mAh\nBattery Capacity: {sensors.battery_capacity} mAh")

    def handleLED(self):
        if self.ledRunning:
            self.ledThread.stop()
            self.ledRunning=False
        else:
            self.ledThread.start()
            self.ledRunning=True
    
    def ledToggle(self): 
        if self.ledStatus==True:
            self.robot.led(led_bits=6, power_color=255, power_intensity=0)
            self.ledStatus=False
        else:
            self.robot.led(led_bits=9, power_color=255, power_intensity=255)
            self.ledStatus=True

    def handleDrive(self, driveType):
        if self.driving:
            self.robot.drive_stop()
            #self.driveThread.join()
            self.driving = False
        elif driveType == "W":
            #self.driveThread = Thread(target=self.driveBumpWheeldrop)
            #self.driveThread.start()
            self.driving = True
            self.driveBumpWheeldrop()
        elif driveType == "P":
            #self.driveThread = Thread(target=self.driveLightBumper)
            #self.driveThread.start()
            self.driveLightBumper()
            self.driving = True

    def driveBumpWheeldrop(self):
        vr = int(200)
        vl = int(200)
        self.driving = True
        self.robot.drive_direct(vl, vr)
        while self.driving:
            sensors = self.robot.get_sensors()
            if bump_or_wheeldrop(sensors=sensors):
                print("Bump or wheeldrop")
                self.robot.drive_stop()
                self.driving = False
                break
            time.sleep(self.sensorDelay)
    
    def driveLightBumper(self):
        #light_threshold = 10
        vr = int(200)
        vl = int(200)
        self.driving = True
        self.robot.drive_direct(vl, vr)
        while self.driving:
            sensors = self.robot.get_sensors()
            if light_bumper(sensors=sensors):
                print("Light bumper hit")
                self.robot.drive_stop()
                self.driving = False
                break
            time.sleep(self.sensorDelay)
        

    def goTheDistance(self, velocity, distance=200):
        """goTheDistance
        Drives the robot until it has traveled the requested distance or it detects a bump or wheeldrop  

        Args:
            velocity (int): robot's velocity in mm/s
            distance (int): requested distance in mm
        """

        stats = self.drive_straight_until(velocity, distance, stop_condition=bump_or_wheeldrop)

        # Print current values for traveled and elapsed
        print(f"Robot drove {stats[0]}mm in {stats[1]} seconds.")
    
    def rotate_until(self, velocity: int, degrees=-1, stop_condition: Callable[[cl.Sensors], bool]=None):
        """rotate_until
        Rotates the robot until xit reaches the stop_condition or has turned the specified degrees  

        Args:
            velocity (int): wheel velocity in mm/s
            distance (int): distance limit in mm (only for same wheel speed)
            stop_condition (Callable(self) -> bool): robot will stop driving when this condition is true
        
        Returns:
            Distance travelled in mm
        """
        print(f"Rotating {degrees} degrees at {velocity}mm/s")

        diameter = 235 # Create 2 wheel diameter
        circumference = diameter * math.pi

        distance = circumference * (degrees/360)
        return self.drive_until(l_vel=velocity, r_vel=-velocity, distance=distance, stop_condition=stop_condition)

# LIGHT BUMP SENSOR READINGS
# Farthest reading was just center left hit first with a sensor value of 178
# At a better distance:

    def reverse_drive(self, distance=50):
        # l_vel = -75
        # r_vel = -100
        l_vel = -15
        r_vel = -20
        self.drive_until(l_vel=l_vel, r_vel=r_vel, distance=distance)   

    def drive_straight_until(self, velocity=200, distance=-1, stop_condition: Callable[[cl.Sensors], bool]=None):
        """drive_straight_until
        Drives the robot straight until it reaches the stop_condition or has travelled the specified distance in mm  

        Args:
            velocity (int): wheel velocity in mm/s
            distance (int): distance limit in mm (only for same wheel speed)
            stop_condition (Callable(self) -> bool): robot will stop driving when this condition is true
        
        Returns:
            Distance travelled in mm
        """
        print(f"Driving {distance}mm at {velocity}mm/s")
        return self.drive_until(velocity, velocity, distance, stop_condition)

    def drive_until(self, l_vel, r_vel, distance=-1, stop_condition: Callable[[cl.Sensors], bool]=None):
        """drive_until
        Drives the robot until it reaches the stop_condition or has travelled the specified distance in mm  

        Args:
            l_vel (int): left wheel velocity in mm/s
            r_vel (int): right wheel velocity in mm/s
            distance (int): distance limit in mm (only for same wheel speed)
            stop_condition (Callable(self) -> bool): robot will stop driving when this condition is true
        
        Returns:
            Distance travelled in mm
        """

        # Initialized distance traveled
        traveled = 0

        # Start drive
        self.robot.drive_direct(l_vel,r_vel)
        self.driving = True

        # Initialize timing
        startTime = time.perf_counter() # returns time in seconds
        elapsed = 0 # initialize elapsed time

        d_vel = abs(l_vel) if abs(l_vel) >= abs(r_vel) else abs(r_vel)

        while (traveled < distance):
            # Check traveled distance no matter what so elapsed time and traveled distance are accurate
            checkTime = time.perf_counter() # get what time it is
            # difference between checkTime and startTime is how much time has passed
            elapsed = checkTime - startTime # update elapsed time
            traveled = d_vel * elapsed # update traveled distance
            # print(f"Current Distance: {traveled}\n")

            # Exit loop if there has been a bump or wheeldrop
            if stop_condition != None:
                sensors = self.robot.get_sensors()
                if stop_condition(sensors):
                    break
                # Apply the sensor delay before next iteration
                time.sleep(self.sensorDelay)
            

        # Stop driving
        self.robot.drive_stop()
        self.driving = False

        # Apply the sensor delay before returning
        time.sleep(self.sensorDelay)

        # Print current values for traveled and elapsed
        print(f"Robot drove {traveled:0.1f}mm in {elapsed:0.1f} seconds.")

        # Handle the result
        if traveled < distance:
            return self.drive_until(l_vel=l_vel, r_vel=r_vel, distance=(distance-traveled), stop_condition=stop_condition)
        
        return (traveled, elapsed)

    def dockRobot(self):
        #sensors needed are ir_opcode_right and ir_opcode_left
        sensors = self.robot.get_sensors()
        #back up
        #self.reverse_drive(distance=100)
        #turn right
        self.rotate_until(100, 90)
        #drive
        self.drive_until(l_vel=100, r_vel=100, distance=500)
        #spin right
        #while(sensors.ir_opcode_left != 255 & sensors.ir_opcode_right != 255):
        #    self.rotate_until(100, 45)
        #    if(sensors.ir_opcode_left == 255 | sensors.ir_opcode_right == 255):
        #        self.robot.drive_stop()
        #        self.drive_until(l_vel=50, r_vel=50, distance=200)
        self.rotate_until(100, 270)
        #drive
        self.drive_until(l_vel=100, r_vel=100, distance=100)
        #spin right
        self.rotate_until(100, 270)
        #drive into dock
        #probably need to create another pid controller for docking
        self.drive_until(l_vel=100, r_vel=100, distance=200)
    
    # A PID implementation for following a wall
    # Input to the PID formula is the combined error from all of the left-facing light bumper sensors
    def wall_follow_pid(self):
        # "normal_velocity" is the default speed to go if the robot is moving straight forward
        normal_velocity=100

        self.driving = True

        # Set the number of errors to store
        # The error array is created as this size
        array_size = 10

        # Create array to store errors 
        error_array = [None] * 10

        # and endless stream of 0-9, 0-9, 0-9, etc
        index_circle = cycle(range(array_size))

        # Initialize timing
        startTime = time.perf_counter() # returns time in seconds

        for n in index_circle:
            # Read the sensors
            sensors = self.robot.get_sensors()

            # Check traveled distance no matter what so elapsed time and traveled distance are accurate
            checkTime = time.perf_counter() # get what time it is
            # difference between checkTime and startTime is how much time has passed
            delta_t = checkTime - startTime # update elapsed time

            # End on wheeldrop
            if wheeldrop(sensors):
                break

            # Back up if there is a bump
            if bump(sensors):
                print("Collision detected, reversing")
                self.robot.drive_stop() # stop driving
                # time.sleep(self.sensorDelay)
                self.reverse_drive(distance=200) # back away from the wall
                continue
                # continue to next iteration so sensors are refreshed

            #initiate dock if detected
            if ir_threshold(160, [sensors.ir_opcode_left, sensors.ir_opcode_right]):
                print("Dock detected. Starting Dock")
                #self.dockRobot()

            # Calculate the error
            # Negative error: light reading was too low, turn towards the wall (left)
            # Positive error: light reading was too high, turn away from the wall (right)
            error_n = calc_error(sensors)
            
            # Store the current error
            error_array[n] = error_n

            # Use remainder so it circles
            n_minus_1 = (n - 1) % array_size

            error_n_minus_1 = error_array[n_minus_1]
            if error_n_minus_1 is None:
                # This case should only happen on the first iteration
                # n-1 will be -1 which will
                error_n_minus_1 = 0

            # The big formula for PID that evaluates to "output" on his board
            Kp = 1
            Ki = 1
            Kd = 1
            #deltaT = 1

            proportional = Kp * error_n
            integral = Ki * sum(filter(None, error_array))
            derivative = Kd * (error_n_minus_1 - error_n)
            output = proportional + integral + derivative
            # output = (Kp * error_n) + (Ki * (sum(error_array))) + (Kd * (prev_Error - error_n)) 
            print(f"PID: error={error_n:5} --> out={output:5}")

            # Represents the speed difference for each wheel
            # It is applied to each wheel's speed oppositely
            # Deviation is applied positively to the left wheel,
            #   so visualize it as the change to the left wheel speed
            # Add deviation to one wheel's velocity,
            #  subtract it from the other
            # RIGHT TURN: left wheel speeds up, right wheel slows down
            # LEFT TURN: left wheel slows down, right wheel speeds up
            left_vel = normal_velocity
            right_vel = normal_velocity

            # We need to figure out the output ranges to map to "turn left" and "turn right"
            if total_light(sensors) < 50:
                print(f"Away from wall (total_light: {total_light(sensors)})")
                left_vel = normal_velocity
                right_vel = normal_velocity
            if output < -100000:
                print("Sharp left")
                # left_vel = -normal_velocity
                # right_vel = normal_velocity
                left_vel = 10
                right_vel = 100
            elif in_range(output, -1000, -50): # TODO: determine the correct range for turning left
                print("Turn left")
                left_vel = normal_velocity - 30
                right_vel = normal_velocity + 30
            elif in_range(output, -10, 3000): # TODO: determine the correct range for turning right
                print("Turn right")
                left_vel = normal_velocity + 30
                right_vel = normal_velocity - 30
            elif output >= 3000: # TODO: determine the correct range for turning right
                print("Sharp right")
                # left_vel = normal_velocity
                # right_vel = -normal_velocity
                left_vel = 100
                right_vel = 10

            print(f"Drive: R={left_vel:3} L={right_vel:3}")

            self.robot.drive_direct(left_vel, right_vel)

            # Apply the sensor delay before next iteration
            time.sleep(self.sensorDelay)

        print("Stopping wall follow")
        self.robot.drive_stop()
        self.driving = False  

# each sensor range is a tuple: (range_start, range_stop)
light_ranges = {
    # The "right" sensors are not used since our robot follows using its left side
    # 'right': (0, 1200),
    # 'front_right': (0, 1200),
    # 'center_right': (0, 1200),

    # The left sensors have a "happy" range designed to keep the robot driving parallel
    # Once any sensor is outside its "happy" range, the robot will start adjusting
    
    # If this gets too high it means the robot is head on and needs to turn
    # The low of the range is 0, which happens when the robot is going straight
    'center_left': (0, 500),

    # If this gets too high it means the robot is head on and needs to turn
    # The low of the range is very low, which happens when the robot is going straight
    'front_left': (5, 1000),
    
    # If this gets too high it means the robot is too close to the wall and needs to adjust away
    # If this gets too low it means the robot is moving away from a convex corner,
    #   which means the robot needs to turn towards the corner
    'left': (1000, 15000)
}

# Calculates error for each left side light bumper sensor
# Uses the "happy" ranges defined in the dict above
def calc_error(sensors):
    # TODO: modify this per-sensor?
    weighted = lambda e: e*10 if e < 0 else e

    right = sensors.light_bumper_right
    # right_e = range_error(right, light_ranges['right'])
    right_e = 0
    right_w = 0
    
    front_right = sensors.light_bumper_front_right
    # front_right_e = range_error(front_right, light_ranges['front_right'])
    front_right_e = 0
    front_right_w = 0
    
    center_right = sensors.light_bumper_center_right
    # center_right_e = range_error(center_right, light_ranges['center_right'])
    center_right_e = 0
    center_right_w = 0
    
    center_left = sensors.light_bumper_center_left
    center_left_e = range_error(center_left, light_ranges['center_left'])
    center_left_w = weighted(center_left_e)

    front_left = sensors.light_bumper_front_left
    front_left_e = range_error(front_left, light_ranges['front_left'])
    front_left_w = weighted(front_left_e)

    left = sensors.light_bumper_left
    left_e = range_error(left, light_ranges['left'])
    left_w = weighted(left_e)

    total_e = left_e + front_left_e + center_left_e
    total_w = center_left_w + front_left_w + left_w

    print(f"Sensors  = {left:5} {front_left:5} {center_left:5} {center_right:5} {front_right:5} {right:5}")
    print(f"Error    = {left_e:5} {front_left_e:5} {center_left_e:5} {center_right_e:5} {front_right_e:5} {right_e:5} = {total_e:5}")
    print(f"Weighted = {left_w:5} {front_left_w:5} {center_left_w:5} {center_right_w:5} {front_right_w:5} {right_w:5} = {total_w:5}")
    # print(f"Error    = {right_e:5} {front_right_e:5} {center_right_e:5} {center_left_e:5} {front_left_e:5} {left_e:5} = {total_e:5}")
    # print(f"Weighted = {right_w:5} {front_right_w:5} {center_right_w:5} {center_left_w:5} {front_left_w:5} {left_w:5} = {total_w:5}")
    # print(f"Weighted = {right_w:5} {front_right_w:5} {center_right_w:5} {center_left_w:5} {front_left_w:5} {left_w:5} = {total_w:5}")

    return total_w #TODO: return total_e or total_w?

def total_light(sensors):
    right = sensors.light_bumper_right
    front_right = sensors.light_bumper_front_right
    center_right = sensors.light_bumper_center_right
    center_left = sensors.light_bumper_center_left
    front_left = sensors.light_bumper_front_left
    left = sensors.light_bumper_left
    return right + front_right + center_right + center_left + front_left + left

def calc_error_two(sensors):
    center_left = sensors.light_bumper_center_left
    front_left = sensors.light_bumper_front_left
    left = sensors.light_bumper_left

    total_error = center_left + front_left + left

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
        print("under range start")
        return -(range_start-value)
    elif value > range_stop:
        print("over range stop")
        return value - range_stop
    else:
        return 0

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

def bump_or_wheeldrop(sensors: cl.Sensors):
    wl = sensors.bumps_wheeldrops.wheeldrop_left
    wr = sensors.bumps_wheeldrops.wheeldrop_right
    bl = sensors.bumps_wheeldrops.bump_left
    br = sensors.bumps_wheeldrops.bump_right
    return wl | wr | bl | br

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
    
    # return any_greater_than(threshold=light_threshold, list=[lr, lfr, lcr, lcl, lfl, ll]):
    return lr | lfr | lcr | lcl | lfl | ll

# ----------------------- Main Driver ------------------------------
if __name__ == "__main__":
    app = TetheredDriveApp()
    app.mainloop()