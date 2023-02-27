#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Version History
# v1.0: Python2.7 -- 2015//05/27
# v2.0: Update to Python3 -- 2020/04/01
# v2.1: Stiffler (bare) modifications -- 2022/02/02
# v2.0: Stiffler Quality of Life changes from CreateLib -- 2022/02/02

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

from tkinter import *
import tkinter.messagebox
import tkinter.simpledialog
import time

import struct
import sys, glob # for listing serial ports

try:
    import serial
except ImportError:
    tkinter.messagebox.showerror('Import error', 'Please install pyserial.')
    raise

connection = None

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

    # Mapping of the supported keys (User should modify this to reflect key callbacks)
    supported_keys = {
                        "P": "Passive",
                        "S": "Safe",
                        "F": "Full",
                        "C": "Clean",
                        "D": "Dock",
                        "R": "Reset",
                        "T": "Transmit sensor data", # Lab 02 Task 1
                        "L": "Lights", # Lab 02 Task 2
                        "Space": "Beep",
                        "Arrows": "Motion",
                        "Escape": "Quick Shutdown",
                        "B": "Bumps and Wheeldrops", # Lab 01
                     }

    def help_text(self, key_dict):
        """
        Function that generates "help" based on the supplied Dictionary
        """
        ret_str = "Supported Keys:"
        for key, value in key_dict.items():
            ret_str += f"\n{key}\t{value}"
        ret_str += "\n\nIf nothing happens after you connect, try pressing 'P' and then 'S' to get into safe mode.\n"
        return ret_str

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
        self.bumpWheelpkt = None

    def sendCommandASCII(self, command):
        """
        Takes a string of whitespace-separated, ASCII-encoded base 10 values then encodes
        them for transmission to the robot
        """
        cmd = bytes([int(v) for v in command.split()])
        self.sendCommandRaw(cmd)

    def sendCommandRaw(self, command):
        """
        Takes a string interpreted as a byte array and transmits it to the robot.
        """
        global connection

        try:
            if connection is not None:
                assert isinstance(command, bytes), 'Command must be of type bytes'
                connection.write(command)
                connection.flush()
            else:
                tkinter.messagebox.showerror('Not connected!', 'Not connected to a robot!')
                print("Not connected.")
        except serial.SerialException:
            print("Lost connection")
            tkinter.messagebox.showinfo('Uh-oh', "Lost connection to the robot!")
            connection = None

        seq = ' '.join([ str(c) for c in command ])
        self.text.insert(END, ' '.join([ str(c) for c in command ]))
        self.text.insert(END, '\n')
        self.text.see(END)

    def getDecodedBytes(self, n, fmt):
        """
        Returns a n-byte value decoded using a format string.
        Whether it blocks is based on how the connection was set up.
        """
        global connection

        try:
            return struct.unpack(fmt, connection.read(n))[0]
        except serial.SerialException:
            print("Lost connection")
            tkinter.messagebox.showinfo('Uh-oh', "Lost connection to the robot!")
            connection = None
            return None
        except struct.error:
            print("Got unexpected data from serial port.")
            return None

    def get8Unsigned(self):
        """
        Returns an 8-bit unsigned value.
        """
        return self.getDecodedBytes(1, "B")

    def get8Signed(self):
        """
        Returns an 8-bit signed value.
        """
        return self.getDecodedBytes(1, "b")

    def get16Unsigned(self):
        """
        Returns an 16-bit unsigned value.
        """
        return self.getDecodedBytes(2, ">H")

    def get16Signed(self):
        """
        Returns an 16-bit signed value.
        """
        return self.getDecodedBytes(2, ">h")

    def callbackKey(self, event):
        """
        A handler for keyboard events. Feel free to add more!
        """
        k = event.keysym.upper()
        motionChange = False

        if event.type == '2': # KeyPress; need to figure out how to get constant
            if k == 'P':   # Passive
                self.sendCommandASCII('128')
            elif k == 'S': # Safe
                self.sendCommandASCII('131')
            elif k == 'F': # Full
                self.sendCommandASCII('132')
            elif k == 'C': # Clean
                self.sendCommandASCII('135')
            elif k == 'D': # Dock
                self.sendCommandASCII('143')
            elif k == 'SPACE': # Beep
                self.sendCommandASCII('140 3 1 64 16 141 3')
            elif k == 'R': # Reset
                self.sendCommandASCII('7')
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
                self.destroy()
            # Lab 01
            elif k == 'B': # Bumps and Wheeldrops
                # Check bumps and wheeldrops
                self.sendCommandASCII('142 7') # 142 is sensor read, 7 is the packet ID of the BWD sensor
                time.sleep(0.15)
                #x = get sensor queried list
                self.bumpWheelpkt = self.get8Unsigned()
                time.sleep(0.15)
                print(f"Received value: {self.bumpWheelpkt}")
                checkbit = lambda i, yes, no : yes if ((self.bumpWheelpkt >> i) & 1) == 1 else no
                tkinter.messagebox.showinfo("Bumps and Wheel drops", f"Left wheel: {checkbit(3, 'Dropped', 'Raised')}\nRight wheel: {checkbit(2, 'Dropped', 'Raised')}\nLeft bumper: {checkbit(1, 'Bump', 'No bump')}\nRight bumper: {checkbit(0, 'Bump', 'No bump')}\n")
            elif k == 'T':
                # Wall signal
                self.sendCommandASCII('149 6 8 9 10 11 12 3')
                time.sleep(0.15)
                wall = self.get8Unsigned()
                cliffFL = self.get8Unsigned()
                cliffLeft = self.get8Unsigned()
                cliffFR = self.get8Unsigned()
                cliffRight = self.get8Unsigned()

                # Check packet group 3
                # 21: Charging State, 1 byte
                # 22: Voltage: 2 bytes
                # 23: Current: 2 bytes
                # 24: Temperature: 1 byte
                # 25: Battery Charge: 2 bytes
                # 26: Battery Capacity: 2 bytes
                chargeState = self.get8Unsigned()
                voltage = self.get16Unsigned()
                current = self.get16Signed()
                temp = self.get8Signed()
                charge = self.get16Unsigned()
                capacity = self.get16Unsigned()
                time.sleep(0.15)

                print(f"wall detected? {wall}")
                print(f"cliffs: {cliffLeft} {cliffFL} {cliffFR} {cliffRight}")

                print(f"charge state: {chargeState}")
                print(f"voltage: {voltage}")
                print(f"temp: {temp}")
                print(f"current: {current}")
                print(f"charge: {charge}")
                print(f"capacity: {capacity}")
            elif k == 'L':
                # Start a thread that loops these two indefinitely
                self.sendCommandASCII(f'139 {int(0b0110, 2)} 255 0')
                time.sleep(5)
                self.sendCommandASCII(f'139 {int(0b1001, 2)} 255 255')
                time.sleep(5)

                # Store state of lights, either on or off. If someone turns lights off, then kill the thread
            else:
                print("not handled", repr(k))
                #tkinter.messagebox.showinfo('Bumps and Wheeldrops', "info")
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
            cmd = struct.pack(">Bhh", 145, vr, vl)
            if cmd != self.callbackKeyLastDriveCommand:
                self.sendCommandRaw(cmd)
                self.callbackKeyLastDriveCommand = cmd

    def onConnect(self):
        """
        Handle the Serial Port Connection:
            + Determine if already connected.
            + If not, list available options and attempt to establish connection
        """
        global connection

        if connection is not None:
            tkinter.messagebox.showinfo('Oops', "You're already connected!")
            return

        try:
            ports = self.getSerialPorts()
            port = tkinter.simpledialog.askstring('Port?', 'Enter COM port to open.\nAvailable options:\n' + '\n'.join(ports))
        except EnvironmentError:
            port = tkinter.simpledialog.askstring('Port?', 'Enter COM port to open.')

        if port is not None:
            print("Trying " + str(port) + "... ")
            try:
                connection = serial.Serial(port, baudrate=115200, timeout=1)
                print("Connected!")
                tkinter.messagebox.showinfo('Connected', "Connection succeeded!")
            except:
                print("Failed.")
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

if __name__ == "__main__":
    app = TetheredDriveApp()
    app.mainloop()
