#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
#
# Before runing the program for the first time:
#   Please complie lx16a.c to .so file first!
#   You can use the command as:  gcc -g -o lx16a.so -shared -fPIC lx16a.c
#   The name of lx16a.so can be changed as you like, remember to
#   modify the value 'lib'.
#   If you don't modify lx16a.c and lx16a.h, there's no need to compile any more.
#
#   Also highly recommend the user to read the protocol file of the servo motor
#   very carefully before using this code.
#
# Class Instruction:
#   1. After init the class, init the usb port first.
#
#   2. All the read/get functions will return the data you called, like readPosition(id = 1)
#      will return the current position of servo id = 1. If something wrong happen, the
#      python function will return False and the C function will print the error info.
#      Notice that different get functions might return different numbers of params.
#      Read the Class ServoMotor() carefully first.
#
#   3. After testing, each read command would take around 3.7ms on linux. So if the user find
#      it takes much longer than that on linux, please read USBLATENCY.txt file to
#      adjust the latency timer of linux.
#
#   4. After using setSpeed() function, the mode of lx16a would be set to motor mode. If
#      the user wants to use move() function, lx16a must be set to servo mode firstly, which
#      means to use setServoMode() before move().
#
################################################################################

# Author: Zhihao Zheng (Arthur)

import socket
import struct
import sys
import time
from ctypes import *

lib = CDLL("./lx16a.so")

class ServoMotor():
    def __init__(self, filename):
        self.filename = filename

    def IO_Init(self):
        IO1 = lib.IO_init(c_char_p(self.filename.encode('ascii')))
        return IO1

    def setServoID(self, id, new_id):
        lib.setServoID(id,new_id)

    def move(self, id, position, time):
        lib.move(id, position, time)

    def movePrepare(self, id, position, time):
        lib.movePrepare(id, position, time)

    def getPreparedMove(self, id):
        data = lib.getPreparedMove(id)
        pos = data >> 16
        time = data & 0xffff
        if pos > 1000 or data < 0:
            return False
        else:
            return pos, time

    def moveStart(self,id):
        lib.moveStart(id)

    def moveStop(self, id):
        lib.moveStop(id)

    def setPositionOffset(self, id, deviation):
        lib.setPositionOffset(id, deviation)

    def getPositionOffset(self, id):
        deviation = lib.getPositionOffset(id)
        if abs(deviation) > 125:
            return False
        else:
            return deviation

    def setPositionLimits(self, id, minPos, maxPos):
        lib.setPositionLimits(id, minPos, maxPos)

    def getPositionLimits(self, id):
        data = lib.getPositionLimits(id)
        minPos = data >> 16
        maxPos = data & 0xffff
        if minPos > 1000 or maxPos > 1000 or data < 0:
            return False
        else:
            return minPos, maxPos

    def savePositionOffset(self, id):
        lib.savePositionOffset(id)

    def setVoltageLimits(self, id, minVolt, maxVolt):
        lib.setVoltageLimits(id, minVolt, maxVolt)

    def getVoltageLimits(self, id):
        data = lib.getVoltageLimits(id)
        minVolt = data >> 16
        maxVolt = data & 0xffff
        if minVolt > 12500 or data < 0:
            return False
        else:
            return minVolt, maxVolt

    def setMaxTemp(self, id, temp):
        lib.setMaxTemp(id, temp)

    def getMaxTemp(self, id):
        temp = lib.getMaxTemp(id)
        if temp > 125 or temp < 0:
            return False
        else:
            return temp

    def getTemp(self, id):
        temp = lib.getTemp(id)
        if temp > 125 or temp < 0:
            return False
        else:
            return temp

    def getVoltage(self, id):
        vol = lib.getVoltage(id)
        if vol > 19000 or vol < 0:
            return False
        else:
            return vol

    def motorOn(self, id):
        lib.motorOn(id)

    def motorOff(self, id):
        lib.motorOff(id)

    def isMotorOn(self, id):
        status = lib.isMotorOn(id)
        if status > 1 or status < 0:
            return False
        else:
            return status

    def LEDOn(self, id):
        lib.setLED(id,1)

    def LEDOff(self, id):
        lib.setLED(id,0)

    def isLEDOn(self, id):
        status = lib.isLEDOn(id)
        if status > 1 or status < 0:
            return False
        else:
            return status

    def setLEDErrors(self, id, error):
        lib.setLEDErrors(id, error)

    def getLEDErrors(self, id):
        error = lib.getLEDErrors(id)
        if error > 7 or error < 0:
            return False
        else:
            return error

    def setServoMode(self, id):
        lib.setServoMode(id)

    def getMode(self, id):
        mode = lib.getMode(id)
        if mode > 1 or mode < 0:
            return False
        else:
            return mode

    def setSpeed(self, id, speed):
        lib.setSpeed(id, speed)

    def readSpeedSetting(self, id):
        speed = lib.getSpeedSetting(id)
        if abs(speed) > 1000:
            return False
        else:
            return speed

    def readPosition(self, id):
        position = lib.posRead(id)
        if abs(position) > 2000:
            return False
        else:
            return position

    pass



if __name__ == '__main__':
    print('start test!')

    filename = "/dev/ttyUSB0"
    motor = ServoMotor(filename)

    IO1 = motor.IO_Init()
    if IO1 < 0:
        sys.exit()

    pos = [0,0,0,0,0,0,0,0]
    spd = [0,0,0,0,0,0,0,0]

    minvol,maxvol = motor.getVoltageLimits(1)
    print('get vol limits: ', minvol,maxvol)

    motor.setSpeed(1,150)

    for i in range(1000):
        pos[1] = motor.readPosition(1)
        print('get run pos1: ', pos[1])

        pos2 = motor.readPosition(2)

        spd[1] = motor.readSpeedSetting(1)
        print('get run spd1: ', spd[1])

        vol1 = motor.getVoltage(1)
        print('get vol1: ', vol1)

        temp1 = motor.getTemp(1)
        print('get temp1: ', temp1)

    # motor.setSpeed(1, 0)
    #
    # for i in range(1000):
    #     pos[1] = motor.readPosition(1)
    #     print('get stop pos1: ', pos[1])
    #
    #     spd[1] = motor.readSpeedSetting(1)
    #     print('get stop spd1: ', spd[1])



    motor.setSpeed(1, 0)    #lib.setSpeed(1, 0)

    print('End!')
