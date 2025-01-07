#!/usr/bin/env python3
import os
import sys
import time
import smbus
from smbus2 import SMBus, i2c_msg
#Ultrasonic Usage#

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

class Sonar:

    __units = {"mm":0, "cm":1}
    __dist_reg = 0

    __RGB_MODE = 2
    __RGB1_R = 3
    __RGB1_G = 4
    __RGB1_B = 5
    __RGB2_R = 6
    __RGB2_G = 7
    __RGB2_B = 8

    __RGB1_R_BREATHING_CYCLE = 9
    __RGB1_G_BREATHING_CYCLE = 10
    __RGB1_B_BREATHING_CYCLE = 11
    __RGB2_R_BREATHING_CYCLE = 12
    __RGB2_G_BREATHING_CYCLE = 13
    __RGB2_B_BREATHING_CYCLE = 14
    def __init__(self):
        self.i2c_addr = 0x77
        self.i2c = 1
        self.R1 = 0
        self.G1 = 0
        self.B1 = 0
        self.R2 = 0
        self.G2 = 0
        self.B2 = 0
        self.RGBMode = 0
        self.bus = SMBus(self.i2c)

    def __getattr(self, attr):
        if attr in self.__units:
            return self.__units[attr]
        if attr == "Distance":
            return self.getDistance()
        else:
            raise AttributeError('Unknow attribute : %s'%attr)
    
    #Set the RGB mode, 0: colored light mode (single static color), 1: breathing light mode (pulsates or cycles through colors)
    def setRGBMode(self, mode):
        try:
            self.bus.write_byte_data(self.i2c_addr, self.__RGB_MODE, mode)
        except:
            print('Sensor not connected!')
    
    #Set the color of the light
    #Parameter 1: 0 for the left light, 1 for the right light
    #Parameter 2：RGB ratio value of the color， passed in as a tuple，ranging from 0-255, in order of r，g，b
    def setRGB(self, index, rgb):
        start_reg = 3 if index == 1 else 6
        try:
            self.bus.write_byte_data(self.i2c_addr, start_reg, rgb[0])
            self.bus.write_byte_data(self.i2c_addr, start_reg+1, rgb[1])
            self.bus.write_byte_data(self.i2c_addr, start_reg+2, rgb[2])
        except:
            print('Sensor not connected!')
    
    #Set the color change cycle
    #Parameter 1：0 for the left light, 1 for the right light
    #Parameter 2：Color channel， 0 means natural，1 means g， 2 means b
    #Parameter 3：Color change period (units: ms)
    def setBreathCycle(self, index, rgb, cycle):
        start_reg = 9 if index == 1 else 12
        cycle = int(cycle / 100)
        try:
            self.bus.write_byte_data(self.i2c_addr, start_reg + rgb, cycle)
        except:
            print('Sensor not connected!')

    def startSymphony(self):
        self.setRGBMode(1)
        self.setBreathCycle(1, 0, 2000)
        self.setBreathCycle(1, 1, 3300)
        self.setBreathCycle(1, 2, 4700)
        self.setBreathCycle(0, 0, 4600)
        self.setBreathCycle(0, 1, 2000)
        self.setBreathCycle(0, 2, 3400)

    #Get the distance (units: mm)
    def getDistance(self):
        dist = 5000
        try:
            msg = i2c_msg.write(self.i2c_addr, [0,])
            self.bus.i2c_rdwr(msg)
            read = i2c_msg.read(self.i2c_addr, 2)
            self.bus.i2c_rdwr(read)
            dist = int.from_bytes(bytes(list(read)), byteorder='little', signed=False)
            if dist > 5000:
                dist = 5000
        except BaseException as e:
            print('Sensor not connected!', e)
        return dist

if __name__ == '__main__':
    s = Sonar()
    s.setRGBMode(0)
    s.setRGB(1, (0, 0, 255))
    s.setRGB(0, (0, 0, 255))
#     s.startSymphony()
    while True:
        time.sleep(0.1)
        print('distance: {}(mm)'.format(s.getDistance()))
