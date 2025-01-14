import time

import sys
import time
import signal
import threading
import ros_robot_controller_sdk as rrc
from smbus2 import SMBus, i2c_msg


print('''
**********************************************************
********CS/ME 301 Assignment Template*******
**********************************************************
----------------------------------------------------------
Usage:
    sudo python3 asn_template.py
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close the program. If it fails,
      please try multiple times！
----------------------------------------------------------
''')

class Sonar:
    __units = {"mm":0, "cm":1}
    __dist_reg = 0

    def __init__(self):
        self.i2c_addr = 0x77
        self.i2c = 1
        self.Pixels = [0,0]
        self.RGBMode = 0

    def __getattr(self, attr):
        if attr in self.__units:
            return self.__units[attr]
        if attr == "Distance":
            return self.getDistance()
        else:
            raise AttributeError('Unknow attribute : %s'%attr)
        
    def getDistance(self):
        dist = 99999
        try:
            with SMBus(self.i2c) as bus:
                msg = i2c_msg.write(self.i2c_addr, [0,])
                bus.i2c_rdwr(msg)
                read = i2c_msg.read(self.i2c_addr, 2)
                bus.i2c_rdwr(read)
                dist = int.from_bytes(bytes(list(read)), byteorder='little', signed=False)
                if dist > 5000:
                    dist = 5000
        except BaseException as e:
            print(e)
        return dist

back_left_leg_azimuth = 1
back_left_leg_elevation_1 = 2
back_left_leg_elevation_2 = 3

middle_left_leg_azimuth = 4
middle_left_leg_elevation_1 = 5
middle_left_leg_elevation_2 = 6

front_left_leg_azimuth = 7
front_left_leg_elevation_1 = 8
front_left_leg_elevation_2 = 9

front_right_leg_azimuth = 10
front_right_leg_elevation_1 = 11
front_right_leg_elevation_2 = 12

middle_right_leg_azimuth = 13
middle_right_leg_elevation_1 = 14
middle_right_leg_elevation_2 = 15

back_right_leg_azimuth = 16
back_right_leg_elevation_1 = 17
back_right_leg_elevation_2 = 18

board = rrc.Board()
start = True

def Stop(signum, frame):
    global start

    start = False
    print('what are you doing')
    

    signal.signal(signal.SIGINT, Stop)

def reset_pos():
    board.bus_servo_set_position(0.5, [[1, 500], [4, 500], [7, 500], [10, 500], [13, 500], [16, 500]])
    board.bus_servo_set_position(0.5, [[2, 500], [5, 500], [8, 500], [11, 500], [14, 500], [17, 500]])
    board.bus_servo_set_position(0.5, [[3, 300], [6, 300], [9, 300], [12, 700], [15, 700], [18, 700]])

def walk():

    # Elevation 1
    # group 1: 2, 8, 14
    # group 2: 5, 11, 17

    # Azmith
    # group 1: 1, 7, 13
    # group 2: 4, 10, 16

    # lift up elevation group 1
    board.bus_servo_set_position(1, [[back_left_leg_elevation_1, 400],[front_left_leg_elevation_1, 400],[middle_right_leg_elevation_1, 600]])
    time.sleep(1)
    # azmith group 2 backward turn
    board.bus_servo_set_position(1, [[middle_left_leg_azimuth, 600],[front_right_leg_azimuth, 400],[back_right_leg_azimuth, 400]])
    time.sleep(1)
    # azmith group 1 forward turn (in air)
    board.bus_servo_set_position(1, [[back_left_leg_azimuth,400],[front_left_leg_azimuth, 400],[middle_right_leg_azimuth, 600]])
    time.sleep(1)
    # put down elevation group 1
    board.bus_servo_set_position(1, [[back_left_leg_elevation_1, 500],[front_left_leg_elevation_1, 500],[middle_right_leg_elevation_1, 500]])
    time.sleep(1)


    # lift up elevation group 2
    board.bus_servo_set_position(1, [[middle_left_leg_elevation_1, 400],[front_right_leg_elevation_1, 600],[back_right_leg_elevation_1, 600]])
    time.sleep(1)
    # azmith group 1 backward turn
    board.bus_servo_set_position(1, [[back_left_leg_azimuth, 600],[front_left_leg_azimuth, 600],[middle_right_leg_azimuth, 400]])
    time.sleep(1)
    # azmith group 2 forward turn (in air)
    board.bus_servo_set_position(1, [[middle_left_leg_azimuth, 400],[front_right_leg_azimuth, 600],[back_right_leg_azimuth, 600]])
    time.sleep(1) 
    # put down elevation group 2
    board.bus_servo_set_position(1, [[middle_left_leg_elevation_1, 500],[front_right_leg_elevation_1, 500],[back_right_leg_elevation_1, 500]])
    time.sleep(1)

def turn_90():

    # we need a function to turn right AND left in a different file I think
    for i in range (1, 5):
        reset_pos()
        time.sleep(1)
        board.bus_servo_set_position(0.5, [[front_left_leg_elevation_1, 300], [middle_right_leg_elevation_1, 700], [back_left_leg_elevation_1, 300]])
        time.sleep(.6)
        board.bus_servo_set_position(0.5, [[front_left_leg_azimuth, 300], [middle_right_leg_azimuth, 300], [back_left_leg_azimuth, 300]])
        time.sleep(.6)
        board.bus_servo_set_position(0.5, [[front_left_leg_elevation_1, 500], [middle_right_leg_elevation_1, 500], [back_left_leg_elevation_1, 500]])
        time.sleep(.6)

        board.bus_servo_set_position(0.5, [[front_right_leg_elevation_1, 700], [middle_left_leg_elevation_1, 300], [back_right_leg_elevation_1, 700]])
        time.sleep(.6)
        board.bus_servo_set_position(0.5, [[front_right_leg_azimuth, 300], [middle_left_leg_azimuth, 300], [back_right_leg_azimuth, 300]])
        time.sleep(.6)
        board.bus_servo_set_position(0.5, [[front_right_leg_elevation_1, 500], [middle_left_leg_elevation_1, 500], [back_right_leg_elevation_1, 500]])
        time.sleep(1)

    reset_pos()

def read_sonar():
    start_time = time.time()
    end_time = start_time + 1

    s = Sonar()
    while time.time() < end_time:
        print(s.getDistance())

if __name__ == '__main__':
    read_sonar()

    signal.signal(signal.SIGINT, Stop)
    exit()

                
        
