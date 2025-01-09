import time

import sys
import time
import signal
import threading
import ros_robot_controller_sdk as rrc


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

if __name__ == '__main__':
    

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

    signal.signal(signal.SIGINT, Stop)
    exit()

                
        
