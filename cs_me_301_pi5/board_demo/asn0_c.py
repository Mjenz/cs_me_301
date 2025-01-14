import time

import sys
import time
import signal
import threading
import ros_robot_controller_sdk as rrc
from smbus2 import SMBus, i2c_msg
from sensor import dot_matrix_sensor


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

"""
Resets the robot to a default position

Args:
    speed(int):
        Amount of time the servos have to turn. Smaller numbers mean faster action. 0.2 < speed < 3

Returns:
    None
"""
def reset_pos(speed=1):
    # Make sure we're not moving too fast or too slow
    if not (0.2 <= speed <= 3):
        speed = 3

    board.bus_servo_set_position(speed, [[1, 500], [4, 500], [7, 500], [10, 500], [13, 500], [16, 500]])
    board.bus_servo_set_position(speed, [[2, 500], [5, 500], [8, 500], [11, 500], [14, 500], [17, 500]])
    board.bus_servo_set_position(speed, [[3, 300], [6, 300], [9, 300], [12, 700], [15, 700], [18, 700]])

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

"""
Turns the robot 90 degrees

Args:
    offset (1, -1): 
        Direction for turn. 1 will turn _, -1 will turn _
    speed(int):
        Amount of time the servos have to turn. Smaller numbers mean faster action. 0.2 < speed < 3

Returns:
    None
"""
def turn_90(offset=1, speed=1):
    if offset != 1 or offset != -1:
        offset = 1
    offset = offset * 200
    # Make sure we're not moving too fast or too slow
    if not (0.2 <= speed <= 3):
        speed = 3

    # we need a function to turn right AND left in a different file I think
    for i in range (1, 5):
        # Reset to default position
        reset_pos()
        time.sleep(speed)

        # Lift first 3 legs
        board.bus_servo_set_position(0.5, [[front_left_leg_elevation_1, 300], [middle_right_leg_elevation_1, 700], [back_left_leg_elevation_1, 300]])
        time.sleep(speed)

        # Turn lifted legs
        board.bus_servo_set_position(0.5, [[front_left_leg_azimuth, 500 - offset], [middle_right_leg_azimuth, 500 - offset], [back_left_leg_azimuth, 500 - offset]])
        time.sleep(speed)

        # Put lifted legs back down
        board.bus_servo_set_position(0.5, [[front_left_leg_elevation_1, 500], [middle_right_leg_elevation_1, 500], [back_left_leg_elevation_1, 500]])
        time.sleep(speed)

        # Repeat process
        board.bus_servo_set_position(0.5, [[front_right_leg_elevation_1, 700], [middle_left_leg_elevation_1, 300], [back_right_leg_elevation_1, 700]])
        time.sleep(speed)
        board.bus_servo_set_position(0.5, [[front_right_leg_azimuth, 500 - offset], [middle_left_leg_azimuth, 500 - offset], [back_right_leg_azimuth, 500 - offset]])
        time.sleep(speed)
        board.bus_servo_set_position(0.5, [[front_right_leg_elevation_1, 500], [middle_left_leg_elevation_1, 500], [back_right_leg_elevation_1, 500]])
        time.sleep(speed)

    reset_pos()

"""
Reads from sonar and prints.

Args:
    readTime (int): 
        time in seconds to read for 0.5 < readTime < 10

Returns:
    None
"""
def read_sonar(readTime=1):
    readTime = max(0.5, min(10, readTime))
    start_time = time.time()
    end_time = start_time + readTime

    s = Sonar()
    while time.time() < end_time:
        print(s.getDistance())

"""
Reads from sonar and returns average over time.

Args:
    readTime (int): 
        time in seconds to read for 0.5 < readTime < 10

Returns:
    average distance over readTime
"""
def sonar_average(readTime = 1):
    readTime = max(0.5, min(10, readTime))
    start_time = time.time()
    end_time = start_time + readTime
    all_dist = []
    s = Sonar()
    while time.time() < end_time:
        dist = s.getDistance()
        all_dist.append(dist)
        print(s.getDistance())
        time.sleep(.05)

    return sum(all_dist)/len(all_dist)

"""
Prints "Stuck"

Args:
    None:
Returns:
    None
"""
def display_message_stuck():
    tm = dot_matrix_sensor.TM1640(dio=7, clk=8)
    ## display message
    tm.display_buf = (
        0x7C, 0x54, 0x5C,    # S (3 columns)
        0x44, 0x7E,          # T (2 columns)
        0x7C, 0x40, 0x7C,    # U (3 columns)
        0x7C, 0x44, 0x28,    # C (3 columns)
        0x7C, 0x10, 0x68,    # K (3 columns)
        0x00, 0x00           # 2 blank columns
    )

    tm.update_display()

    time.sleep(5)
    tm.display_buf = [0] * 16
    tm.update_display()

"""
Checks for obstacles, walks if none detected, checks for change in average distance from sonar, then loops

Args:
    cycles (int): 
        number of times to repeat the walk cycle

Returns:
    None
"""
def walk_with_object_avoidance(cycles=10):
    # for detecting if we're infinitly turning
    turn_counter = 0

    # Executes the walk and obstacle detection cycle i times. Each cycle consists of checking the sonar average to detect if there's something in front of the pod.
    # walks if none, else turns. After walk, takes new sonar average and compares to old average. Turns if enough decrease is noticed, else loops 
    for i in range(cycles):
        # Sonar average before walk
        cycle_one_average = sonar_average()

        # Makes sure there's room to walk. if not, turns
        if cycle_one_average > 400:
            walk()
            turn_counter = 0
        else:
            turn_90()
            turn_counter += 1

            # Checks to see if we're stuck
            if turn_counter >= 4:
                print("We're stuck")
                display_message_stuck()
                break
        
        # Checks average after walk
        cycle_two_average = sonar_average()

        difference = cycle_one_average - cycle_two_average

        # Turns if difference is small enough to indicate an obstacle
        if difference > 200:
            turn_90()
            turn_counter += 1
        

if __name__ == '__main__':

    signal.signal(signal.SIGINT, Stop)
    exit()

                
        
