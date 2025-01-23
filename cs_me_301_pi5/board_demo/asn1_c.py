import time

import sys
import time
import signal
import threading
import ros_robot_controller_sdk as rrc
from smbus2 import SMBus, i2c_msg
from sensor import dot_matrix_sensor
import argparse
from multiprocessing import Process
import numpy as np

parser = argparse.ArgumentParser(
    prog='asn0_c.py'
)
parser.add_argument('-b', '--behavior', nargs='+', type=str, help="Takes in valid behaviors to run and calls them with default values")
parser.add_argument('-t', '--turn', nargs=2, type=float, default=[])
parser.add_argument('-t180', '--turn180', nargs=2, type=float, default=[])
parser.add_argument('-wwa', '--walk_with_avoidance', nargs=1, type=int, help="Walks for arg1 cycles, checking for obstacles after during each cycle")
parser.add_argument('-w', '--walk', help="walks forward once")
parser.add_argument('-kd', '--keep_distance', nargs=1, type=int, help="Walks forward until obstacle is detected, then walks backward")
parser.add_argument('-r', '--reset_pos', nargs=1, type=int, default=1)
parser.add_argument('-rsa', '--read_sonar_average', nargs=1, type=int, default=1, help="Reads the average sonar output over inputted time in seconds")
parser.add_argument('-fw', '--follow_wall', nargs=3, type=float, help="follow a wall")
parser.add_argument('-wt', '--wave_turn')
args = parser.parse_args()
print(f"running following behaviors: {args}")



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
    

#signal.signal(signal.SIGINT, Stop)

"""
Resets the robot to a default position

Args:
    speed(int):
        Amount of time the servos have to turn. Smaller numbers mean faster action. 0.2 < speed < 3

Returns:
    None
"""
def reset_pos(speed=1, height=0):
    # Make sure we're not moving too fast or too slow
    if not (0.2 <= speed or speed <= 3):
        speed = 3
    if not (0 <= height or height <= 450):
        height = 0

    board.bus_servo_set_position(speed, [[1, 500], [4, 500], [7, 500], [10, 500], [13, 500], [16, 500]])
    board.bus_servo_set_position(speed, [[2, 500-height], [5, 500-height], [8, 500-height], [11, 500+height], [14, 500+height], [17, 500+height]])
    board.bus_servo_set_position(speed, [[3, 300], [6, 300], [9, 300], [12, 700], [15, 700], [18, 700]])
def walk(direction=1, speed=1):

    # Elevation 1
    # group 1: 2, 8, 14
    # group 2: 5, 11, 17

    # Azmith
    # group 1: 1, 7, 13
    # group 2: 4, 10, 16
    if direction == 1:
        # lift up elevation group 1
        board.bus_servo_set_position(speed, [[back_left_leg_elevation_1, 400],[front_left_leg_elevation_1, 400],[middle_right_leg_elevation_1, 600]])
        time.sleep(1)
        # azmith group 2 backward turn
        board.bus_servo_set_position(speed, [[middle_left_leg_azimuth, 600],[front_right_leg_azimuth, 400],[back_right_leg_azimuth, 400]])
        time.sleep(1)
        # azmith group 1 forward turn (in air)
        board.bus_servo_set_position(speed, [[back_left_leg_azimuth,400],[front_left_leg_azimuth, 400],[middle_right_leg_azimuth, 600]])
        time.sleep(1)
        # put down elevation group 1
        board.bus_servo_set_position(speed, [[back_left_leg_elevation_1, 500],[front_left_leg_elevation_1, 500],[middle_right_leg_elevation_1, 500]])
        time.sleep(1)


        # lift up elevation group 2
        board.bus_servo_set_position(speed, [[middle_left_leg_elevation_1, 400],[front_right_leg_elevation_1, 600],[back_right_leg_elevation_1, 600]])
        time.sleep(1)
        # azmith group 1 backward turn
        board.bus_servo_set_position(speed, [[back_left_leg_azimuth, 600],[front_left_leg_azimuth, 600],[middle_right_leg_azimuth, 400]])
        time.sleep(1)
        # azmith group 2 forward turn (in air)
        board.bus_servo_set_position(speed, [[middle_left_leg_azimuth, 400],[front_right_leg_azimuth, 600],[back_right_leg_azimuth, 600]])
        time.sleep(1) 
        # put down elevation group 2
        board.bus_servo_set_position(speed, [[middle_left_leg_elevation_1, 500],[front_right_leg_elevation_1, 500],[back_right_leg_elevation_1, 500]])
        time.sleep(1)
    else:
         # lift up elevation group 1
        board.bus_servo_set_position(speed, [[back_left_leg_elevation_1, 400],[front_left_leg_elevation_1, 400],[middle_right_leg_elevation_1, 600]])
        time.sleep(1)
        # azmith group 2 forward turn
        board.bus_servo_set_position(speed, [[middle_left_leg_azimuth, 400],[front_right_leg_azimuth, 600],[back_right_leg_azimuth, 600]])
        time.sleep(1)
        # azmith group 1 back turn (in air)
        board.bus_servo_set_position(speed, [[back_left_leg_azimuth,600],[front_left_leg_azimuth, 600],[middle_right_leg_azimuth, 400]])
        time.sleep(1)
        # put down elevation group 1
        board.bus_servo_set_position(speed, [[back_left_leg_elevation_1, 500],[front_left_leg_elevation_1, 500],[middle_right_leg_elevation_1, 500]])
        time.sleep(1)


        # lift up elevation group 2
        board.bus_servo_set_position(speed, [[middle_left_leg_elevation_1, 400],[front_right_leg_elevation_1, 600],[back_right_leg_elevation_1, 600]])
        time.sleep(1)
        # azmith group 1 forward turn
        board.bus_servo_set_position(speed, [[back_left_leg_azimuth, 400],[front_left_leg_azimuth, 400],[middle_right_leg_azimuth, 600]])
        time.sleep(1)
        # azmith group 2 forward turn (in air)
        board.bus_servo_set_position(speed, [[middle_left_leg_azimuth, 600],[front_right_leg_azimuth, 400],[back_right_leg_azimuth, 400]])
        time.sleep(1) 
        # put down elevation group 2
        board.bus_servo_set_position(speed, [[middle_left_leg_elevation_1, 500],[front_right_leg_elevation_1, 500],[back_right_leg_elevation_1, 500]])
        time.sleep(1)

        
def strafe(direction=1, speed=1):
    reset_pos()
    time.sleep(1)
    # lift group 1 legs and realign
    board.bus_servo_set_position(speed, [[front_left_leg_elevation_1, 300], [back_left_leg_elevation_1, 300], [middle_right_leg_elevation_1, 700]])
    board.bus_servo_set_position(speed, [[front_left_leg_azimuth, 650], [back_left_leg_azimuth, 350]])
    time.sleep(1)
    board.bus_servo_set_position(speed, [[front_left_leg_elevation_1, 500], [back_left_leg_elevation_1, 500], [middle_right_leg_elevation_1, 500]])
    time.sleep(1)

    # lift group 2 legs and realign
    board.bus_servo_set_position(speed, [[front_right_leg_elevation_1, 700], [back_right_leg_elevation_1, 700], [middle_left_leg_elevation_1, 300]])
    board.bus_servo_set_position(speed, [[front_right_leg_azimuth, 650], [back_right_leg_azimuth, 350], [middle_left_leg_azimuth, 500]])
    time.sleep(1)
    board.bus_servo_set_position(speed, [[front_right_leg_elevation_1, 500], [back_right_leg_elevation_1, 500], [middle_left_leg_elevation_1, 500]])
    time.sleep(1)
    for i in range(0,5):

        # Lift group 1 legs
        board.bus_servo_set_position(speed, [[front_left_leg_elevation_1, 300], [back_left_leg_elevation_1, 300], [middle_right_leg_elevation_1, 700]])
        time.sleep(1)


        # Strafe group 2 legs
        board.bus_servo_set_position(speed*2, [[middle_left_leg_elevation_2, 550], [back_right_leg_elevation_2, 800], [front_right_leg_elevation_2, 800]])
        time.sleep(1)

        # Set group 1 legs down
        board.bus_servo_set_position(speed, [[front_left_leg_elevation_1, 500], [back_left_leg_elevation_1, 500], [middle_right_leg_elevation_1, 500]])
        time.sleep(1)
        # Lift group 2 legs up and reset positions
        board.bus_servo_set_position(speed, [[front_right_leg_elevation_1, 800], [back_right_leg_elevation_1, 800], [middle_left_leg_elevation_1, 200]])
        board.bus_servo_set_position(speed, [[front_right_leg_elevation_2, 705], [back_right_leg_elevation_2, 705], [middle_left_leg_elevation_2, 295]])
        time.sleep(1)
        board.bus_servo_set_position(speed, [[front_right_leg_elevation_1, 500], [back_right_leg_elevation_1, 500], [middle_left_leg_elevation_1, 500]])

        time.sleep(4)
    # lift group 1 legs and realign to original position
    board.bus_servo_set_position(speed, [[front_left_leg_elevation_1, 300], [back_left_leg_elevation_1, 300], [middle_right_leg_elevation_1, 700]])
    board.bus_servo_set_position(speed, [[front_left_leg_azimuth, 500], [back_left_leg_azimuth, 500]])
    time.sleep(1)
    board.bus_servo_set_position(speed, [[front_left_leg_elevation_1, 500], [back_left_leg_elevation_1, 500], [middle_right_leg_elevation_1, 500]])
    time.sleep(1)

    # lift group 2 legs and realign to original position
    board.bus_servo_set_position(speed, [[front_right_leg_elevation_1, 700], [back_right_leg_elevation_1, 700], [middle_left_leg_elevation_1, 300]])
    board.bus_servo_set_position(speed, [[front_right_leg_azimuth, 500], [back_right_leg_azimuth, 500], [middle_left_leg_azimuth, 500]])
    time.sleep(1)
    board.bus_servo_set_position(speed, [[front_right_leg_elevation_1, 500], [back_right_leg_elevation_1, 500], [middle_left_leg_elevation_1, 500]])
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
    offset = int(offset)
    
    if offset != 1 :
        if offset != -1:
            print("fixing")
            offset = 1
    if offset == 1:
        offset = offset * 207
    else: 
        offset = offset * 214
    print(offset)
    # Make sure we're not moving too fast or too slow
    if not (0.2 <= speed <= 3):
        speed = 3

    # we need a function to turn right AND left in a different file I think
    for i in range (1, 5):
        # Reset to default position
        reset_pos(speed)
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

    reset_pos(speed)

def turn_180(offset=1, speed=1):
    reset_pos()
    turn_90(offset, speed)
    time.sleep(1)
    turn_90(offset, speed)

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
def sonar_average(readTime = 1, v=False):
    readTime = max(0.1, min(10, readTime))
    start_time = time.time()
    end_time = start_time + readTime
    all_dist = []
    s = Sonar()
    while time.time() < end_time:
        dist = s.getDistance()
        all_dist.append(dist)
        if v:
            print(s.getDistance())
        #time.sleep(.05)

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
        0x7C, 0x54, 0x5C, 0x00,    # S (3 columns)
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
    reset_pos()
    if cycles < 1 or cycles > 15:
        cycles = 2

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

    reset_pos()

def execute_behaviors():
    reset_pos()
    print(args)
    behavior_map ={
    "turn_90": turn_90,
    "walk": walk,
    "walk_with_object_avoidance": walk_with_object_avoidance
    }
    try:
        if args.behavior:
            for b in args.behavior:
                if b in behavior_map:
                    behavior_map[b]()

        if args.turn:
            direction = int(args.turn[0])
            turn_90(args.turn[0], args.turn[1])

        if args.turn180:
            direction = int(args.turn180[0])
            turn_180(args.turn180[0], args.turn180[1])

        if args.walk:
            walk()

        if args.walk_with_avoidance:
            walk_with_object_avoidance(args.walk_with_avoidance[0])
        
        if args.keep_distance:
            keep_distance(args.keep_distance[0])

        if args.reset_pos:
            reset_pos(args.reset_pos[0])

        if args.read_sonar_average:
            results = sonar_average(args.read_sonar_average[0])
            print(results)

        if args.follow_wall:
            print("following wall")
            wall_following(args.follow_wall[0], args.follow_wall[1], args.follow_wall[2])

        if args.wave_turn:
            print("wave turning")
            wave_turn()

    except Exception as e:
        print(e)

def keep_distance(setpoint = 300):
    s = Sonar()
    for i in range(0, 10):
        distance = []
        end_time = time.time()+1
        while time.time() < end_time:
            distance.append(s.getDistance())
            #print(s.getDistance())
        avg = np.average(distance)
        print(f"{i} average: {avg}")

        if avg < setpoint:
            number_of_steps = round((setpoint-avg)/100)+1 # Since each step is roughly 100mm forward, take that many steps forward
            speed = 2/number_of_steps
            if speed < .25:
                speed = .25
           
            print(f"avg ({avg})forward at speed {speed} for {number_of_steps} steps")
            walk(-1,speed)
            # time.sleep(5)
        if avg > setpoint:
            number_of_steps = round((avg-setpoint)/100)+1
            speed = 2/number_of_steps
            if speed < .25:
                speed = .25
            print(f"avg ({avg})forward at speed {speed} for {number_of_steps} steps")
            walk(1,speed)
            # time.sleep(5)

        else:
            continue








left_el_1_rest = 350
right_el_1_rest = 650

def walking_pos():
    board.bus_servo_set_position(1, [[1, 500], [4, 500], [7, 500], [10, 500], [13, 500], [16, 500]])
    board.bus_servo_set_position(1, [[2, left_el_1_rest], [5, left_el_1_rest], [8, left_el_1_rest], [11, right_el_1_rest], [14, right_el_1_rest], [17, right_el_1_rest]])
    board.bus_servo_set_position(1, [[3, 250], [6, 250], [9, 250], [12, 750], [15, 750], [18, 750]])
    time.sleep(1)

   
def walk_straight(speed = 1):
     # lift up elevation group 1
    board.bus_servo_set_position(speed, [[back_left_leg_elevation_1, left_el_1_rest-100],[front_left_leg_elevation_1, left_el_1_rest-100],[middle_right_leg_elevation_1, right_el_1_rest+100]])
    time.sleep(speed)
    # azmith group 2 backward turn
    board.bus_servo_set_position(speed, [[middle_left_leg_azimuth, 575],[front_right_leg_azimuth, 425],[back_right_leg_azimuth, 425]])
    time.sleep(speed)
    # azmith group 1 forward turn (in air)
    board.bus_servo_set_position(speed, [[back_left_leg_azimuth,425],[front_left_leg_azimuth, 425],[middle_right_leg_azimuth, 575]])
    time.sleep(speed)
    # put down elevation group 1
    board.bus_servo_set_position(speed, [[back_left_leg_elevation_1, left_el_1_rest],[front_left_leg_elevation_1, left_el_1_rest],[middle_right_leg_elevation_1, right_el_1_rest]])
    time.sleep(speed)


    # lift up elevation group 2
    board.bus_servo_set_position(speed, [[middle_left_leg_elevation_1, left_el_1_rest-100],[front_right_leg_elevation_1, right_el_1_rest+100],[back_right_leg_elevation_1, right_el_1_rest+100]])
    time.sleep(speed)
    # azmith group 1 backward turn ********* THIS IS THE ONE THAT CHANGES THE DIRECTION *********
    board.bus_servo_set_position(speed, [[back_left_leg_azimuth, 560],[front_left_leg_azimuth, 560],[middle_right_leg_azimuth, 425]])
    time.sleep(speed)
    # azmith group 2 forward turn (in air)
    board.bus_servo_set_position(speed, [[middle_left_leg_azimuth, 425],[front_right_leg_azimuth, 575],[back_right_leg_azimuth, 575]])
    time.sleep(speed) 
    # put down elevation group 2
    board.bus_servo_set_position(speed, [[middle_left_leg_elevation_1, left_el_1_rest],[front_right_leg_elevation_1, right_el_1_rest],[back_right_leg_elevation_1, right_el_1_rest]])
    time.sleep(speed)

def walk_variable(speed,u_sign,u):
    if u_sign is True: 
        # POSITIVE error indicating that robot is too close
            # for RIGHT following --> turn left
            # for LEFT following --> turn right
            # sign is actually handled in the controller, this is assuming that we are right following. so turn LEFT
        # lift up elevation group 1
        board.bus_servo_set_position(speed, [[back_left_leg_elevation_1, left_el_1_rest-100],[front_left_leg_elevation_1, left_el_1_rest-100],[middle_right_leg_elevation_1, right_el_1_rest+100]])
        time.sleep(speed)
        # azmith group 2 backward turn
        board.bus_servo_set_position(speed, [[middle_left_leg_azimuth, 575],[front_right_leg_azimuth, 425],[back_right_leg_azimuth, 425]])
        time.sleep(speed)
        # azmith group 1 forward turn (in air)
        board.bus_servo_set_position(speed, [[back_left_leg_azimuth,425],[front_left_leg_azimuth, 425],[middle_right_leg_azimuth, 575]])
        time.sleep(speed)
        # put down elevation group 1
        board.bus_servo_set_position(speed, [[back_left_leg_elevation_1, left_el_1_rest],[front_left_leg_elevation_1, left_el_1_rest],[middle_right_leg_elevation_1, right_el_1_rest]])
        time.sleep(speed)


        # lift up elevation group 2
        board.bus_servo_set_position(speed, [[middle_left_leg_elevation_1, left_el_1_rest-100],[front_right_leg_elevation_1, right_el_1_rest+100],[back_right_leg_elevation_1, right_el_1_rest+100]])
        time.sleep(speed)
        # azmith group 1 backward turn ********* THIS IS THE ONE THAT CHANGES THE DIRECTION *********
        # 560 is roughly straight ahead
        board.bus_servo_set_position(speed, [[back_left_leg_azimuth, round(560-u)],[front_left_leg_azimuth, round(560-u)],[middle_right_leg_azimuth, 425]])
        time.sleep(speed)
        # azmith group 2 forward turn (in air)
        board.bus_servo_set_position(speed, [[middle_left_leg_azimuth, 425],[front_right_leg_azimuth, 575],[back_right_leg_azimuth, 575]])
        time.sleep(speed) 
        # put down elevation group 2
        board.bus_servo_set_position(speed, [[middle_left_leg_elevation_1, left_el_1_rest],[front_right_leg_elevation_1, right_el_1_rest],[back_right_leg_elevation_1, right_el_1_rest]])
        time.sleep(speed)



def walk_variable_mightnotwork(speed,u_sign,u):
    if u_sign is True: 
        # POSITIVE error indicating that robot is too close
            # for RIGHT following --> turn left
            # for LEFT following --> turn right
            # sign is actually handled in the controller, this is assuming that we are right following. so turn LEFT

        l_az_adjust = round(560-u)
        r_az_adjust = round(425-u)

        # Joint limits

        if l_az_adjust < 500:
            l_az_adjust = 500
        if r_az_adjust < 350:
            r_az_adjust = 350

        # lift up elevation group 1
        board.bus_servo_set_position(speed, [[back_left_leg_elevation_1, left_el_1_rest-100],[front_left_leg_elevation_1, left_el_1_rest-100],[middle_right_leg_elevation_1, right_el_1_rest+100]])
        time.sleep(speed)
        # azmith group 2 backward turn
        board.bus_servo_set_position(speed, [[middle_left_leg_azimuth,l_az_adjust ],[front_right_leg_azimuth, r_az_adjust],[back_right_leg_azimuth, r_az_adjust]])
        time.sleep(speed)
        # azmith group 1 forward turn (in air)
        board.bus_servo_set_position(speed, [[back_left_leg_azimuth,425],[front_left_leg_azimuth, 425],[middle_right_leg_azimuth, 575]])
        time.sleep(speed)
        # put down elevation group 1
        board.bus_servo_set_position(speed, [[back_left_leg_elevation_1, left_el_1_rest],[front_left_leg_elevation_1, left_el_1_rest],[middle_right_leg_elevation_1, right_el_1_rest]])
        time.sleep(speed)


        # lift up elevation group 2
        board.bus_servo_set_position(speed, [[middle_left_leg_elevation_1, left_el_1_rest-100],[front_right_leg_elevation_1, right_el_1_rest+100],[back_right_leg_elevation_1, right_el_1_rest+100]])
        time.sleep(speed)
        # azmith group 1 backward turn ********* THIS IS THE ONE THAT CHANGES THE DIRECTION *********
        # 560 is roughly straight ahead
        board.bus_servo_set_position(speed, [[back_left_leg_azimuth, l_az_adjust],[front_left_leg_azimuth, l_az_adjust],[middle_right_leg_azimuth, r_az_adjust]])
        time.sleep(speed)
        # azmith group 2 forward turn (in air)
        board.bus_servo_set_position(speed, [[middle_left_leg_azimuth, 425],[front_right_leg_azimuth, 575],[back_right_leg_azimuth, 575]])
        time.sleep(speed) 
        # put down elevation group 2
        board.bus_servo_set_position(speed, [[middle_left_leg_elevation_1, left_el_1_rest],[front_right_leg_elevation_1, right_el_1_rest],[back_right_leg_elevation_1, right_el_1_rest]])
        time.sleep(speed)

    if u_sign is False: 
        # NEGATIVE error indicating that robot is too far
            # for RIGHT following --> turn right
            # for LEFT following --> turn left
            # sign is actually handled in the controller, this is assuming that we are right following. so turn RIGHT

        l_az_adjust = round(560+u)
        r_az_adjust = round(425+u)

        # Joint limits

        if l_az_adjust > 700:
            l_az_adjust = 700
        if r_az_adjust < 500:
            r_az_adjust = 500

         # lift up elevation group 1
        board.bus_servo_set_position(speed, [[back_left_leg_elevation_1, left_el_1_rest-100],[front_left_leg_elevation_1, left_el_1_rest-100],[middle_right_leg_elevation_1, right_el_1_rest+100]])
        time.sleep(speed)
        # azmith group 2 backward turn
        board.bus_servo_set_position(speed, [[middle_left_leg_azimuth, l_az_adjust],[front_right_leg_azimuth, r_az_adjust],[back_right_leg_azimuth, r_az_adjust]])
        time.sleep(speed)
        # azmith group 1 forward turn (in air)
        board.bus_servo_set_position(speed, [[back_left_leg_azimuth,425],[front_left_leg_azimuth, 425],[middle_right_leg_azimuth, 575]])
        time.sleep(speed)
        # put down elevation group 1
        board.bus_servo_set_position(speed, [[back_left_leg_elevation_1, left_el_1_rest],[front_left_leg_elevation_1, left_el_1_rest],[middle_right_leg_elevation_1, right_el_1_rest]])
        time.sleep(speed)


        # lift up elevation group 2
        board.bus_servo_set_position(speed, [[middle_left_leg_elevation_1, left_el_1_rest-100],[front_right_leg_elevation_1, right_el_1_rest+100],[back_right_leg_elevation_1, right_el_1_rest+100]])
        time.sleep(speed)
        # azmith group 1 backward turn ********* THIS IS THE ONE THAT CHANGES THE DIRECTION *********
        board.bus_servo_set_position(speed, [[back_left_leg_azimuth, l_az_adjust],[front_left_leg_azimuth, l_az_adjust],[middle_right_leg_azimuth, r_az_adjust]])
        time.sleep(speed)
        # azmith group 2 forward turn (in air)
        board.bus_servo_set_position(speed, [[middle_left_leg_azimuth, 425],[front_right_leg_azimuth, 575],[back_right_leg_azimuth, 575]])
        time.sleep(speed) 
        # put down elevation group 2
        board.bus_servo_set_position(speed, [[middle_left_leg_elevation_1, left_el_1_rest],[front_right_leg_elevation_1, right_el_1_rest],[back_right_leg_elevation_1, right_el_1_rest]])
        time.sleep(speed)


def wall_following(speed,side=1, k_p=0.01):
    """
    Follow a wall on a specified side

    :param speed - how fast the walking movements are
    :param side - indicates the side that you are following a wall on, 1 is right, -1 is left.
    """
    setpoint = 350 # setpoint is 35 cm from wall

    # init vars
    e_int = 0
    e_prev = 0

    # k_p = from parameter
    k_i = 0
    k_d = 0 

    while True:
        # get reading on left or right side
        walking_pos()
        actual = sonar_average()

        # calculate error
        e = setpoint - actual
        e_int += e # should be multiplied by delta time
        e_der = e - e_prev # should be divided by delta time
        e_prev = e
        

        # apply gains
        u = k_p * e + k_i * e_int + k_d * e_der

        print(f"u {u}\te {e}\te_int {e_int}\te_der {e_der}\t distance {actual}")

        u_sign = True if np.sign(u) == 1 else False

        u = abs(u)

        if side < 0: # Indicates left following, flip u_sign
            u_sign = not u_sign

        # walk_variable(speed,u_sign,u)
        # walk_variable(speed,u_sign,u)
        walk_variable_mightnotwork(speed,u_sign,u)

# def turn_from_obstacles(speed=1, cycles=100):

    # for i in range(cycles):
        # sonar_average() of front sonar
        # sonar_average() of left sonar
        # sonar_average() or right sonar

        # if sonar_average(front) < 350:
            # if sonar_average(left) ^ sonar_average(right) < 350:
                # turn(1)
            # elif sonar_average(right) < 350:
                # turn(-1)
            # elif sonar_average(right) < 350 && sonar_average(left):
                # turn_180()

def wave_turn():
    readTime = 10
    start_time = time.time()
    end_time = start_time + readTime
    waves = 0
    already_turned = False
    while time.time() < end_time:
        step = sonar_average(0.1)
        if step < 200:
            while sonar_average(0.1) < 200:
                pass
            waves += 1
            print(f"detected wave: {waves}")
            time.sleep(.3)
        if waves == 3:
            print("turning around")
            turn_180()
            already_turned = True
            break
    if waves == 1:
        print("turning right")
        turn_90(1, 1)
    elif waves == 2:
        print("turning left")
        turn_90(-1, 1)
    elif waves == 3 and not already_turned:
        print("turning around")
        turn_180()

if __name__ == '__main__':

    reset_pos()
    time.sleep(1)
    execute_behaviors()
    time.sleep(0.5)
    wall_following(0.2, 1, 0.25)
    # wave_turn()
    signal.signal(signal.SIGINT, Stop)
    exit()

                
        
