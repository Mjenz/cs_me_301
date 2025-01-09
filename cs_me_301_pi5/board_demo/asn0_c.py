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

while True:

    print('''Assignment 0 for Group c''')
    time.sleep(1) 
    


    


board = rrc.Board
start = True

def Stop(signum, frame):
    global start

    start = False
    print('what are you doing')
    

signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    
    while True:

        for i in range(1, 8, 3):
            try:
                board.bus_servo_set_position(1, [[i+1, 500]])
                board.bus_servo_set_position(1, [[i+10, 500]])            
                time.sleep(1)
                board.bus_servo_set_position(1, [[i, 500]])
                board.bus_servo_set_position(1, [[i+9, 500]])
                time.sleep(1)
                board.bus_servo_set_position(1, [[i+1, -500]])
                board.bus_servo_set_position(1, [[i+10, -500]]) 
                
                    
                if not start:
                    board.bus_servo_stop([1, 2])
                    time.sleep(1)
                    print('what')
                    break
            except KeyboardInterrupt:
                break
            
    
    
        
