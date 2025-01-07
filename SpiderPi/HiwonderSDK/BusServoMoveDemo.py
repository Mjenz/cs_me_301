import time
import Board

print('''
**********************************************************
********SpiderPi, Serial Servo Control Routine*******
**********************************************************
----------------------------------------------------------
Official website:http://www.hiwonder.com
Online mall:https://huaner.tmall.com/
----------------------------------------------------------
The following commands need to be used in the terminal.
      The terminal can be opened through  ctrl+alt+t,
      or click on the black icon in the upper bar.
----------------------------------------------------------
Usage:
    sudo python3 BusServoMove.py
----------------------------------------------------------
Version: --V1.1  2020/11/13
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close the program. If it fails,
      please try multiple times！
----------------------------------------------------------
''')

while True:
    # Parameters：Parameter 1: Servo id; Parameter 2：Position; Parameter 3：Running time
    Board.setBusServoPulse(9, 200, 1000)  
    Board.setBusServoPulse(7, 500, 1000)
    time.sleep(1) 
    
    Board.setBusServoPulse(9, 500, 500)  # Servo #9 turns to positon 500 in 500ms
    time.sleep(0.5)  # Delay 0.5s
    
    Board.setBusServoPulse(9, 800, 500)  # The rotation range of the servo is 0-240 degrees and the corresponding pulse range is 1-1000
    time.sleep(0.5)
    
    Board.setBusServoPulse(7, 300, 400) # Servo #7 turns to positon 300 in 400ms
    time.sleep(0.4)

    Board.setBusServoPulse(7, 700, 800)
    time.sleep(0.8) 

    Board.setBusServoPulse(7, 500, 400)
    time.sleep(0.4)  

   
