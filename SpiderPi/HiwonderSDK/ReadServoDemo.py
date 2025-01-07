import time
import Board

print('''
**********************************************************
*****SpiderPi Test******
**********************************************************
----------------------------------------------------------
The following commands need to be used in the terminal.
      The terminal can be opened through  ctrl+alt+t,
      or click on the black icon in the upper bar.
----------------------------------------------------------
Usage:
    sudo python3 test.py
----------------------------------------------------------
Version: --V1.1  2020/11/13
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close the program. If it fails,
      please try multiple times！
----------------------------------------------------------
''')

def getBusServoStatus():

    Pulse = Board.getBusServoPulse(2)  # Get the position information of servo 2
    Temp = Board.getBusServoTemp(9)  # Get the temperature information of servo 9
    Vin = Board.getBusServoVin(9)  # Get the voltage information of servo 9
    print('Pulse: {}\nTemp:  {}℃\nVin:   {}mV\n'.format(Pulse, Temp, Vin)) # print the information

    time.sleep(0.5)  # time delay
#    Board.unloadBusServo(9)

while True:   
    getBusServoStatus()
    time.sleep(1)
