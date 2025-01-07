#!/usr/bin/env python3
import os
import sys
import time
import pigpio
import RPi.GPIO as GPIO
from PWMServo import *
from BusServoCmd import *
from smbus2 import SMBus, i2c_msg

#SDK, SPIDERPI expansion board SDK#
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

Servos = ()
pi = pigpio.pi()  # Connect to local Pi
def initPWMServo(d):
    global Servos
    
    servo1 = PWM_Servo(pi, 12, deviation=d[0], control_speed=True)  # 1 on the expansion board 1
    servo2 = PWM_Servo(pi, 13, deviation=d[1], control_speed=True)  # 2
    Servos = (servo1, servo2)

d = [0, 0]
initPWMServo(d)

def setPWMServoPulse(servo_id, pulse = 1500, use_time = 1000):
    if servo_id < 1 or servo_id > 2:
        return

    pulse = 2500 if pulse > 2500 else pulse    
    pulse = 500 if pulse < 500 else pulse 
    use_time = 30000 if use_time > 30000 else use_time    
    use_time = 20 if use_time < 20 else use_time

    Servos[servo_id - 1].setPosition(pulse, use_time)

    return pulse

def setDeviation(servo_id, dev):
    if servo_id < 1 or servo_id > 2:
        return
    if d < -300 or d > 300:
        return
    Servos[servo_id - 1].setDeviation(dev)

def setBuzzer(new_state):
    GPIO.setup(31, GPIO.OUT)
    GPIO.output(31, new_state)

def setBusServoID(oldid, newid):
    """
    Configure the bus servo ID number, the factory default is 1
    :param oldid: the original id, the factory default is 1
    :param newid: new id
    """
    serial_serro_wirte_cmd(oldid, LOBOT_SERVO_ID_WRITE, newid)

def getBusServoID(id=None):
    """
    Read the serial port servo ID number
    :param id: servo id, default is None
    :return: servo id
    """
    
    while True:
        if id is None:  # there can only be 1 servo on the bus
            serial_servo_read_cmd(0xfe, LOBOT_SERVO_ID_READ)
        else:
            serial_servo_read_cmd(id, LOBOT_SERVO_ID_READ)
        # get the content of the read command
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ID_READ)
        if msg is not None:
            return msg

def setBusServoPulse(id, pulse, use_time):
    """
    Drive the serial port servo (motor) to the specified position -- send a command to the servo
    :param id: id of the servo to be driven
    :pulse: the position to be driven to
    :use_time: time required to rotate to the specified position
    """

    pulse = 0 if pulse < 0 else pulse
    pulse = 1000 if pulse > 1000 else pulse
    use_time = 0 if use_time < 0 else use_time
    use_time = 30000 if use_time > 30000 else use_time
    serial_serro_wirte_cmd(id, LOBOT_SERVO_MOVE_TIME_WRITE, pulse, use_time)

def stopBusServo(id=None):
    '''
    Stop the servo motor
    :param id: servo id, default is None
    :return: None
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_MOVE_STOP)

def setBusServoDeviation(id, d=0):
    """
    Set the servo deviation
    :param id: Servo id
    :param d:  deviation value
    """
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_ADJUST, d)

def saveBusServoDeviation(id):
    """
    Save the servo deviation, Power off protection
    :param id: Servo id
    """
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_WRITE)

time_out = 50
def getBusServoDeviation(id):
    '''
    Read the servo deviation value
    :param id: Servo id
    :return:
    '''
    # Send the read command to the servo motor
    count = 0
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_READ)
        # Get the content of the read command
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ANGLE_OFFSET_READ)
        count += 1
        if msg is not None:
            return msg
        if count > time_out:
            return None

def setBusServoAngleLimit(id, low, high):
    '''
    Set the servo rotation range
    :param id: Servo id
    :param low: Low limit
    :param high: High limit
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_LIMIT_WRITE, low, high)

def getBusServoAngleLimit(id):
    '''
    Read the servo rotation range
    :param id: Servo id
    :return: Return to origin 0： low bit  1： high bit
    '''
    
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_ANGLE_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ANGLE_LIMIT_READ)
        if msg is not None:
            count = 0
            return msg

def setBusServoVinLimit(id, low, high):
    '''
    Set the servo voltage range
    :param id: Servo id
    :param low: Low limit
    :param high: High limit
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_VIN_LIMIT_WRITE, low, high)

def getBusServoVinLimit(id):
    '''
    Read the servo voltage range
    :param id:
    :return: Return to origin 0： low bit  1： high bit
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_VIN_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_VIN_LIMIT_READ)
        if msg is not None:
            return msg

def setBusServoMaxTemp(id, m_temp):
    '''
    Set the maximum temperature alarm value of the servo
    :param id: Servo id
    :param m_temp: Maximum temperature alarm value
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE, m_temp)

def getBusServoTempLimit(id):
    '''
    Read the maximum temperature alarm value of the servo
    :param id: Servo id
    :return:
    '''
    
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_TEMP_MAX_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_TEMP_MAX_LIMIT_READ)
        if msg is not None:
            return msg

def getBusServoPulse(id):
    '''
    Read the current position of the servo
    :param id: Servo id
    :return:
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_POS_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_POS_READ)
        if msg is not None:
            return msg

def getBusServoTemp(id):
    '''
    Read the current temperature of the servo
    :param id:
    :return:
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_TEMP_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_TEMP_READ)
        if msg is not None:
            return msg

def getBusServoVin(id):
    '''
    Read the current voltage of the servo
    :param id: Servo id
    :return:
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_VIN_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_VIN_READ)
        if msg is not None:
            return msg

def restBusServoPulse(oldid):
    # Servo clear deviation and P value median（500）
    serial_servo_set_deviation(oldid, 0)    # Clear deviation
    time.sleep(0.1)
    serial_serro_wirte_cmd(oldid, LOBOT_SERVO_MOVE_TIME_WRITE, 500, 100)    # Median

## Power on/off
def unloadBusServo(id):
    serial_serro_wirte_cmd(id, LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, 0)

## Read whether the power is off
def getBusServoLoadStatus(id):
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_LOAD_OR_UNLOAD_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_LOAD_OR_UNLOAD_READ)
        if msg is not None:
            return msg


if __name__ == '__main__':
    pass
