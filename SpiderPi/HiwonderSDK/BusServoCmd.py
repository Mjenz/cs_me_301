#!/usr/bin/env python3
# encoding: utf-8
import time
import serial
import ctypes
import RPi.GPIO as GPIO
#Bus Servo Communication#
# Wrapper that sends commands to the bus servo controller

LOBOT_SERVO_FRAME_HEADER         = 0x55
LOBOT_SERVO_MOVE_TIME_WRITE      = 1
LOBOT_SERVO_MOVE_TIME_READ       = 2
LOBOT_SERVO_MOVE_TIME_WAIT_WRITE = 7
LOBOT_SERVO_MOVE_TIME_WAIT_READ  = 8
LOBOT_SERVO_MOVE_START           = 11
LOBOT_SERVO_MOVE_STOP            = 12
LOBOT_SERVO_ID_WRITE             = 13
LOBOT_SERVO_ID_READ              = 14
LOBOT_SERVO_ANGLE_OFFSET_ADJUST  = 17
LOBOT_SERVO_ANGLE_OFFSET_WRITE   = 18
LOBOT_SERVO_ANGLE_OFFSET_READ    = 19
LOBOT_SERVO_ANGLE_LIMIT_WRITE    = 20
LOBOT_SERVO_ANGLE_LIMIT_READ     = 21
LOBOT_SERVO_VIN_LIMIT_WRITE      = 22
LOBOT_SERVO_VIN_LIMIT_READ       = 23
LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE = 24
LOBOT_SERVO_TEMP_MAX_LIMIT_READ  = 25
LOBOT_SERVO_TEMP_READ            = 26
LOBOT_SERVO_VIN_READ             = 27
LOBOT_SERVO_POS_READ             = 28
LOBOT_SERVO_OR_MOTOR_MODE_WRITE  = 29
LOBOT_SERVO_OR_MOTOR_MODE_READ   = 30
LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE = 31
LOBOT_SERVO_LOAD_OR_UNLOAD_READ  = 32
LOBOT_SERVO_LED_CTRL_WRITE       = 33
LOBOT_SERVO_LED_CTRL_READ        = 34
LOBOT_SERVO_LED_ERROR_WRITE      = 35
LOBOT_SERVO_LED_ERROR_READ       = 36

rx_pin = 7
tx_pin = 13

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

serialHandle = serial.Serial("/dev/ttyAMA0", 115200)  # Open the serial port at 115200 baud (1 baud = 1 bit per second)

def portInit():  # Configure the GPIO port used
    GPIO.setup(rx_pin, GPIO.OUT)  # Configure RX_CON that is GPIO17 as output
    GPIO.output(rx_pin, 0)
    GPIO.setup(tx_pin, GPIO.OUT)  # Configure TX_CON that is GPIO27 as output
    GPIO.output(tx_pin, 1)

portInit()

def portWrite():  # Configure the single-wire serial port as output
    GPIO.output(tx_pin, 1)  # Pull TX_CON that is GPIO27 as high
    GPIO.output(rx_pin, 0)  # Pull RX_CON that is GPIO17 as low

def portRead():  # Configure the single-wire serial port as input
    GPIO.output(rx_pin, 1)  # Pull RX_CON that is GPIO17 as high
    GPIO.output(tx_pin, 0)  # Pull X_CON that is GPIO27 as low

def portRest():
    time.sleep(0.1)
    serialHandle.close()
    GPIO.output(rx_pin, 1)
    GPIO.output(tx_pin, 1)
    serialHandle.open()
    time.sleep(0.1)

def checksum(buf):
    # Calculate the checksum
    sum = 0x00
    for b in buf:  # Sum
        sum += b
    sum = sum - 0x55 - 0x55  # Remove the 2 0x55 at the beginning
    sum = ~sum  # Negate
    return sum & 0xff

def serial_serro_wirte_cmd(id=None, w_cmd=None, dat1=None, dat2=None):
    '''
    Write command
    :param id: Servo id, default is None
    :param w_cmd: Command, default is None
    :param dat1: 
    :param dat2:
    :return:
    '''
    portWrite()
    buf = bytearray(b'\x55\x55')  # Header
    buf.append(id)
    # Command length
    if dat1 is None and dat2 is None:
        buf.append(3)
    elif dat1 is not None and dat2 is None:
        buf.append(4)
    elif dat1 is not None and dat2 is not None:
        buf.append(7)

    buf.append(w_cmd)  # Command
    # Write data
    if dat1 is None and dat2 is None:
        pass
    elif dat1 is not None and dat2 is None:
        buf.append(dat1 & 0xff)  # Deviation
    elif dat1 is not None and dat2 is not None:
        buf.extend([(0xff & dat1), (0xff & (dat1 >> 8))])  # 分低8位 高8位 放入缓存
        buf.extend([(0xff & dat2), (0xff & (dat2 >> 8))])  # 分低8位 高8位 放入缓存
    # Checksum
    buf.append(checksum(buf))
    # for i in buf:
    #     print('%x' %i)
    serialHandle.write(buf)  # Write data (send)

def serial_servo_read_cmd(id=None, r_cmd=None):
    '''
    Send read command
    :param id: Servo id, default is None
    :param r_cmd: Read command, default is None
    :param dat:
    :return:
    '''
    portWrite()
    buf = bytearray(b'\x55\x55')  # Header
    buf.append(id)
    buf.append(3)  # Command length
    buf.append(r_cmd)  # Append command
    buf.append(checksum(buf))  # checksum
    serialHandle.write(buf)  # Write data (send)
    time.sleep(0.00034)

def serial_servo_get_rmsg(cmd):
    '''
    # Get the content of the read command
    :param cmd: Read command
    :return: Data
    '''
    serialHandle.flushInput()  # Clear the cache
    portRead()  # Configure the single-wire serial port to input
    time.sleep(0.005)  # Delay 5ms to wait for the data to be sent
    count = serialHandle.inWaiting()    # Get the number of bytes received
    if count != 0:  # if the number of bytes received is not 0
        recv_data = serialHandle.read(count)  # Read the received data
        # for i in recv_data:
        #     print('%#x' %ord(i))
        # Is it a read id command
        try:
            if recv_data[0] == 0x55 and recv_data[1] == 0x55 and recv_data[4] == cmd:
                dat_len = recv_data[3]
                serialHandle.flushInput()  # Clear the cache
                if dat_len == 4:
                    # print ctypes.c_int8(ord(recv_data[5])).value    # Converted to signed integer
                    return recv_data[5]
                elif dat_len == 5:
                    pos = 0xffff & (recv_data[5] | (0xff00 & (recv_data[6] << 8)))
                    return ctypes.c_int16(pos).value
                elif dat_len == 7:
                    pos1 = 0xffff & (recv_data[5] | (0xff00 & (recv_data[6] << 8)))
                    pos2 = 0xffff & (recv_data[7] | (0xff00 & (recv_data[8] << 8)))
                    return ctypes.c_int16(pos1).value, ctypes.c_int16(pos2).value
            else:
                return None
        except BaseException as e:
            print(e)
    else:
        serialHandle.flushInput()  # Clear the cache
        return None
