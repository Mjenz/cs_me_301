#!/usr/bin/env python3
import os
import sys
import time
import tm1640 as tm

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

print('''
**********************************************************
********Dot Matrix Display Experimental Routine*********
**********************************************************
----------------------------------------------------------
The following commands need to be used in the terminal.
      The terminal can be opened through  ctrl+alt+t,
      or click on the black icon in the upper bar.
----------------------------------------------------------
Run the program:
    sudo python3 LatticeDisplayDemo.py
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close the program. If it fails,
      please try multiple times！
----------------------------------------------------------
''')

## Print/Display 'Hello'
tm.display_buf = (0x7f, 0x08, 0x7f, 0x00, 0x7c, 0x54, 0x5c, 0x00,
                  0x7c, 0x40, 0x00,0x7c, 0x40, 0x38, 0x44, 0x38)

tm.update_display()

time.sleep(5)
tm.display_buf = [0] * 16
tm.update_display()
