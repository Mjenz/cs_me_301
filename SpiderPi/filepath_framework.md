# File Structure Framework

This details the framework for the filepath of the SpiderPi project.

<pre>
|--cs_me_301
     |--SpiderPi 
          |--HiwonderSDK   # contains code for the Hiwonder SDK
               |--__init__.py
               |--Board.py   # contains the functions needed to control the robot; will need to learn this very well to understand how to control the robot
               |--BusServoCmd.py   # wrapper that writes commands to the bus servo and used by Board.py
               |--BusServoMoveDemo.py  # example for moving the bus servos
               |--LatticeDisplay.py  # example for controlling the LED display
               |--Mpu6050.py   # contains the functions needed to read the IMU data
               |--PWMServoCmd.py   # wrapper that writes commands to the PWM servo and used by Board.py
               |--PWMServoMoveDemo.py  # example for moving the PWM servos
               |--ReadServoDemo.py  # example for reading the bus servo data
               |--Sonar.py  # wrapper that reads and writes to the ultrasonic sensor data; will need to edit this to work with multiple sensors
               |--tm1640.py   # contains the functions needed to control the LED display
          |--__init__.py
          |--lab_config.yaml   # example for writing yaml files
          |--servo_config.yaml   # example for writing yaml files
          |--yaml_handle.py   # functions for reading and writing yaml files
               
</pre>