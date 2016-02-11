servoblaster.py: 

library for handling servoblaster library (https://github.com/tizianofiorenzani/ros_tutorials.git). For installing follow the tutorial here: https://www.leenabot.com/Servo-Motor-driver/



     Servo number    GPIO number   Pin in P1 header
          0               4             P1-7
          1              17             P1-11
          2              18             P1-12
          3             21/27           P1-13
          4              22             P1-15
          5              23             P1-16
          6              24             P1-18
          7              25             P1-22

P1-13 is connected to either GPIO-21 or GPIO-27, depending on board revision.

after installation (just "make" and "make install") edit the config file
/etc/init.d/servoblaster

set in the OPTS="your_options". Mine are
--idle-timeout=2000 --pcm --p1pins=0,0,0,0,15,16,0,0
so you have 2 seconds timeout before setting to idle, pcm mode (rather than pwm) and only pins 15 and 16 set for the servo (gpio 22 and 23)

tfmini.py
Interface to tfmini laser


tfmini_servo_scanner.py
Class for a laser scanner
