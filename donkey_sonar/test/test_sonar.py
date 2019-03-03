#!/usr/bin/python
#-- FROM https://pimylifeup.com/raspberry-pi-distance-sensor/
import RPi.GPIO as GPIO
import time

try:
      GPIO.setmode(GPIO.BCM)

      #-- FRONT
      PIN_TRIGGER = 5
      PIN_ECHO = 6
      
      #-- RIGHT
      # PIN_TRIGGER = 27
      # PIN_ECHO = 22

      #-- LEFT
      # PIN_TRIGGER = 4
      # PIN_ECHO = 17

      GPIO.setup(PIN_TRIGGER, GPIO.OUT)
      GPIO.setup(PIN_ECHO, GPIO.IN)

      GPIO.output(PIN_TRIGGER, GPIO.LOW)

      print "Waiting for sensor to settle"

      time.sleep(2)

      print "Calculating distance"

      while True:
          GPIO.output(PIN_TRIGGER, GPIO.HIGH)

          time.sleep(0.00001)

          GPIO.output(PIN_TRIGGER, GPIO.LOW)

          while GPIO.input(PIN_ECHO)==0:
                pulse_start_time = time.time()
          while GPIO.input(PIN_ECHO)==1:
                pulse_end_time = time.time()

          pulse_duration = pulse_end_time - pulse_start_time
          distance = round(pulse_duration * 17150, 2)
          print "Distance:",distance,"cm"

finally:
      GPIO.cleanup()