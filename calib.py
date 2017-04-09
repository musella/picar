#!/usr/bin/env python

from picar import Sensors
import RPi.GPIO as GPIO


# ------------------------------------------------------------------------------------------        
if __name__ == "__main__":

    
    GPIO.setmode(GPIO.BCM)
    sensors = Sensors()

    resp=""
    while resp!="q":
        print sensors.run()
        resp=raw_input('')

    
