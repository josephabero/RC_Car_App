#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import RPi.GPIO as gpio
from time import sleep
from gpio_test import gpioSetup

# PIN       sys.argv[1]
# FREQ      sys.argv[2]
# DUTY CYC  sys.argv[3]

print("pwm.py: " + sys.argv[1] + " " + sys.argv[2] + " " + sys.argv[3])


PIN       = int(sys.argv[1])
FREQ      = float(sys.argv[2])
DUTY_CYC  = float(sys.argv[3])

gpioSetup()

print(str(PIN) + " " + str(FREQ) + " " + str(DUTY_CYC))

pwm = gpio.PWM(PIN, FREQ)
pwm.start(0)

while True:
    # print(DUTY_CYC)
    pwm.ChangeDutyCycle(DUTY_CYC)
    # for duty in range(0,101,1):
        # print(duty)
        # pwm.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
        # sleep(0.01)
    # sleep(0.5)  
    # for duty in range(100,-1,-1):
        # print(duty)
        # pwm.ChangeDutyCycle(duty)
        # sleep(0.01)
    sleep(0.5)
    
# 18 : Left
# 12 : Back
# 13 : Forward
# 19 : Right