#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import RPi.GPIO as GPIO
from time import sleep

# print sys.argv[2]

# PIN
# FREQ
# DUTY CYC

ledpin = 18			# PWM pin connected to LED
GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BCM)		#set pin numbering system
GPIO.setup(ledpin,GPIO.OUT)
pi_pwm = GPIO.PWM(ledpin,1000)		#create PWM instance with frequency
pi_pwm.start(0)				#start PWM of required Duty Cycle 

# pi_pwm.ChangeDutyCycle(100)
while True:
    # pi_pwm.ChangeDutyCycle(25)
    for duty in range(0,101,1):
        print(duty)
        pi_pwm.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
        sleep(0.01)
    sleep(0.5)  
    for duty in range(100,-1,-1):
        print(duty)
        pi_pwm.ChangeDutyCycle(duty)
        sleep(0.01)
    sleep(0.5)
    
# 18 : Left
# 12 : Back
# 13 : Forward
# 19 : Right