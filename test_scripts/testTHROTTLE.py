#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import RPi.GPIO as GPIO
from time import sleep
from numpy import arange

# print sys.argv[2]

# PIN
# FREQ
# DUTY CYC

ledpin = 18			# PWM pin connected to LED
GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BCM)		#set pin numbering system
GPIO.setup(ledpin,GPIO.OUT)
pi_pwm = GPIO.PWM(ledpin,50)		#create PWM instance with frequency
pi_pwm.start(0)				#start PWM of required Duty Cycle 

# sleep(2)
# print("setting high duty cycle")
# pi_pwm.ChangeDutyCycle(10);
# sleep(10)
# print("setting low duty cycle")
# pi_pwm.ChangeDutyCycle(5);
# sleep(10)

while True:
    # print("--------")
    # print("0 - 100")
    # for duty in range(0,10,1):
        # print(duty)
        # pi_pwm.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
        # sleep(0.01)
    # sleep(1)  
    # print("forward")
    # for duty in arange(7, 8, .25):
        # pi_pwm.ChangeDutyCycle(duty)
        # sleep(0.01)
    # sleep(5)
    
    print("super forward")
    pi_pwm.ChangeDutyCycle(9)
    sleep(3)
    print("forward")
    pi_pwm.ChangeDutyCycle(8.5)
    sleep(3)
    
    print("slowly forward")
    pi_pwm.ChangeDutyCycle(8)
    sleep(3)
    
    print("stop")
    pi_pwm.ChangeDutyCycle(7.5)

    sleep(3)
    print("back")
    pi_pwm.ChangeDutyCycle(7)
    sleep(3)
    
    print("super back")
    pi_pwm.ChangeDutyCycle(5)
    sleep(2)
    
    print("stop")
    pi_pwm.ChangeDutyCycle(7.5)
    sleep(3)

    

    
    
    
    
# 18 : Left
# 12 : Back
# 13 : Forward
# 19 : Right