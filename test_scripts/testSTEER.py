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

ledpin = 12			# PWM pin connected to LED
GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BCM)		#set pin numbering system
GPIO.setup(ledpin,GPIO.OUT)
pi_pwm = GPIO.PWM(ledpin,50)		#create PWM instance with frequency
pi_pwm.start(0)				#start PWM of required Duty Cycle 
    

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
    print("right")
    pi_pwm.ChangeDutyCycle(10)
    # for duty in arange(8, 7.4, -.25):
        # print(duty)
        # pi_pwm.ChangeDutyCycle(duty)
        # sleep(0.01)
    sleep(2)
    print("forward")
    pi_pwm.ChangeDutyCycle(7.5)
    # for duty in arange(7.5, 6.9, -.25):
        # print(duty)
        # pi_pwm.ChangeDutyCycle(duty)
        # sleep(0.01)
    sleep(2)
    print("left")
    pi_pwm.ChangeDutyCycle(5)
    sleep(2)
    # print("100 - 0")
    # print("--------")
    # for duty in range(10,-1,-1):
        # print(duty)
        # pi_pwm.ChangeDutyCycle(duty)
        # sleep(0.01)
    # sleep(1)
    
    
    
    # print("0")
    # pi_pwm.ChangeDutyCycle(0)
    # sleep(0.01)
    # print("25")
    # pi_pwm.ChangeDutyCycle(25)
    # sleep(0.01)
    # print("50")
    # pi_pwm.ChangeDutyCycle(50)
    # sleep(0.01)
    # print("75")
    # pi_pwm.ChangeDutyCycle(75)
    # sleep(0.01)
    # print("100")
    # pi_pwm.ChangeDutyCycle(100)
    
    # sleep(1)
    
    # print("100")
    # pi_pwm.ChangeDutyCycle(100)
    # sleep(0.01)
    # print("75")
    # pi_pwm.ChangeDutyCycle(75)
    # sleep(0.01)
    # print("50")
    # pi_pwm.ChangeDutyCycle(50)
    # sleep(0.01)
    # print("25")
    # pi_pwm.ChangeDutyCycle(25)
    # sleep(0.01)
    # print("0")
    # pi_pwm.ChangeDutyCycle(0)
    
    # pi_pwm.ChangeDutyCycle(100)
    # sleep(1)
    
    
    
    
# 18 : Left
# 12 : Back
# 13 : Forward
# 19 : Right