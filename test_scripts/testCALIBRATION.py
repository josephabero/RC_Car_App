#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import RPi.GPIO as GPIO
from time import sleep
from numpy import arange

ledpin = 18			# PWM pin connected to LED
GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BCM)		#set pin numbering system
GPIO.setup(ledpin,GPIO.OUT)
pi_pwm = GPIO.PWM(ledpin,50)		#create PWM instance with frequency
pi_pwm.start(0)				#start PWM of required Duty Cycle 

# sleep(2)
print("setting high duty cycle")
pi_pwm.ChangeDutyCycle(10);
sleep(3)
print("setting low duty cycle")
pi_pwm.ChangeDutyCycle(5);
sleep(10)
