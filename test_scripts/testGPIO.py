#!/usr/bin/env python
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
from time import sleep

print("GPIO TEST")

ledpin = 18			# PWM pin connected to LED
GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BCM)		#set pin numbering system
GPIO.setup(ledpin,GPIO.OUT)

while True:
    print("HIGH")
    GPIO.output(ledpin, GPIO.HIGH)
    sleep(0.5)
    print("LOW")
    GPIO.output(ledpin, GPIO.LOW)
    sleep(0.5)