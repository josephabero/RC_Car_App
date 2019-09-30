#!/usr/bin/env python
# -*- coding: utf-8 -*-

import paho.mqtt.client as mqtt
import RPi.GPIO as gpio
import subprocess

LEFT_GPIO       = 18
BACK_GPIO       = 12
FORWARD_GPIO    = 13
RIGHT_GPIO      = 19



def gpioSetup():
    
    gpio.setmode(gpio.BCM)
    gpio.setup(LEFT_GPIO, gpio.OUT)
    gpio.setup(RIGHT_GPIO, gpio.OUT)
    gpio.setup(FORWARD_GPIO, gpio.OUT)
    gpio.setup(BACK_GPIO, gpio.OUT)
    
    pwmLeft = gpio.PWM(LEFT_GPIO, 200)
    pwmLeft.start(0)
    pwmRight = gpio.PWM(RIGHT_GPIO, 200)
    pwmRight.start(0)
    pwmForward = gpio.PWM(FORWARD_GPIO, 200)
    pwmForward.start(0)
    pwmBack = gpio.PWM(BACK_GPIO, 200)
    pwmBack.start(0)

def parseMessage(message):
    return message.split()[1], message.split()[3], message.split()[5]
    
    
    # for word in message.split():
    #   if(word == "button") 

def connectionStatus(client, userdata, flags, rc):
    if rc == 0:
        print("Connected!")
        print(client.subscribe("rpi/gpio"))
    

def messageDecoder(client, userdata, msg):
    message = msg.payload.decode(encoding = 'UTF-8')
    print(message)
    buttonVal, dCycVal, freqVal = parseMessage(message)
    
    print (buttonVal + ": " + str(buttonVal == "rightOn"))
    
    if buttonVal == "rightOn":
        # gpio.output(RIGHT_GPIO, gpio.HIGH)
        # pwmRight.changeDutyCycle(100)
        # execfile("pwmTest.py")
        # subprocess.call("~/Documents/RC_Car_App/pwmTest.py", shell=True)
        pwmTest = subprocess.Popen(["python", "pwmTest.py", freqVal, dCycVal]) 
        
        poll = pwmTest.poll()
        if poll == None:
            print("Right LED is ON")
        
    # elif buttonVal == "rightOff":
        # gpio.output(RIGHT_GPIO, gpio.LOW)
        # # pwmRight.changeDutyCycle(0)
        # print("Right LED is OFF")
        
    # elif buttonVal == "backOn":
        # gpio.output(BACK_GPIO, gpio.HIGH)
        # # pwmBack.changeDutyCycle(100)
        # print("Back LED is ON")
        
    # elif buttonVal == "backOff":
        # gpio.output(BACK_GPIO, gpio.LOW)
        # # pwmBack.changeDutyCycle(0)
        # print("Back LED is OFF")
        
    # elif buttonVal == "forwardOn":
        # gpio.output(FORWARD_GPIO, gpio.HIGH)
        # # pwmForward.changeDutyCycle(100)
        # print("Forward LED is ON")
        
    # elif buttonVal == "forwardOff":
        # gpio.output(FORWARD_GPIO, gpio.LOW)
        # print("Forward LED is OFF")
        
    # elif buttonVal == "leftOn":
        # gpio.output(LEFT_GPIO, gpio.HIGH)
        # print("Left LED is ON")
        
    # elif buttonVal == "leftOff":
        # gpio.output(LEFT_GPIO, gpio.LOW)
        # print("Left LED is OFF")
        
    # elif buttonVal == "leftLong":
        
        # print("Left Long Pressed. Attempting PWM")
        
    else:
        print("Unknown message!: " + message)

print("Setting Up!")
gpioSetup()

clientName = "RPIO3B"
serverAddress = "192.168.0.32"
mqttClient = mqtt.Client(clientName)

print(mqttClient)
mqttClient.on_connect = connectionStatus
mqttClient.on_message = messageDecoder

print("Connecting to Server! " + serverAddress)
mqttClient.connect(serverAddress)

print("Looping!")
mqttClient.loop_forever()
