#!/usr/bin/env python
# -*- coding: utf-8 -*-

import paho.mqtt.client as mqtt
import RPi.GPIO as gpio
import subprocess
from time import sleep
import os

LEFT_GPIO       = 18
BACK_GPIO       = 12
FORWARD_GPIO    = 13
RIGHT_GPIO      = 19

processList = {LEFT_GPIO: 0, RIGHT_GPIO: 0}
pwmList = []

def gpioSetup():

    print(LEFT_GPIO)
    print(RIGHT_GPIO)
    print(FORWARD_GPIO)
    print(BACK_GPIO)
    
    gpio.setwarnings(False)
    gpio.setmode(gpio.BCM)
    gpio.setup(LEFT_GPIO, gpio.OUT)
    gpio.setup(RIGHT_GPIO, gpio.OUT)
    gpio.setup(FORWARD_GPIO, gpio.OUT)
    gpio.setup(BACK_GPIO, gpio.OUT)
    
    print("Initializing PWM")
    initializePWM(LEFT_GPIO,    100, 0)
    initializePWM(RIGHT_GPIO,   100, 0)
    initializePWM(FORWARD_GPIO, 100, 0)
    initializePWM(BACK_GPIO,    100, 0)


def parseMessage(message):
    # return message.split()[1], message.split()[3], message.split()[5]
    return message.split()[1]

def runPWM(gpioPin, dutyCycle, freq):
    print("Set PWM")
    
    if(processList[gpioPin] != 0):
        os.system("sudo kill " + str(processList[gpioPin]))
    processPWM = subprocess.Popen(["python", "pwm.py", str(gpioPin), str(freq), str(dutyCycle)])
    
    processList[gpioPin] = processPWM.pid
    # print(processPWM.pid)
    return processPWM.pid

def connectionStatus(client, userdata, flags, rc):
    if rc == 0:
        print("Connected!")
        # print(client.subscribe("rpi/gpio"))
        client.subscribe("rpi/gpio")
        print("Subscribed to topic: rpi/gpio")
    
def initializePWM(gpioPin, freq, dutyCycle):
    pwm = gpio.PWM(gpioPin, freq)
    pwm.start(dutyCycle)
    pwmList.append(pwm);
    
def resetGPIO():
    gpio.output(RIGHT_GPIO, gpio.LOW)
    gpio.output(BACK_GPIO, gpio.LOW)
    gpio.output(FORWARD_GPIO, gpio.LOW)
    gpio.output(LEFT_GPIO, gpio.LOW)

def messageDecoder(client, userdata, msg):
    message = msg.payload.decode(encoding = 'UTF-8')
    # print("Message: " + message)
    # buttonVal, dCycVal, freqVal = parseMessage(message)
    buttonVal = parseMessage(message)
    dCycVal = 0
    freqVal = 0
    gpioPin = 12
    
    validMessage = True
    
    # # BASIC GPIO TESTING
    # if buttonVal == "rButton":
        # print("Right LED is ON")
        # gpio.output(RIGHT_GPIO, gpio.HIGH)
        # gpioPin = RIGHT_GPIO
    # elif buttonVal == "lButton":
        # print("Left LED is ON")
        # gpio.output(LEFT_GPIO, gpio.HIGH)
        # gpioPin = LEFT_GPIO
    # elif buttonVal == "bButton":
        # print("Back LED is ON")
        # gpio.output(BACK_GPIO, gpio.HIGH)
        # gpioPin = BACK_GPIO
    # elif buttonVal == "fButton":
        # print("Forward LED is ON")
        # gpio.output(FORWARD_GPIO, gpio.HIGH)
        # gpioPin = FORWARD_GPIO
    # elif buttonVal == "sButton":
        # print("Stop LED is ON")
        # gpio.output(FORWARD_GPIO, gpio.HIGH)
        # gpioPin = FORWARD_GPIO
    print(pwmList
        
    # BASIC GPIO TESTING
    if buttonVal == "rButton":
        # print("Turning RIGHT")
        print("Right LED")
        # gpio.output(RIGHT_GPIO, gpio.HIGH)
        # gpioPin = RIGHT_GPIO
        # dCycVal = 130
        
    elif buttonVal == "lButton":
        print("Left LED")
        gpio.output(LEFT_GPIO, gpio.HIGH)
        gpioPin = LEFT_GPIO
    elif buttonVal == "bButton":
        print("Back LED")
        gpio.output(BACK_GPIO, gpio.HIGH)
        gpioPin = BACK_GPIO
    elif buttonVal == "fButton":
        print("Forward LED")
        gpio.output(FORWARD_GPIO, gpio.HIGH)
        gpioPin = FORWARD_GPIO
    elif buttonVal == "sButton":
        print("Reset All LEDs")
        # gpio.output(FORWARD_GPIO, gpio.HIGH)
        # gpioPin = FORWARD_GPIO
        resetGPIO()
    
    
    # # LEGACY GPIO TESTING
    # if buttonVal == "rightOn":
        # gpio.output(RIGHT_GPIO, gpio.HIGH)
        # pwmRight.changeDutyCycle(100)
        # execfile("pwmTest.py")
        # subprocess.call("~/Documents/RC_Car_App/pwmTest.py", shell=True)
        # gpioPin = RIGHT_GPIO
        # print("Right LED is ON")
        
    # elif buttonVal == "rightOff":
        # gpio.output(RIGHT_GPIO, gpio.LOW)
        # pwmRight.changeDutyCycle(0)
        # gpioPin = RIGHT_GPIO
        # print("Right LED is OFF")
        
    # elif buttonVal == "backOn":
        # gpio.output(BACK_GPIO, gpio.HIGH)
        # pwmBack.changeDutyCycle(100)
        # gpioPin = BACK_GPIO
        # print("Back LED is ON")
        
    # elif buttonVal == "backOff":
        # gpio.output(BACK_GPIO, gpio.LOW)
        # pwmBack.changeDutyCycle(0)
        # gpioPin = BACK_GPIO
        # print("Back LED is OFF")
        
    # elif buttonVal == "forwardOn":
        # gpio.output(FORWARD_GPIO, gpio.HIGH)
        # pwmForward.changeDutyCycle(100)
        # gpioPin = FORWARD_GPIO
        # print("Forward LED is ON")
        
    # elif buttonVal == "forwardOff":
        # gpio.output(FORWARD_GPIO, gpio.LOW)
        # gpioPin = FORWARD_GPIO
        # print("Forward LED is OFF")
        
    # elif buttonVal == "leftOn":
        # gpio.output(LEFT_GPIO, gpio.HIGH)
        # gpioPin = LEFT_GPIO
        # print("Left LED is ON")
        
    # elif buttonVal == "leftOff":
        # gpio.output(LEFT_GPIO, gpio.LOW)
        # gpioPin = LEFT_GPIO
        # print("Left LED is OFF")
        
    # elif buttonVal == "leftLong":
        
        # print("Left Long Pressed. Attempting PWM")
   
    else:
        validMessage = False
        print("Unknown message!: " + message)
        
    # if(validMessage):
        # print("Starting PWM")
        # runPWM(gpioPin, dCycVal, freqVal)
        # print("PWM set")
     
    # sleep(.1)
    # resetGPIO()
    # sleep(.1)
    # print(runPWM(gpioPin, 50, 1000))
    # print(processList)
    

def main():
    print("Setting Up!")
    gpioSetup()
    
    clientName = "RPIO3B"
    serverAddress = "192.168.0.32"
    mqttClient = mqtt.Client(clientName)
    
    mqttClient.on_connect = connectionStatus
    mqttClient.on_message = messageDecoder
    
    print("Connecting to Server! " + serverAddress)
    mqttClient.connect(serverAddress)
    
    print("Looping!")
    mqttClient.loop_forever()
    
if __name__ == '__main__':
    main()
