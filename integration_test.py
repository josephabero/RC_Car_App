#!/usr/bin/env python
# -*- coding: utf-8 -*-

import paho.mqtt.client as mqtt
import RPi.GPIO as gpio
import subprocess
from time import sleep
import os

STEER_SERVO         = 18
THROTTLE_SERVO      = 12
CURRENT_DIR         = 0
CURRENT_SPEED       = 0

processList = {STEER_SERVO: 0, THROTTLE_SERVO: 0}
pwmList = {}

def gpioSetup():
    gpio.setwarnings(False)
    gpio.setmode(gpio.BCM)
    gpio.setup(STEER_SERVO, gpio.OUT)
    gpio.setup(THROTTLE_SERVO, gpio.OUT)
    
    print("Initializing PWM")
    initializePWM(STEER_SERVO,      100, 0)
    initializePWM(THROTTLE_SERVO,   100, 0)


def parseMessage(message):
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
    pwmList[gpioPin] = pwm
    
def resetGPIO():
    gpio.output(THROTTLE_SERVO, gpio.LOW)
    gpio.output(STEER_SERVO,    gpio.LOW)

def turnRight():
    global STEER_SERVO, CURRENT_DIR
    right = 20
    if(CURRENT_DIR <= right):
        for duty in range(CURRENT_DIR,right,1):
            pwmList[STEER_SERVO].ChangeDutyCycle(duty)
            sleep(0.01)
    else:
        for duty in range(CURRENT_DIR,right,-1):
            pwmList[STEER_SERVO].ChangeDutyCycle(duty)
            sleep(0.01)
    
    CURRENT_DIR = right;

def turnLeft():
    global STEER_SERVO, CURRENT_DIR
    left = 0
    if(CURRENT_DIR <= left):
        for duty in range(CURRENT_DIR,left,1):
            pwmList[STEER_SERVO].ChangeDutyCycle(duty)
            sleep(0.01)
    else:
        for duty in range(CURRENT_DIR,left,-1):
            pwmList[STEER_SERVO].ChangeDutyCycle(duty)
            sleep(0.01)
    
    CURRENT_DIR = left;

def centerDir():
    global STEER_SERVO, CURRENT_DIR
    
    center = 5
    if(CURRENT_DIR <= center):
        for duty in range(CURRENT_DIR,center,1):
            pwmList[STEER_SERVO].ChangeDutyCycle(duty)
            sleep(0.01)
    else:
        for duty in range(CURRENT_DIR,center,-1):
            pwmList[STEER_SERVO].ChangeDutyCycle(duty)
            sleep(0.01)
    
    CURRENT_DIR = center;

def move():
    global THROTTLE_SERVO, CURRENT_SPEED

    maxSpeed = 20
    for duty in range(CURRENT_SPEED,maxSpeed,1):
        pwmList[THROTTLE_SERVO].ChangeDutyCycle(duty)
        sleep(0.01)
    CURRENT_SPEED = maxSpeed

def stop():
    global THROTTLE_SERVO, CURRENT_SPEED

    stopSpeed = 0
    for duty in range(CURRENT_SPEED,stopSpeed,-1):
        pwmList[THROTTLE_SERVO].ChangeDutyCycle(duty)
        sleep(0.01)
    CURRENT_SPEED = stopSpeed

def messageDecoder(client, userdata, msg):
    message = msg.payload.decode(encoding = 'UTF-8')
    # print("Message: " + message)
    # buttonVal, dCycVal, freqVal = parseMessage(message)
    buttonVal = parseMessage(message)

    
    validMessage = True
    
    # print(pwmList)
        
    # BASIC GPIO TESTING
    if buttonVal == "rButton":
        print("Turning RIGHT")
        turnRight()
        move()
        
    elif buttonVal == "lButton":
        print("Turning LEFT")
        turnLeft()
        move()
        
    elif buttonVal == "bButton":
        print("Moving BACK")
        centerDir()
        move()
        
    elif buttonVal == "fButton":
        print("Moving FORWARD")
        centerDir()
        move()
        
    elif buttonVal == "sButton":
        print("STOPPING")
        stop()
    
    else:
        validMessage = False
        print("Unknown message!: " + message)

    

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

