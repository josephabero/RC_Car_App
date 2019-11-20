#!/usr/bin/env python
# -*- coding: utf-8 -*-


# import the necessary packages
import paho.mqtt.client as mqtt
import RPi.GPIO as gpio
import subprocess
import time
from time import sleep
import os

from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import sys
from numpy import arange


# MANUAL CONTROL VARS
STEER_SERVO         = 12
THROTTLE_SERVO      = 18
CURRENT_DIR         = 0
CURRENT_SPEED       = 0

processList = {STEER_SERVO: 0, THROTTLE_SERVO: 0}
pwmList = {}

prev = 0

# Determines which type of control: True = manual, False = auto
manualControl = False

# AUTO CONTROL VARS

# Parse arguments
parsedArgs = argparse.ArgumentParser()
parsedArgs.add_argument("-v", "--video",help="path to the (optional) video file")
parsedArgs.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
finalArgs = vars(parsedArgs.parse_args())

# lower and upper boundaries of the tennis ball
tennisLower = (29, 86, 6)
tennisUpper = (64, 255, 255)
pts = deque(maxlen=finalArgs["buffer"])

# if a video path was not supplied, use webcam reference
if not finalArgs.get("video", False):
        vs = VideoStream(src=0).start()

# or use a video file reference
else:
        vs = cv2.VideoCapture(finalArgs["video"])

    # allow the camera or video file to warm up
time.sleep(2.0)

# OpenCV function
# Should be called repeatedly in a loop
def getch():
        global prev

        # store current frame
        currentFrame = vs.read()

        frame = currentFrame[1] if finalArgs.get("video", False) else currentFrame


        # resize the frame, blur it, and convert it to the HSV color
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # erode and dilate mask for the color desired (green)
        mask = cv2.inRange(hsv, tennisLower, tennisUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)


        # Use OpenCV to find the countours
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        center = None

        # check if a countour was found
        if len(contours) > 0:
                # use largest countour to find minimum enclosing circle
                largestContour = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(largestContour)
                moment = cv2.moments(largestContour)
                center = (int(moment["m10"] / moment["m00"]), int(moment["m01"] / moment["m00"]))
              #  print(center)
                # only proceed if the radius meets a minimum size
                if radius > 10:
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(frame, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
                        cv2.circle(frame, center, 5, (0, 0, 255), -1)
                        
                        # Use center point to determine direction the motors should turn
                        # TURN LEFT
                        if (center[0] >= 0 and center[0] < 100):
                            if prev != 1:
                                prev = 1
                                print("left " + str(prev))
                                turnLeft()
                        # GO FORWARD
                        elif (center[0] >= 100 and center[0] < 440):
                            if prev != 2:
                                prev = 2
                                print("forward " + str(prev))
                                centerDir()
                        # TURN RIGHT
                        elif (center[0] >= 440 and center[0] < 601):
                            if prev != 3:
                                prev = 3
                                print("right " + str(prev))
                                turnRight()
                        else:
                            prev = 0
                else:
                    print("ball not found")
                    sleep(0.5)
                sleep(.1)

        # show the frame on the screen
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF


def gpioSetup():
    gpio.setwarnings(False)
    gpio.setmode(gpio.BCM)
    gpio.setup(STEER_SERVO, gpio.OUT)
    gpio.setup(THROTTLE_SERVO, gpio.OUT)

    print("Initializing PWM")
    initializePWM(STEER_SERVO,      50, 0)
    initializePWM(THROTTLE_SERVO,   50, 0)


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

def resetgpio():
    gpio.output(THROTTLE_SERVO, gpio.LOW)
    gpio.output(STEER_SERVO,    gpio.LOW)

def turnRight():
    global STEER_SERVO, CURRENT_DIR, pwmList
    
    pwmList[STEER_SERVO].ChangeDutyCycle(10)
    sleep(.1)
    return

def turnLeft():
    global STEER_SERVO, CURRENT_DIR

    pwmList[STEER_SERVO].ChangeDutyCycle(5)
    sleep(.1)
    return

def centerDir():
    global STEER_SERVO, CURRENT_DIR

    pwmList[STEER_SERVO].ChangeDutyCycle(7.55)
    sleep(.1)
    return

def move():
    global THROTTLE_SERVO, CURRENT_SPEED

    pwmList[THROTTLE_SERVO].ChangeDutyCycle(8.15)
    CURRENT_SPEED = maxSpeed

def stop():
    global THROTTLE_SERVO, CURRENT_SPEED

    pwmList[THROTTLE_SERVO].ChangeDutyCycle(7.5)
    CURRENT_SPEED = stopSpeed

def messageDecoder(client, userdata, msg):
    global manualControl

    message = msg.payload.decode(encoding = 'UTF-8')
    # print("Message: " + message)
    # buttonVal, dCycVal, freqVal = parseMessage(message)
    buttonVal = parseMessage(message)
    print(buttonVal)


    validMessage = True

    if buttonVal == "manual":
        manualControl = True
    elif buttonVal == "auto":
        manualControl = False

    # print(pwmList)
    if manualControl:
        # BASIC gpio TESTING
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

def on_disconnect(client, userdata,rc=0):
    logging.debug("DisConnected result code "+str(rc))
    client.loop_stop()


def main():
    print("Setting Up INTEGRATION TEST!")
    gpioSetup()

    clientName = "RPIO3B"
    serverAddress = "172.20.10.10"
    mqttClient = mqtt.Client(clientName)

    mqttClient.on_connect = connectionStatus
    mqttClient.on_message = messageDecoder

    print("Connecting to Server! " + serverAddress)
    mqttClient.connect(serverAddress)

    print("Looping!")
    mqttClient.loop_start()

    # Continue looping forever

    # If button is pressed by iOS, callback (messageDecoder) will be triggered to handle button press
    # Otherwise, keep running OpenCV function
    while True:
        if manualControl == False:  getch()
        else:                       sleep(0.5)

if __name__ == '__main__':
    main()
