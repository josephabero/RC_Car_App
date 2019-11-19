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
 # construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
pts = deque(maxlen=args["buffer"])

    # if a video path was not supplied, grab the reference
    # to the webcam
if not args.get("video", False):
        vs = VideoStream(src=0).start()

    # otherwise, grab a reference to the video file
else:
        vs = cv2.VideoCapture(args["video"])

    # allow the camera or video file to warm up
time.sleep(2.0)

# def turn_left():
#     print("turning left")
#     pi_pwm.ChangeDutyCycle(5)
#     sleep(0.5)
#     #pi_pwm.ChangeDutyCycle(7.5)
#
# def turn_right():
#     print("turning right")
#     pi_pwm.ChangeDutyCycle(10)
#     sleep(0.5)
#     #pi_pwm.ChangeDutyCycle(7.5)
#
# def turn_forward():
#     print("go forward")
#     pi_pwm.ChangeDutyCycle(7)
#     sleep(0.5)

# keep looping
def getch():
        global prev
        # grab the current frame
        frame = vs.read()

        # handle the frame from VideoCapture or VideoStream
        frame = frame[1] if args.get("video", False) else frame

        # if we are viewing a video and we did not grab a frame,
        # then we have reached the end of the video
#       if frame is None:
#               break

        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)


    # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
              #  print(center)
                # only proceed if the radius meets a minimum size
                if radius > 10:
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(frame, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
                        cv2.circle(frame, center, 5, (0, 0, 255), -1)
                        #cv2.imshow("Frame", frame)
                        #print(center)
                        #print(center[0])
                        if (center[0] >= 0 and center[0] < 100):
                            # print(center[0])
                            if prev != 1:
                                prev = 1
                                print("left " + str(prev))
                                turnRight()
                        elif (center[0] >= 100 and center[0] < 440):
                            # print(center[0])
                            if prev != 2:
                                prev = 2
                                print("forward " + str(prev))
                                centerDir()
                            #sleep(1.5)
                        elif (center[0] >= 440 and center[0] < 601):
                            # print(center[0])
                            if prev != 3:
                                prev = 3
                                print("right " + str(prev))
                                turnLeft()
                        else:
                            prev = 0
                            #pi_pwm.ChangeDutyCycle(7.5)
                            # print("not found")
                            # sleep(1.5)
                else:
                    print("ball not found")
                    sleep(0.5)
                sleep(.1)

        # update the points queue
        #pts.appendleft(center)

    # loop over the set of tracked points
        #for i in range(1, len(pts)):
                # if either of the tracked points are None, ignore
                # them
                #if pts[i - 1] is None or pts[i] is None:
                        #continue

                # otherwise, compute the thickness of the line and
                # draw the connecting lines
                #thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
                #cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

        # show the frame to our screen
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        # if the 'q' key is pressed, stop the loop
    #   if key == ord("q"):
        #       break


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
    # right = 20
    # if(CURRENT_DIR <= right):
        # for duty in range(CURRENT_DIR,right,1):
            # pwmList[STEER_SERVO].ChangeDutyCycle(duty)
            # sleep(0.01)
    # else:
        # for duty in range(CURRENT_DIR,right,-1):
            # pwmList[STEER_SERVO].ChangeDutyCycle(duty)
            # sleep(0.01)
    pwmList[STEER_SERVO].ChangeDutyCycle(10)
    sleep(.1)
    return
    # CURRENT_DIR = right;

def turnLeft():
    global STEER_SERVO, CURRENT_DIR
    # left = 0
    # if(CURRENT_DIR <= left):
        # for duty in range(CURRENT_DIR,left,1):
            # pwmList[STEER_SERVO].ChangeDutyCycle(duty)
            # sleep(0.01)
    # else:
        # for duty in range(CURRENT_DIR,left,-1):
            # pwmList[STEER_SERVO].ChangeDutyCycle(duty)
            # sleep(0.01)
    pwmList[STEER_SERVO].ChangeDutyCycle(5)
    sleep(.1)
    return

def centerDir():
    global STEER_SERVO, CURRENT_DIR

    # center = 5
    # if(CURRENT_DIR <= center):
        # for duty in range(CURRENT_DIR,center,1):
            # pwmList[STEER_SERVO].ChangeDutyCycle(duty)
            # sleep(0.01)
    # else:
        # for duty in range(CURRENT_DIR,center,-1):
            # pwmList[STEER_SERVO].ChangeDutyCycle(duty)
            # sleep(0.01)
    pwmList[STEER_SERVO].ChangeDutyCycle(7.55)
    sleep(.1)
    return

def move():
    global THROTTLE_SERVO, CURRENT_SPEED

    # maxSpeed = 20
    # for duty in range(CURRENT_SPEED,maxSpeed,1):
        # pwmList[THROTTLE_SERVO].ChangeDutyCycle(duty)
        # sleep(0.01)

    pwmList[THROTTLE_SERVO].ChangeDutyCycle(8.15)
    CURRENT_SPEED = maxSpeed

def stop():
    global THROTTLE_SERVO, CURRENT_SPEED

    # stopSpeed = 0
    # for duty in range(CURRENT_SPEED,stopSpeed,-1):
        # pwmList[THROTTLE_SERVO].ChangeDutyCycle(duty)
        # sleep(0.01)

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

    print("after loop start")

    while True:
        if manualControl == False:  getch()
        else:                       sleep(0.5)

if __name__ == '__main__':
    main()
