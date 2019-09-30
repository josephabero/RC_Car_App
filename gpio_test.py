#!/usr/bin/env python
# -*- coding: utf-8 -*-

import paho.mqtt.client as mqtt
import RPi.GPIO as gpio


def gpioSetup():
    
    gpio.setmode(gpio.BCM)
    gpio.setup(21, gpio.OUT)
    gpio.setup(20, gpio.OUT)
    gpio.setup(16, gpio.OUT)
    gpio.setup(12, gpio.OUT)


def connectionStatus(client, userdata, flags, rc):
    if rc == 0:
        print("Connected!")
        print(client.subscribe("rpi/gpio"))
    

def messageDecoder(client, userdata, msg):
    message = msg.payload.decode(encoding = 'UTF-8')
    
    if message == "rightOn":
        gpio.output(21, gpio.HIGH)
        print("Right LED is ON")
    elif message == "rightOff":
        gpio.output(21, gpio.LOW)
        print("Right LED is OFF")
    elif message == "backOn":
        gpio.output(16, gpio.HIGH)
        print("Back LED is ON")
    elif message == "backOff":
        gpio.output(16, gpio.LOW)
        print("Back LED is OFF")
    elif message == "forwardOn":
        gpio.output(20, gpio.HIGH)
        print("Forward LED is ON")
    elif message == "forwardOff":
        gpio.output(20, gpio.LOW)
        print("Forward LED is OFF")
    elif message == "leftOn":
        gpio.output(12, gpio.HIGH)
        print("Left LED is ON")
    elif message == "leftOff":
        gpio.output(12, gpio.LOW)
        print("Left LED is OFF")
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
