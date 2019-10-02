#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os

print("hello")

myCmd = 'ls'
os.system(myCmd)

testDict = {12 : 0, 13: 10}

if (testDict[12] != 0):
    print "sudo kill " + str(testDict[12])
    
if (testDict[13] != 0):
    print "sudo kill " + str(testDict[13])