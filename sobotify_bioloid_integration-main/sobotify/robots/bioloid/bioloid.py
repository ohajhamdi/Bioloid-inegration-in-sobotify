#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Attribution: Part of this code is based on 
https://github.com/hhuebert/sobotify/blob/main/sobotify/robots/mykeepon/mykeepon.py
(MIT Licensed)
"""

import serial
import struct
import time
import math
import sobotify.robots.robot as default_robot

class Bioloid(): 

    def __init__(self,PORT,cam_device,sound_device):
        self.motion=motion(PORT)
        self.speech=speech()
        self.vision=vision(cam_device)
        self.sound=sound(sound_device)

    def terminate(self):
        self.motion.terminate()
        self.speech.terminate()
        self.vision.terminate()
        self.sound.terminate()


def send_motor_positions(motor1, motor2, motor3, motor4, motor5, motor6):
    data = struct.pack('<HHHHHH', motor1, motor2, motor3, motor4, motor5, motor6)
    return data



class motion(default_robot.motion): 

    def __init__(self,PORT):
        super().__init__()
        self.fileExtension = "_bioloid" 
        self.bio=serial.Serial(PORT,57600,timeout=1)
        

    def move(self,line):
    
        self.bio.write(send_motor_positions(int(line[0]), int(line[1]), int(line[2]), int(line[3]), int(line[4]), int(line[5])))
        print(line)
            
    def terminate(self):
        pass
        #self.myKeepOn.close()

class speech(default_robot.speech):
    pass

class vision(default_robot.vision):
    pass

class sound(default_robot.sound):
    pass