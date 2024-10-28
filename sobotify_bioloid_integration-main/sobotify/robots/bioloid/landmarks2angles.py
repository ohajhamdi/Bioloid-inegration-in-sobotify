#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Attribution: Part of this code is based on 
https://github.com/Kazuhito00/mediapipe-python-sample/blob/main/sample_pose.py
(Apache 2.0 Licensed)
"""

import numpy as np
import math
from sobotify.commons.external.utils import KeypointsToAngles

keypointsToAngles = KeypointsToAngles()

"""
Attribution: The following function is taken from (and modified) pepper_approach_control_thread.py 
from https://github.com/FraPorta/pepper_openpose_teleoperation/tree/main/pepper_teleoperation
(Apache 2.0 Licensed)

also taken from (and modified) landmarks2angles.py for nao
https://github.com/hhuebert/sobotify/blob/main/sobotify/robots/nao/landmarks2angles.py
(MIT Licensed)
"""
##  function saturate_angles
#   Saturate angles before using them for controlling Pepper joints
def saturate_angles(LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll) :
    # global LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll
    # limit percentage for some angles 
    limit = 0.9
        
    ## LEFT ##
    # LShoulderPitch saturation
    if LShoulderPitch is None:
        print ("warning : LShoulderPitch value missing")
    elif LShoulderPitch < -2.0857:
        LShoulderPitch = -2.0857
    elif LShoulderPitch > 2.0857:
        LShoulderPitch = 2.0857
    
    # LShoulderRoll saturation
    if LShoulderRoll is None:
        print ("warning : LShoulderRoll value missing")
    elif LShoulderRoll < 0.0087:
        LShoulderRoll = 0.0087
    elif LShoulderRoll > 1.5620:
        LShoulderRoll = 1.5620
        
    # LElbowYaw saturation
    if LElbowYaw is None:
        print ("warning : LElbowYaw value missing")
    elif LElbowYaw < -2.0857*limit:
        LElbowYaw = -2.0857*limit
    elif LElbowYaw > 2.0857*limit:
        LElbowYaw = 2.0857*limit

    # LElbowRoll saturation
    if LElbowRoll is None:
        print ("warning : LElbowRoll value missing")
    elif LElbowRoll < -1.5620: 
        LElbowRoll = -1.5620
    elif LElbowRoll > -0.0087:
        LElbowRoll = -0.0087

    ## RIGHT ##
    # RShoulderPitch saturation
    if RShoulderPitch is None:
        print ("warning : RShoulderPitch value missing")
    elif RShoulderPitch < -2.0857:
        RShoulderPitch = -2.0857
    elif RShoulderPitch > 2.0857:
        RShoulderPitch = 2.0857
    
    # RShoulderRoll saturation
    if RShoulderRoll is None:
        print ("warning : RShoulderPitch value missing")
    elif RShoulderRoll < -1.5620 :
        RShoulderRoll = -1.5620
    elif RShoulderRoll > -0.0087:
        RShoulderRoll = -0.0087


    # RElbowYaw saturation
    if RElbowYaw is None:
        print ("warning : RElbowYaw value missing")
    elif RElbowYaw < -2.0857*limit:
        RElbowYaw = -2.0857*limit
    elif RElbowYaw > 2.0857*limit:
        RElbowYaw = 2.0857*limit

    # RElbowRoll saturation
    if RElbowRoll is None:
        print ("warning : RElbowRoll value missing")
    elif RElbowRoll < 0.0087:
        RElbowRoll = 0.0087
    elif RElbowRoll > 1.5620:
        RElbowRoll = 1.5620

    angles = [RShoulderPitch,LShoulderPitch,RShoulderRoll,LShoulderRoll, RElbowRoll, LElbowRoll]
    return angles 





"""
Attribution: The following function is taken from (and modified) teleop.py 
from https://github.com/elggem/naoqi-pose-retargeting
(Apache 2.0 Licensed)

also taken from (and modified) landmarks2angles.py for nao
https://github.com/hhuebert/sobotify/blob/main/sobotify/robots/nao/landmarks2angles.py
(MIT Licensed)
"""

def radiant_zu_grad(wert):
    if wert is None:
        raise ValueError("Input to radiant_zu_grad is None")
    return int(wert * (180.0 / math.pi))


def right_ellbow_roll(wert):
    temp = (-6.38 * wert) + 667
    if (temp > 480):
        RightEllbow = 500
    elif (temp < 220):
        RightEllbow = 200
    else:
        RightEllbow = (-6.38 * wert) + 667
    return RightEllbow


def right_shoulder_roll(wert):
    temp = (-3.125 * wert) + 250
    if (temp > 480):
        RightshoulderRoll = 500
    elif (temp < 270):
        RightshoulderRoll = 250
    else:
        RightshoulderRoll = (-3.125 * wert) + 250
    
    return RightshoulderRoll


def right_shoulder_pitch(wert):
    temp = (-3.529 * wert) + 500
    if (temp > 800):
        RightshoulderPitch = 800
    elif (temp < 200):
        RightshoulderPitch = 200
    elif (temp > 520 and temp < 480):
        RightshoulderPitch = 500
    else:
        RightshoulderPitch = (-3.529 * wert) + 500
    
    return RightshoulderPitch

def left_ellbow_roll(wert):
    temp = (-6.38 * wert) + 340
    if (temp > 780):
        leftEllbow = 800
    elif (temp < 520):
        leftEllbow = 500
    else:
        leftEllbow = (-6.38 * wert) + 340
    
    return leftEllbow


def left_shoulder_roll(wert):
    temp = (-3.125 * wert) + 765.625
    if (temp > 730):
        leftshoulderRoll = 750
    elif (temp < 520):
        leftshoulderRoll = 500
        
    else:
        leftshoulderRoll = (-3.125 * wert) + 765.625
    
    return leftshoulderRoll

def left_shoulder_pitch(wert):
    temp = (3.529 * wert) + 500
    if (temp > 800):
        leftshoulderPitch = 800
    elif (temp < 200):
        leftshoulderPitch = 200
    elif (temp > 520 and temp < 480):
        leftshoulderPitch = 500
    else:
        leftshoulderPitch = (3.529 * wert) + 500  
    
    return leftshoulderPitch



def checkLim(val, limits):
    return val < limits[0] or val > limits[1]

def convert(world_landmarks_array, time_stamp,angles_filename):

    visibility_threshold=0.5


    pNeck =   (0.5 * (np.array(world_landmarks_array[11]) + np.array(world_landmarks_array[12]))).tolist()
    pMidHip = (0.5 * (np.array(world_landmarks_array[23]) + np.array(world_landmarks_array[24]))).tolist()

    LShoulderPitch, LShoulderRoll = keypointsToAngles.obtain_LShoulderPitchRoll_angles(pNeck, world_landmarks_array[11], world_landmarks_array[13], pMidHip)
    RShoulderPitch, RShoulderRoll = keypointsToAngles.obtain_RShoulderPitchRoll_angles(pNeck, world_landmarks_array[12], world_landmarks_array[14], pMidHip)
    LElbowYaw, LElbowRoll = keypointsToAngles.obtain_LElbowYawRoll_angle(pNeck, world_landmarks_array[11], world_landmarks_array[13], world_landmarks_array[15])
    RElbowYaw, RElbowRoll = keypointsToAngles.obtain_RElbowYawRoll_angle(pNeck, world_landmarks_array[12], world_landmarks_array[14], world_landmarks_array[16])

    winkeln =saturate_angles(LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll)

    rightEllbow = int(right_ellbow_roll(radiant_zu_grad(winkeln[4])))
    rightShoulderRoll = int(right_shoulder_roll(radiant_zu_grad(winkeln[2])))
    rightShoulderPitch = int(right_shoulder_pitch(radiant_zu_grad(winkeln[0])))
   
    
    leftEllbow = int(left_ellbow_roll(radiant_zu_grad(winkeln[5])))
    leftShoulderRoll = int(left_shoulder_roll(radiant_zu_grad(winkeln[3])))
    leftShoulderPitch = int(left_shoulder_pitch(radiant_zu_grad(winkeln[1])))
    

    
    angles = [rightShoulderPitch, leftShoulderPitch, rightShoulderRoll, leftShoulderRoll, rightEllbow, leftEllbow]
    
    return True, angles




