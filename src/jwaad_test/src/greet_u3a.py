#!/usr/bin/env python

##Brief
#QT will jumble a song then ask you to put it back in order
#Process: half song is give to you, you need to only choose 2nd half.
#
##TODO GENERAL
#
#
#
#
##Steps:
#QT plays explains game 1 time ever on blank screen.
#QT plays the song 
#

##Libraries
from aubio import source, tempo
from numpy import median, diff
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from qt_motors_controller.srv import *
import rospy
import time
import tty
import sys
import termios
import os
import pyaudio
import struct
import math
import numpy
from musi_care.msg import SongData
from musi_care.srv import sound_player_srv
from musi_care.srv import qt_command
import pygame
import pygame.freetype
from musi_care_lib import ImageButton as IB
from musi_care_lib import ToggleImageButton as TIB
from musi_care_lib import DragablePlayableImageButton as IDB #image dragable button
from musi_care_lib import TimeFunctions           
import wave
import contextlib
from pydub import AudioSegment
import random


#TODO Add these to a new lib, QT CONTROL
def send_qt_command( command_type, command_content= "", command_blocking = False):
    """Neatens and simplifies sending commands to QT """
    rospy.loginfo("Waiting for qt_command_service")
    rospy.wait_for_service('/qt_command_service')
    command_controller = rospy.ServiceProxy('/qt_command_service', qt_command)
    command_complete = command_controller(command_type, command_content, command_blocking)
    rospy.loginfo("completed handshake with qt_command_service")
    
    return command_complete
    
"""   
def qt_say_blocking( text):
    """'Makes QT say something, then makes you wait until the speaking is done'"""
    timer_len = len(text) * 0.08 #0.2s per letter 4 letter word is given 0.8s to be said
    Timer.CreateTimer("QT_SAY_BLOCKING", timer_len) #creates timer with ID 1 for 8s   
    send_qt_command("tts", text)
    talking = True
    while talking and not rospy.is_shutdown():
        if self.Timer.CheckTimer("QT_SAY_BLOCKING"): #if our timer is done
            talking = False           
"""

def qt_say(text):
    """Makes QT say something, then makes starts a timer until the speaking is done"""
    #timer_len = len(text) * 0.08 #0.2s per letter 4 letter word is given 0.8s to be said
    #self.Timer.CreateTimer("QT_SAY", timer_len) #creates timer with ID 1 for 8s   
    send_qt_command("tts", text)         


def qt_gesture(gesture):
    """Make QT do gesture, non blocking """
    send_qt_command("gesture", gesture)


def qt_emote(emote):
    """Make QT emote """
    send_qt_command("emote", emote)
    
    
    
def Main():
    qt_emote("grin")
    qt_say("Hi!")
    rospy.sleep(1)
    qt_gesture("wave")
    qt_emote("talking")
    qt_say("Great to meet you all!... I am Q-T the robot")
    rospy.sleep(3)
    qt_say("I love Obi one ken-oh-bee, but i am practising to become as good as, Bobby Mc-Ferrin.")
    rospy.sleep(2)
    qt_say("Hopefully you enjoyed my colleagues' presentation!... I thought you might be a bit bored, so I can show you something a bit more fun!")
    rospy.sleep(3)
    
                    
if __name__ == '__main__':
    #Initialise game
    rospy.init_node('Greet_U3A', anonymous=False)
    rospy.loginfo("Node launched successfully")
    
    Main()

            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
