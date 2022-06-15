#!/usr/bin/env python

import pygame
import pygame.freetype
import time
from random import randint
import numpy as np
import sys
import rospy
import os
from musi_care.msg import SongData
from musi_care.srv import sound_player_srv
from musi_care.srv import qt_command
from musicare_lib import TimeFunctions 


class Guess_The_Mood_Game():
	""" Class to generate and handle guess the mood game """
	
	def __init__(self):
	    """Initialise"""
        self.pygame = pygame
        self.pygame.init() #start py engine
        self.pygame.freetype.init() 
        res = pygame.display.Info() #get our screen resolution
        self.window_x = res.current_w -150#Width of window -150 to account for the linux toolbar
        self.window_y = res.current_h  #Height of window
        self.window_center = (int(self.window_x/2), int(self.window_y/2))
        self.window = pygame.display.set_mode( (self.window_x, self.window_y) ) #Create window and set size
        self.pygame.display.set_caption("Testing Env!") #Label window
        self.run = True
        self.pygame.mouse.set_visible(False)
        self.quit = False #Check to see if the game ended or it was quit
        self.track_playing = False
        self.previous_track_data = None
        self.difficulty = "easy" #Default difficulty to play
        self.current_level = 1 #Default level to play
        self.music_data = {} #our database that we will write into 
        self.get_song_database() #From save file load all of the level data 
        self.pygame.mouse.set_visible(False)
        self.timer_manager = TimeFunctions()
        self.music_filepath = "/game_assets/music/" #relative path to songs # "/home/qtrobot/catkin_ws/src/musi_care/src/game_assets/music/" 
        #self.default_volume = 1 # change volume of laptop
        #self.
        #self.volume_change(self.default_volume) # Set a default volume
        #self.set_robot_volume() #TODO add this functionality  
                   
                   
    def get_song_database(self):
        """Read the database file and get the levels data"""

        #data_filepath = ("/home/qtrobot/catkin_ws/src/musi_care/src/game_assets/music/music_data.txt")
        data_filepath = ("game_assets/data/gtm_level_data.txt") #gtm = guess the mood
	
        with open (data_filepath, "r") as database:
            self.music_data = {"easy":{1:{""}}, "medium":{1:{""}}, "hard":{1:{""}}} #Reset data to overwrite it thouroughly
            
            #sort data into their difficulty tiers
            data = database.read().splitlines() #read data and load into raw into "data"
            for difficulty in self.music_data.keys(): #extract for each difficulty seperately
                open_bracket_line_num = 0
                open_bracket_found= False
                close_bracket_line_num = 0
                close_bracket_found = False
                line_num = 0

                #Look for brackets and get the data between them
                for line in data:
                    if line == "{":
                        open_bracket_line_num = line_num
                        open_bracket_found = True
                    elif line == "}":
                        close_bracket_line_num = line_num
                        close_bracket_found = True
                    #We have found the brackets, save the information and delete it from data
                    if open_bracket_found and close_bracket_found:
                        #Organise data and put it into a new dict   
                        difficulty_data = data[open_bracket_line_num+1:close_bracket_line_num]
                        new_song = False
                        level = 1
                        self.music_data[difficulty][level] = dict() #init 1st level
                        for attribute in difficulty_data:
                            if attribute == "£#":
                                new_song = True
                            if not new_song: #if all the atrributes describe the same song, add them to the same dict
                                split_attribute = attribute.split("=")
                                attribute_label = split_attribute[0].replace(" ", "")
                                if attribute_label != "hint": #dont get rid of the spaces in hint
                                    attribute_value = split_attribute[1].replace(" ", "")
                                else:
                                    attribute_value = split_attribute[1][1:] # Get rid of the space at the start
                                self.music_data[difficulty][level][attribute_label] = attribute_value
                            else:
                                new_song = False
                                level += 1
                                self.music_data[difficulty][level] = dict() # Create new song entry labeled as the correct level
                        data = data[close_bracket_line_num+1:]
                        break
                    line_num += 1         
                   

    def 
                   
                   
                   
#If we run this node, run the game on it's own
if __name__ == '__main__':
    #Initialise game
    rospy.init_node('guess_the_mood_game', anonymous=False)
    rospy.loginfo("Node launched successfully")
    game_object = Guess_The_Mood_Game()

    #Run the game
    try:
        game_object.Main()
    except(KeyboardInterrupt or rospy.exceptions.ROSInterruptException):
        game_object.pygame.quit
        print("Audio may not be stopped due to interrupt")
