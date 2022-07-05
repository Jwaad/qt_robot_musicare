#!/usr/bin/env python

import pygame
import pygame.freetype
import time
import numpy as np
import sys
import rospy
import os
import functools
import math
from random import randint
from musi_care.msg import SongData
from musi_care.srv import sound_player_srv
from musi_care.srv import qt_command
from musicare_lib import TimeFunctions 
from musicare_lib import Button
from musicare_lib import ToggleButton
from musicare_lib import AnimationManager
from musicare_lib import SoundManager
from musicare_lib import QTManager
from musicare_lib import Renderer
from musicare_lib import HorizontalSlider
from musicare_lib import StandardLevels

#################################################################Initialise#################################################################

class Fix_The_Song_Game():
    """ Class to generate and handle guess the mood game """
	
    def __init__(self):
        """Initialise"""
        x=145#x pos of screen
        y=0  # y pos of screen
        os.environ['SDL_VIDEO_WINDOW_POS'] = '%d,%d' % (x,y) #move screen to x and y pos
        self.previous_screen = "" # used so we can go backwards a screen
        self.next_screen = "" #used to skip screen, low priority feature
        self.pygame = pygame
        self.pygame.init() #start py engine
        self.pygame.freetype.init() 
        res = pygame.display.Info() #get our screen resolution
        self.window_x = res.current_w -150#Width of window -150 to account for the linux toolbar
        self.window_y = res.current_h  #Height of window
        self.window_center = (int(self.window_x/2), int(self.window_y/2))
        self.cen_x = self.window_center[0]
        self.cen_y = self.window_center[1]
        self.window = pygame.display.set_mode( (self.window_x, self.window_y) ) #Create window and set size
        self.background_colour = (100,100,100) #background black by default
        self.pygame.display.set_caption("Fix The Song!") #Label window
        self.run = True
        #self.pygame.mouse.set_visible(False) #set to false when not testing
        self.quit = False #Check to see if the game ended or it was quit
        self.track_playing = False
        self.previous_track_data = None
        self.difficulty = "easy" #Default difficulty to play
        self.current_level = 1 #Default level to play
        self.music_data = self.get_song_database() #From save file load all of the level data 
        print(self.music_data)
        self.music_filepath = "/game_assets/music/" #relative path to songs # "/home/qtrobot/catkin_ws/src/musi_care/src/game_assets/music/" 
        self.timer_manager = TimeFunctions()
        self.animation_manager = AnimationManager(self.pygame)
        self.sound_manager = SoundManager(self.music_filepath) #load soundplayer with sound file path
        self.command_manager = QTManager()
        self.renderer = Renderer(self.window,self.window_center)
        self.level_loader = StandardLevels(self.window, self.window_center, self.pygame, self.music_filepath)
        #self.music_vol = 1 # change volume of laptop
        #self.qt_voice_vol
        #self.sound_manager.volume_change(self.music_vol) # Set a default volume
        #self.set_robot_volume(qt_voice_vol) #TODO add this functionality  
        
  
########################################################Low level methods################################################################
        
    def get_song_database(self):
        """Read the database file and get the levels data"""

        #data_filepath = ("/home/qtrobot/catkin_ws/src/musi_care/src/game_assets/music/music_data.txt")
        data_filepath = ("/game_assets/data/fsg_level_data.txt") #gtm = guess the mood
        full_path = this_file_path = os.path.dirname(__file__) + data_filepath
        music_data = []
	    
        with open (full_path, "r") as database:
            difficulty_labels = ["tut","easy","medium", "hard"]
            music_data = {"tut":{1:{""}},"easy":{1:{""}}, "medium":{1:{""}}, "hard":{1:{""}}} #Reset data to overwrite it thouroughly
            
            #sort data into their difficulty tiers
            data = database.read().splitlines() #read data and load into raw into "data"
            for difficulty in difficulty_labels: #extract for each difficulty seperately
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
                        music_data[difficulty][level] = dict() #init 1st level
                        for attribute in difficulty_data:
                            if attribute == "£#":
                                new_song = True
                            if not new_song: #if all the atrributes describe the same song, add them to the same dict
                                split_attribute = attribute.split("=") #split line by the =
                                attribute_label = split_attribute[0].replace(" ", "") #get rid of any spaces now
                                if attribute_label != "hint": #dont get rid of the spaces in hint
                                    attribute_value = split_attribute[1].replace(" ", "")
                                else:
                                    attribute_value = split_attribute[1][1:] # Get rid of the space at the start
                                music_data[difficulty][level][attribute_label] = attribute_value
                            else:
                                new_song = False
                                level += 1
                                music_data[difficulty][level] = dict() # Create new song entry labeled as the correct level
                        data = data[close_bracket_line_num+1:]
                        break
                    line_num += 1 
            return music_data
            
        rospy.log_info("Catastrophic error, data read failed.")
        return music_data
        
        
#######################################################Level / screen code###############################################################
        
    def play_music_blocking(self, song_path, run, background_colour): 
        """Level with just music player"""
        
        #Create slider
        slider_y = 125
        slider_x = 450
        song_duration_slider = self.CreateHorizontalSlider("track_duration_slider.png", "track_cursor.png", (slider_y,slider_x))
        
        #Load track
        self.sound_manager.load_track(song_path)
        
        music_playing = True
        qt_spoken = False
        while music_playing and not rospy.is_shutdown() and self.run:
            
            #Format song time elapsed to display on screen
            formatted_data = self.GetTrackInfo(formatted_output = True)
            current_track_time = formatted_data[0]
            track_total_time = formatted_data[1] #Total track time
            progress = self.elapsed_time_secs / self.total_track_secs #elapsed time in percentage completion, so slider can represent that on a bar
            
            #Draw background and objects
            self.DrawBackground(background_colour)
            self.DrawText(str(current_track_time), (165, slider_x +100)) #draw current time
            self.DrawText(str(track_total_time), (1240, slider_x+100)) #draw total track time
            self.DrawText("Please listen to the song", (700, 100 ), 50)
            song_duration_slider.render(self.window, progress)
            self.pygame.display.update() #Update all drawn objects
            
            if qt_spoken == False:
                self.qt_emote("talking")
                self.qt_say_blocking("I am going to play the full song, listen carefully!")
                qt_spoken = True
                self.pause_unpause() #play the song
                
            #Check for end
            if self.check_track_ended(): #must come after draw_text
                music_playing = False
            
            #Check if the X was clicked
            for event in self.pygame.event.get():
                if event.type == self.pygame.QUIT:
                    self.run = False #Stops the program entirely

        
        
#################################################################Main####################################################################   

    def Main(self, difficulty = "easy", level =  1): #input what level and difficulty to play, the program will handle the rest
        """Main Func"""
        
        error = self.level_loader.QTSpeakingScreen("Lets play Fix The Song!", self.run, self.background_colour)
        
        if error == "QUIT": #if someone clicked quit during this screen then quit instead
            self.run = False
            self.quit = True
        
        """
        tut = self.level_loader.yes_or_no_screen('Should I explain how to play "Fix The Song" ?', self.run, self.background_colour)
        if tut:
            self.guided_tut()
        elif tut == "QUIT": #if someone clicked quit during this screen then quit instead
            self.run = False
            self.quit = True
        """
        
        error = self.level_loader.tap_to_continue(self.run, self.background_colour)
        if error == "QUIT":
            self.run = False
            self.quit = True
            
        self.play_level(difficulty, level)
        
        
        
######################################################On execution#######################################################################

#If we run this node, run the game on it's own
if __name__ == '__main__':
    #Initialise game
    rospy.init_node('fix_song_game', anonymous=False)
    rospy.loginfo("Node launched successfully")
    game_object = Fix_The_Song_Game()

    #Run the game
    try:
        game_object.Main()
    except(KeyboardInterrupt or rospy.exceptions.ROSInterruptException):
        game_object.pygame.quit
        SoundManager().stop_track()
        print("Audio may not be stopped due to interrupt")
        
        
        
        
        
        
        
