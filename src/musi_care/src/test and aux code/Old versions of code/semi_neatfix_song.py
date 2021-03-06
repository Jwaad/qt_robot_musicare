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
import random
from random import randint
from musi_care.msg import SongData
from musi_care.srv import sound_player_srv
from musi_care.srv import qt_command
from musicare_lib import TimeFunctions 
from musicare_lib import Button
from musicare_lib import DragableButton
from musicare_lib import ToggleButton
from musicare_lib import PausePlayButton
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
        #print(self.music_data)
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
        this_file_path = os.path.dirname(__file__)
        full_path =  this_file_path + data_filepath
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
                            if attribute == "??#":
                                new_song = True
                            if not new_song: #if all the atrributes describe the same song, add them to the same dict
                                split_attribute = attribute.split("=") #split line by the =
                                attribute_label = split_attribute[0].replace(" ", "") #get rid of any spaces now
                                if attribute_label == "distract_song": #parse differently
                                    if split_attribute[1].replace(" ", "") == "none": #if there's no distract songs, just return none
                                        attribute_value = None
                                    else:
                                        split_attribute.pop(0)
                                        split_attribute #get rid of the label, the rest are songs
                                        songs = split_attribute[0].split(",")
                                        attribute_value = []
                                        for song in songs:
                                            distract_song = song.replace(" ", "") #remove spaces
                                            attribute_value.append(distract_song)
                                else:
                                    attribute_value = split_attribute[1].replace(" ", "") # Get rid of the space at the start
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
        

    def GetTrackInfo(self, formatted_output = False): 
        """Subscribe to sound_player publisher and get elapsed track time"""

        song_data = self.sound_manager.request_song_data()
        
        self.track_title = song_data.track_title #track title
        self.total_track_secs = song_data.track_total_time #track time in seconds
        self.elapsed_time_secs = song_data.track_elapsed_time #current time in seconds

        #Either return the above, or format the secs to be displayed
        if formatted_output:
            total_mins, total_secs = self.format_elapsed_display(self.total_track_secs) 
            elapsed_mins, elapsed_secs = self.format_elapsed_display(self.elapsed_time_secs)
            
            elapsed_time = elapsed_mins+ ":" + elapsed_secs
            total_time = total_mins + ":" + total_secs
        
            return elapsed_time, total_time
        else:
            return self.track_title , self.elapsed_time_secs, self.total_track_secs      


    def get_song_info(self, prev_track_time = "", prev_total_time=""):
        """Get variables that we will draw onto screen"""
        formatted_data = self.GetTrackInfo(formatted_output = True)
        self.current_track_time = formatted_data[0]          #Time gotten from sound_player node
        self.track_total_time = formatted_data[1] #Total track time          
        self.progress = self.elapsed_time_secs / self.total_track_secs #elapsed time in percentage completion, so slider can represent that on a bar
        self.song_ended = self.progress >= 0.99 # if self.progress > 99% = song is finished, otherwise false
        return self.current_track_time, self.track_total_time, self.progress, self.song_ended
        
        
    def format_elapsed_display(self, time):
        "bit of code that converts secs to mins and secs"
        if time < 60:
            secs = int(time)
            mins = str(0 )
        else:
            secs = int(time%60)
            mins = str(int((time - secs) / 60))
        if secs < 10:
            secs = "0" + str(secs)
        return str(mins), str(secs)


    def CreateButton(self, file_name, alt_file_name, location, scale=1, unique_id=""):
        """code creates button using the button_image class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/graphics'
        file_path = os.path.join(this_file_path, relative_path, file_name)
        alt_path = os.path.join(this_file_path, relative_path, alt_file_name)
        
        button = Button(file_path, alt_path, location, self.pygame, scale, unique_id)
        return(button)


    def CreateToggleButton(self, file_name, alt_file_name, default_image_grey, toggled_image_grey, location, scale=1, unique_id="", return_info="", when_toggle_on=object, when_toggle_off=object):
        """code creates button using the button_image class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = '/game_assets/graphics'
        file_path = this_file_path + relative_path + file_name
        alt_path = this_file_path + relative_path + alt_file_name
        
        button = ToggleButton(file_name, alt_file_name, default_image_grey, toggled_image_grey, location, self.pygame, scale, unique_id, return_info, when_toggle_on, when_toggle_off)
        return(button)
        
        #default_image_path, toggled_image_path, default_image_grey, toggled_image_grey, x_y_locations, pygame, scale=1, unique_id = "", return_info="", when_toggle_on=object, when_toggle_off=object
        

    def CreateDragButton(self,file_name, alt_file_name, location,  scale=1, return_info="", when_toggle_on=object, when_toggle_off=object):
        """code creates button using the button_image class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/graphics'
        file_path = os.path.join(this_file_path, relative_path, file_name)
        alt_path = os.path.join(this_file_path, relative_path, alt_file_name)
        
        button = DragableButton(file_path, alt_path, location, self.pygame, scale, return_info = return_info, when_toggle_on=when_toggle_on, when_toggle_off=when_toggle_off)
        return(button)


    def CreatePlayButton(self, file_name, alt_file_name, file_grey, alt_file_grey, rewind_name, rewind_name_grey, location,  scale=1, unique_id = "", on_pause=object, on_play=object):
        """code creates toggle button using the toggle button class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/graphics'
        file_path = os.path.join(this_file_path, relative_path, file_name)
        alt_path = os.path.join(this_file_path, relative_path, alt_file_name)
        file_path_grey = os.path.join(this_file_path, relative_path, file_grey)
        alt_path_grey = os.path.join(this_file_path, relative_path, alt_file_grey)
        file_path = os.path.join(this_file_path, relative_path, file_name)
        alt_path = os.path.join(this_file_path, relative_path, alt_file_name)
        rewind_path = os.path.join(this_file_path, relative_path, rewind_name)
        rewind_path_grey = os.path.join(this_file_path, relative_path, rewind_name_grey)
        
        button = PausePlayButton(file_path, alt_path, file_path_grey, alt_path_grey, rewind_path, rewind_path_grey, location, self.pygame, scale, unique_id, on_pause, on_play)
        return(button)  
        
        
    def CreateHorizontalSlider(self, slider_name, cursor_name, x_y_locations, scale=1, on_click= object, on_release = object):
        """Creates horizontal slider using the horizontal slider class"""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/graphics'
        slider_path = os.path.join(this_file_path, relative_path, slider_name)
        cursor_path = os.path.join(this_file_path, relative_path, cursor_name)
        
        slider = HorizontalSlider(slider_path, cursor_path, x_y_locations, scale, on_click, on_release)
        return slider
    
    
    def update_graphics(self, current_track_time, track_total_time, progress, slider_x, slider_y):
        """Redraw graphics """
        self.renderer.DrawBackground(self.background_colour)
        self.renderer.DrawTextCentered("What mood does this song have?", font_size = 100, y = 100)
        self.renderer.DrawText(str(current_track_time), (slider_x - 75, slider_y +75), font_size = 50) #draw current time
        self.renderer.DrawText(str(track_total_time), (2650, slider_y +75), font_size = 50) #draw total track time
        self.song_duration_slider.render(self.window, progress)
        for button in self.buttons: #Draw buttons using button list
            button.render(self.window)
        self.animation_manager.DrawTouchAnimation(self.window) #last so it shows up on top


    def update_grey_graphics(self, current_track_time, track_total_time, progress, slider_x, slider_y):
        """reinitialise grey graphics taking with parameters they need """
        #Load graphics into dict # need to be in a specific order for this tut
        grey_graphics = {}
        grey_graphics[1] = functools.partial(self.renderer.DrawTextCentered, "What mood does this song have?", font_size = 100, y = 100)
        grey_graphics[2] = functools.partial(self.renderer.DrawText, str(current_track_time), (slider_x - 75, slider_y +75), font_size = 50) #draw current time
        grey_graphics[3] = functools.partial(self.renderer.DrawText, str(track_total_time), (2650, slider_y +75), font_size = 50)
        grey_graphics[4] = functools.partial(self.song_duration_slider.render, self.window, progress, grey = True)
        i = 4
        for button in self.buttons:
            i+=1
            grey_graphics[i] = functools.partial(button.render, self.window, grey = True)
        
        return grey_graphics      
        
        
    def highlight_block(self, events, target_rect = None, msg = "Click anywhere to continue ... ", timer_complete = None):
        """
        Highlight a certain object and check for click
        target_graphic = the graphic that's in colour
        Graphics here are a dict of the partial functions for each renderer in grey
        target_rect the rect that we want to high light around
        should be used by being kept in a while loop with other graphics to be drawn
        """

        #Handle events
        for event in events:
            #reset / init variables
            option_chosen = ""
            mouse_pos = self.pygame.mouse.get_pos()
            if event.type == self.pygame.MOUSEBUTTONUP:  #on mouse release play animation to show where cursor is
                self.animation_manager.StartTouchAnimation(mouse_pos) #tell system to play animation when drawing
                return True
                
        #Render graphics
        if target_rect != None: #so we can have blocking functionality without highlighting
            self.renderer.HighlightRect(target_rect, self.pygame) #draw arrow and box
        if msg != "": #we can send an empty msg to msg instead to have it not display anything
            if timer_complete == None: #if user didnt specify a timer, just show text like normal
                self.renderer.DrawTextCentered(msg, font_size = 75, font_colour = (0,0,0))
            else:
                if timer_complete: #only render once timer done
                    self.renderer.DrawTextCentered(msg, font_size = 75, font_colour = (0,0,0))
                
        return False
                 
                 
    def load_list_graphics(self, graphics, keys):
        """takes graphics dict and loads them
            Graphics = {1: graphic_func_1, 2: graphic_func_2. etc}
            key = [1,3,4,5]
        """
        for key in graphics.keys(): #Draw each object
            if key in keys: #only draw the graphics we ask for
                graphics[key]() #run as func
    
    
    def create_graphics(self, slider_scale, slider_x, slider_y):
        """Create the pygame objects that we will use """
        self.sad_button = self.CreateButton("sad_button.png", "sad_button_depressed.png", (675,750), scale=1.3, unique_id="sad") 
        self.happy_button = self.CreateButton("happy_button.png", "happy_button_depressed.png", (675,1150), scale=1.3, unique_id="happy") 
        self.unsure_button = self.CreateButton("unsure_button.png", "unsure_button_depressed.png", (850,1550), scale=1, unique_id = "unsure") 
        self.play_button = self.CreatePlayButton("pause_button.png", "pause_button_grey.png", "play_button.png",  "play_button_grey.png", "rewind_button.png", "rewind_button_grey.png", (self.cen_x-100, 175), scale = 1, on_play= self.sound_manager.unpause , on_pause = self.sound_manager.pause) #create pause and play button
        self.song_duration_slider = self.CreateHorizontalSlider("track_duration_slider.png", "track_cursor.png", (slider_x,slider_y), on_click = self.sound_manager.stop_track, on_release = self.sound_manager.start_track, scale=slider_scale)
        
        
    def get_target_behaviour(self, key, progress):
        """tells our tut what to draw and what to do with events"""
        target_graphics = []
        target_event_handler = None
        if key ==3:
            target_graphics = [functools.partial(self.sad_button.render, self.window), functools.partial(self.happy_button.render, self.window)]
        elif key ==4:
            target_graphics = [functools.partial(self.unsure_button.render, self.window)]
        elif key == 5:
            target_graphics = [functools.partial(self.song_duration_slider.render, self.window, progress), functools.partial(self.play_button.render, self.window)]
        elif key == 8:
            target_graphics = [functools.partial(self.play_button.render, self.window)]
            
        return target_graphics, target_event_handler #if our logic sifts failed


    def play_seg_track(self,song_path):
        """Uses the stored info attribute to play music on button press """
        
        def play_track():
            self.sound_manager.start_track(song_path)
        
        return play_track
     
     
    def stop_seg_track(self,song_path):
        """Uses the stored info attribute to play music on button press """
        
        def stop_track():
            self.sound_manager.stop_track()
        
        return stop_track
                
#######################################################Level / screen code###############################################################
        
    def play_music_blocking(self, difficulty, level): 
        """Level with just music player"""
        if self.run:
            
            
            #Get the level's data
            level_data = self.music_data[difficulty][level] #{"song_name":"title", "mood":"happy", "hint":"some text"}
            self.track_name = level_data["song_name"]
            
            #Create slider and buttons
            slider_x = 275
            slider_y = 800
            song_duration_slider = self.CreateHorizontalSlider("track_duration_slider.png", "track_cursor.png", (slider_x,slider_y), scale=2)
            self.next_button = self.CreateButton("next_button.png", "next_button_grey.png", (self.cen_x - 300,1200), self.pygame, scale=1)
            self.play_button = self.CreatePlayButton("pause_button.png", "pause_button_grey.png", "play_button.png",  "play_button_grey.png", "rewind_button.png", "rewind_button_grey.png", (self.cen_x-100, 450), scale = 1, on_play= self.sound_manager.unpause , on_pause = self.sound_manager.pause) #create pause and play button
            
            #Load track
            self.sound_manager.load_track(self.track_name)

            #Variables
            music_playing = True
            self.song_ended = False
            self.current_track_time = 0
            self.track_total_time = 100
            self.progress = 0
            self.current_track_time, self.track_total_time, self.progress, self.song_ended = self.get_song_info(self.current_track_time, self.track_total_time)
            
            self.sound_manager.unpause()
            music_ended = False
            next_pressed = False
            while not next_pressed and not rospy.is_shutdown() and self.run:
                time = rospy.get_time()
                
                #Format song time elapsed to display on screen
                formatted_data = self.GetTrackInfo(formatted_output = True)
                self.current_track_time, self.track_total_time, self.progress, self.song_ended = self.get_song_info(self.current_track_time, self.track_total_time)
                
                if self.song_ended:
                    self.play_button.its_rewind_time() #draw rewind symbol
                    self.sound_manager.load_track(self.track_name) #reload the track at the start
                    music_ended = True #use this var, so we have one that stays true for the rest of the loop

                #Draw background and objects
                self.renderer.DrawBackground(self.background_colour)
                self.renderer.DrawText(str(self.current_track_time), (slider_x - 75, slider_y +75), font_size = 50) #draw current time
                self.renderer.DrawText(str(self.track_total_time), (2650, slider_y +75), font_size = 50) #draw total track time
                self.renderer.DrawTextCentered("Please listen to the song.", font_size = 100, y = 300)
                song_duration_slider.render(self.window, self.progress)
                if music_ended: #render next button grey until music ended
                    self.next_button.render(self.window, grey= False)
                else:
                    self.next_button.render(self.window, grey = True)
                self.play_button.render(self.window)
                self.animation_manager.DrawTouchAnimation(self.window) # also draw touches
                self.pygame.display.update() #Update all drawn objects

                #Check if the X was clicked
                for event in self.pygame.event.get():
                    if event.type == self.pygame.QUIT:
                        self.run = False #Stops the program entirely
                        self.sound_manager.stop_track()
                    mouse_pos = self.pygame.mouse.get_pos()
                    if event.type == self.pygame.MOUSEBUTTONUP:
                        self.animation_manager.StartTouchAnimation(mouse_pos) #draw mouse click animation
                        pass
                    #Check for button press
                    if music_ended: #if music ended start checking for next press, otherwise ignore it
                        next_button_clicked = self.next_button.get_event(event, mouse_pos)
                        if next_button_clicked:
                            next_pressed = True
                            self.sound_manager.stop_track()
                    self.play_button.get_event(event, mouse_pos)
                    
                 #print(rospy.get_time() - time)


    def play_level(self, difficulty, level_num):
        """Sequence plays the levels"""
        if self.run: #Dont start this screen if the previous screen wanted to close out the game
            
            #Get the level's data
            level_data = self.music_data[difficulty][level_num] #{"song_name":"title", "mood":"happy", "hint":"some text"}
            self.track_name = level_data["song_name"]
            self.distract_song = level_data["distract_song"] #will be None or a list of songs
            
            #store colours in here so we can 
            self.segment_graphics = {
            1:("music_segment_play_blue.png","music_segment_pause_blue.png"), 
            2:("music_segment_play_orange.png","music_segment_pause_orange.png"), 
            3:("music_segment_play_purple.png","music_segment_pause_purple.png"),
            4:("music_segment_play_grey.png","music_segment_pause_grey.png")
            #blue = 1 orange = 2 purple = 3 grey = 4.  #4 is reserved for the ones at the top and when grey screen
            }
            
            #create buttons for each segments
            segment_num = 2 #split song into this many segs
            all_seg_paths, correct_segments = self.sound_manager.slice_song(segment_num, self.track_name, self.distract_song)
            segment_x_y = { 0: (450,450), 1:(750,450), 2:(450,750), 3:(750,750), 4: (100,100), 5: (100,200)} #temp hard coded num TODO: change these to calculated pos for each
            
            seg_pos_list = []
        
            for i in range(len(correct_segments)): #TODO find better way to do this
                seg_pos_list.append(i) #were going to use this list to randomise the pos of the answer segments
                
            #create and order dragable buttons this is scalable
            dragable_buttons = {}
            i = 0
            correct_seg_order = []
            dragable_pos = {} 
            
            for song_path in correct_segments:
                #create a moving button for each and also setup correct_seg
                correct_seg_order.append(song_path)
                seg_pos = segment_x_y[seg_pos_list.pop(random.randint(0,len(seg_pos_list)-1))]
                #seg_colour = self.segment_graphics[random.randint(2,3)] #ignore blue
                seg_colour = self.segment_graphics[1] #segments should be blue
                dragable_pos[i] = seg_pos
                dragable_buttons[i] = self.CreateDragButton(seg_colour[0], seg_colour[1], seg_pos, return_info =song_path, when_toggle_on = self.play_seg_track(song_path), when_toggle_off = self.stop_seg_track(song_path))
                i+=1
            
            #Create main buttons
            loading_button = self.CreateButton("loading_screen_button_depressed.png", "loading_screen_button_depressed.png", (200,225))
            song_segment_y = 550
            song_half = self.CreateToggleButton(self.segment_graphics[2][0], self.segment_graphics[2][1],self.segment_graphics[4][1],self.segment_graphics[4][1],(song_segment_y, 90),return_info =correct_segments[0],  when_toggle_on = self.play_seg_track(correct_segments[0]), when_toggle_off = self.stop_seg_track(song_path))
            song_unknown = self.CreateButton( "music_segment_greyed_out.png", "music_segment_greyed_out.png",(song_segment_y+150,90)) #TODO make this auto scale and spawn according to difficulty
            #play_button = self.CreateToggleButton("play_button.png","pause_button.png",(song_segment_y+250,130))
            check_button = self.CreateButton("check_button.png", "check_button.png", (song_segment_y+350,130))
            main_buttons = [loading_button, song_unknown] #list of buttons so we can easier render them
            
            #TODO shuffle in some fake segments

            qt_has_spoken = False
            song_restored = False
            current_seg_order = [song_half.return_info]
            slot_free = True #there's no segment in the slot
            
            #Copy format of our previous data
            is_in_slot = dragable_pos.copy() #the inital pos of each dragable
            for key in is_in_slot.keys():
                is_in_slot[key] = False
                
            while not song_restored and not rospy.is_shutdown() and self.run:
                self.DrawBackground()
                song_half.render(self.window)
                for button in main_buttons:
                    button.render(self.window)
                if not slot_free: #only render this once a song is in the slot
                    check_button.render(self.window)
                self.DrawText("Put the song back together", (700, 40), 50)
                self.DrawText("Drag the segment into the slot", (700, 300), 50)
                for key in dragable_buttons: #render dragable buttons
                    dragable_buttons[key].render(self.window) #draw the segments
                self.DrawMouseCursor(self.window)
                self.pygame.display.update() #Update all drawn objects

                if not qt_has_spoken: #in here so he speaks while yes and no are drawn
                    self.qt_gesture("explain_right")
                    self.qt_emote("sad")
                    self.qt_say_blocking("The song is all jumbled up! Help me put it back together!")
                    qt_has_spoken = True #QT has spoken now, so dont let him talk again
                
                for event in self.pygame.event.get():
                    if event.type == self.pygame.QUIT:
                        self.run = False #Stops the program entirely
                        
                    #Check for button press
                    mouse_pos = self.pygame.mouse.get_pos()
                    for button in main_buttons:
                        button_press = button.get_event(event, mouse_pos)
                
                    press = song_half.get_event(event, mouse_pos)
                    if press: #if the track was stopped, or stopped, set them all to false
                            for key in dragable_buttons: #if pressed, set all others to false
                                dragable_buttons[key].toggle = False
                            song_half.toggle = True
                            
                    for key in dragable_buttons:
                        press, button_pos = dragable_buttons[key].get_event(event, mouse_pos)
                        #Check if toggle is pressed
                        if press:
                            for inner_key in dragable_buttons: #if pressed, set all others to false
                                dragable_buttons[inner_key].toggle = False
                            song_half.toggle = False
                            dragable_buttons[key].toggle = True # set our pressed one back to true
                            
                        #Check if seg is placed in the slot and allow only if the slot is free
                        if song_unknown.rect.collidepoint(button_pos.center) and not dragable_buttons[key].mouse_is_held and slot_free: #if our dragged part is released on top of the song slot
                            dragable_buttons[key].rect.x = song_unknown.rect.x
                            dragable_buttons[key].rect.y = song_unknown.rect.y
                            current_seg_order = current_seg_order + [dragable_buttons[key].return_info]#TODO change this so it's scaleable
                            is_in_slot[key] = True #this is currently in the slot
                            slot_free = False
                        #If it wasn't placed in the slot, placed it back in it's original location, and track which seg has left the slot
                        elif not song_unknown.rect.collidepoint(button_pos.center) and not dragable_buttons[key].mouse_is_held: #move the button back to where it was if it was released somewhere random
                            if is_in_slot[key]: #if we moved out from the slot, reset some variables
                                slot_free = True
                                current_seg_order.pop(1) #TODO change this to be scalable
                                print("song popped")
                                is_in_slot[key] = False
                            else:
                                dragable_buttons[key].rect.x = dragable_pos[key][0]
                                dragable_buttons[key].rect.y = dragable_pos[key][1]
                                
                    press = check_button.get_event(event, mouse_pos)
                    if press: #check if song order is correct
                        if current_seg_order == correct_seg_order:
                            self.qt_gesture("happy")
                            self.qt_emote("talking")
                            self.qt_say_blocking("Great job, that is correct!")
                            self.qt_emote("grin")
                            song_restored = True
                        else:
                            self.qt_gesture("shake_head")
                            self.qt_emote("talking")
                            self.qt_say("Sorry, that is not correct, please try again!")
                
            
        
        
            
#################################################################Main####################################################################   


    def Main(self, difficulty = "easy", level =  3): #input what level and difficulty to play, the program will handle the rest
        """Main Func"""
        
        #Introduce game
        self.run = self.level_loader.QTSpeakingScreen("Lets play Fix The Song!", self.run, self.background_colour)

        #Ask if they want to play tutorial
        self.run, tut = self.level_loader.yes_or_no_screen('Should I explain how to play "Fix The Song" ?', self.run, self.background_colour)
        if tut:
            print("tut chosed")
            #self.guided_tut()
        
        #Count in to the start of the game
        self.run = self.level_loader.tap_to_continue(self.run, self.background_colour)

        #Count into level to slow pacing
        self.run = self.level_loader.countdown(3, self.run, self.background_colour, prelim_msg = "Get ready to hear the song!")
        
        #Play the track and block
        self.play_music_blocking(difficulty, level)
        
        #Play main level
        #self.play_level(difficulty, level)
        
        
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
        
        
        
        
        
        
        
