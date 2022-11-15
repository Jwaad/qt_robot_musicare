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
#from qt_motors_controller.srv import *
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
from musicare_lib import ImageButton as IB
from musicare_lib import ToggleImageButton as TIB
from musicare_lib import DragablePlayableImageButton as IDB #image dragable button
from musicare_lib import TimeFunctions           
import wave
import contextlib
from pydub import AudioSegment
import random


#TODO delete this and add it to a lib
class HorizontalSlider():
    """Class to handle all functions of the horizontal sliders"""
    def __init__(self, image_path_slider, image_path_cursor, x_y_locations, slider_scale=1, cursor_scale=1, on_click=object, on_release=object):
        #init slider
        raw_slider_image = pygame.image.load(image_path_slider).convert_alpha()
        img_x = x_y_locations[0]
        img_y = x_y_locations[1]
        slider_img_w = int(raw_slider_image.get_width()*slider_scale)
        slider_img_h = int(raw_slider_image.get_height()*slider_scale)
        scaled_size = (slider_img_w, slider_img_h)
        self.slider_image = pygame.transform.scale(raw_slider_image, scaled_size)
        self.slider_rect = pygame.Rect(img_x,img_y,slider_img_w,slider_img_h)
        
        #init cursor
        raw_cursor_image = pygame.image.load(image_path_cursor).convert_alpha()
        cursor_img_w = int(raw_cursor_image.get_width()*cursor_scale)
        cursor_img_h = int(raw_cursor_image.get_height()*cursor_scale)
        self.half_cursor_height = cursor_img_h/2
        scaled_size = (cursor_img_w, cursor_img_h)
        self.cursor_image = pygame.transform.scale(raw_cursor_image, scaled_size)
        self.cursor_rect = pygame.Rect(img_x,img_y,cursor_img_w,cursor_img_h) 
        
        #init variables
        self.slider_being_held = False
        self.bar_overwrite = 0.0
        self.slider_len = 1080 #total pixel length that the bar should extend to at the end of the track
        slider_min = 161 #starting point of red bar (pixel_x)
        slider_max = slider_min + self.slider_len #ending point of max red bar pixel X location of slider end point
        self.slider_range = (slider_min, slider_max) #min and max including space to left of bar
        self.on_click = on_click
        self.on_release = on_release
    
    
    def render(self, screen, progress):
        """Draw slider, cursor and progress bar onto screen """
        screen.blit(self.slider_image, self.slider_rect)
        self.draw_progress_bar(screen, progress)
        self.draw_cursor(screen)
                
                
    def draw_progress_bar(self, screen, progress):
        """Uses a percentage to colour the completed relevant of the slider in red"""
        complete_bar_width = self.slider_len
        bar_height = 57
        bar_y = 160
        bar_x = self.slider_range[0] +300
        if self.slider_being_held:
            bar_width = complete_bar_width*self.bar_overwrite #If the user is moving the slider, display their new slider 
        else:
            bar_width = complete_bar_width*progress
        
        self.cursor_y = bar_width + bar_y
        self.red_bar = pygame.draw.rect(screen, (255,0,0), pygame.Rect((bar_y,bar_x), (bar_width, bar_height)))
        
        
    def draw_cursor(self, screen):
        """uses progress to move cursor to where it should be. THIS SHOULD ALWAYS BE AFTER 'draw_progress_bar()' """
        cursor_x = 460 + int(self.half_cursor_height) - 10
        self.cursor_rect.center = (self.cursor_y, cursor_x)
        screen.blit(self.cursor_image, self.cursor_rect) 
    
    
    def get_event(self, event, mouse_pos, track_info=["", 0.0, 999]): #track info = title, elapsed_time, total_time
        """handle events """
        mouse_on_cursor = self.cursor_rect.collidepoint(mouse_pos)
        mouse_x = mouse_pos[0]
        
        #Once slider is grabbed
        if event.type == pygame.MOUSEBUTTONDOWN and mouse_on_cursor:
            self.on_click()
            if self.slider_range[0] < mouse_x < self.slider_range[1]:
                self.slider_being_held = True
                
        #Once slider is released
        if event.type == pygame.MOUSEBUTTONUP and self.slider_being_held:
            #expand track_info
            track_title = track_info[0]
            track_time = track_info[1]
            track_total_time = track_info[2]
            
            time_to_start = track_total_time * self.bar_overwrite # get overwrite in secs
            
            self.on_release(track_title, time_to_start)
            
            self.slider_being_held = False
            self.bar_overwrite = 2.0 #  reset slider overwrite. Use a weird number, as this should only occur if i've made a coding error.
            
        #While slider is being dragged
        if self.slider_being_held: #If the user is dragging the cursor
            if self.slider_range[0] < mouse_x < self.slider_range[1]:                   #If mouse x within slider x range
                self.bar_overwrite = (mouse_x - self.slider_range[0])/self.slider_len   #Get percentage of where mouse is compared to len of bar
            elif mouse_x > self.slider_range[1]:
                self.bar_overwrite = 1.0 #Don't let cursor move past slider bar
            elif mouse_x < self.slider_range[0]:
                self.bar_overwrite = 0.0 #Don't let cursor move past slider bar




                              
class Fix_Song_Game():

###INITIALISE###

    def __init__(self):
        self.run = True #Change this if something has gone wrong and we want to skip processes and just end.
        #rospy.init_node('fix_song_game', anonymous=False) #init node
        self.init_robot(50)
        self.init_pygame()
        self.timers = {}
        self.Timer = TimeFunctions()
        self.level_data = {"easy":{1:"you_are_my_sunshine.wav", 2:"dont_fence_me_in_short.wav", 3:"happy_2.wav"},"normal":{},"hard":{}} # TEMP TODO: use a save file instead of this
        self.music_filepath = "/home/qtrobot/catkin_ws/src/musi_care/src/game_assets/music/"
        self.track_playing = False
        #self.volume_change(1) # Set default robot output volume
        
        
    def init_robot(self, arm_vel):
        """Method to init robot parameters"""
        #Set control mode, incase they were changed before hand
        rospy.wait_for_service('/qt_robot/motors/setControlMode')
        self.set_mode = rospy.ServiceProxy('/qt_robot/motors/setControlMode', set_control_mode)
        mode_changed = self.set_mode(["right_arm", "left_arm"], 1)
        if mode_changed:
            rospy.loginfo("Motors successfully set control mode 1")
        else:
            rospy.loginfo("Motor control mode could not be changed")
            self.run = False
        
        #Set velocity of arms incase they were set differently
        rospy.wait_for_service('/qt_robot/motors/setVelocity')
        set_vel = rospy.ServiceProxy('/qt_robot/motors/setVelocity', set_velocity)
        speed_changed = set_vel(["right_arm", "left_arm"], arm_vel)
        if speed_changed:
            rospy.loginfo("Motors successfully set to default speed ({})".format(arm_vel))
        else:
            rospy.loginfo("Motor speed could not be changed")
            self.run = False
    
    
    def init_pygame(self):
        """Method to init pygame variables"""
        self.pygame = pygame
        self.pygame.init() #start py engine
        self.pygame.freetype.init()
        self.window_x = 1400 #Width of window
        self.window_y = 800  #Height of window
        self.window_center = (int(self.window_x/2), int(self.window_y/2))
        self.window = pygame.display.set_mode( (self.window_x, self.window_y) ) #Create window and set size
        self.pygame.display.set_caption("Guess the mood of the music!") #Label window
        self.fps = 30# set fps
        self.clock = pygame.time.Clock() #enable clock that will cap fps
        self.run = True #Decides when the game should end
        self.quit = False #CheckFix_Song_Game to see if the game ended or it was quit
        self.block_events = False #Stop events except quit from happening, used when QT is talking.
        self.pygame.mouse.set_visible(False)
    
    
    def call_sound_player(self, operation, data_1 = "", data_2 = 0.0 ):
        """makes it easier to call sound_player"""
        rospy.loginfo("Waiting for sound player topic")
        rospy.wait_for_service('/sound_player_service')
        sound_player = rospy.ServiceProxy('/sound_player_service', sound_player_srv)
        song_data = sound_player(operation, data_1, data_2)
        rospy.loginfo("completed handshake with sound_player") 
        return song_data
            

    def start_track(self, track_title, track_time=0.0):
        """Starts a track and also saves the information returned as previous song, so we can replay songs without sending a new request"""
        #store default path to music
        track_path = os.path.join(self.music_filepath, track_title)
        #Start track
        operation = "start_track"
        
        song_data = self.call_sound_player(operation, track_path, track_time)
        callback_successful = song_data.status
        
        #variables
        self.track_playing = True
        
        if callback_successful:
            #TODO turn the sound_player into only a service, so that the msgs comes back instantly rather than after 0.5s
            time.sleep(0.3) #Due in part to my incorrect use of service call back we have to wait a second and let the service finish the command. Any message sent during those driver errors wont go through.
            self.previous_track_data = self.request_song_data() #Get the correct data format and the total track time
            self.previous_track_data.track_title = track_title
            self.previous_track_data.track_elapsed_time = 0.0 # previous song data should always start from 0.0
                
        return callback_successful


    def stop_track(self):
        """Stop track, dont hold any data in memory"""
        self.track_playing = False
        operation = "stop_track"
        status = self.call_sound_player(operation).status # we only need operation, the other variables can default, theyre ignored anyways
        return status
        

    def pause_unpause(self):
        """Pause track and resume from the same point later"""
        operation = "pause_resume"
        status = self.call_sound_player(operation).status # we only need operation, the other variables can default, theyre ignored anyways
        return status
        

    def volume_change(self, volume_percentage):
        """change volume, 1.0 = 100%, 0.5 = 50% etc"""
        operation = "volume"
        volume = volume_percentage * 100 #other methods use decimal percentages, so for consistency this method does too, but then converts it to numerical percentage
        status = self.call_sound_player(operation, data_2 = volume).status # we only need operation and data_2 the other variable can default, it's ignored anyways
        return status


    def request_song_data(self):
        """ask the service for data. TODO method needs reworking"""
        if self.track_playing:
            operation = "request_data"
            data = self.call_sound_player(operation)
            return data
        else:
            return self.previous_track_data


    def check_track_ended(self):
        """Check if the track we played has ended, if it has, stop the track officially, this way variables are reset along with the ending of the track"""
        if self.elapsed_time_secs >= self.total_track_secs:
            self.track_playing = False
            return True
        return False
        
        
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
        

    def GetTrackInfo(self, formatted_output = False): 
        """Subscribe to sound_player publisher and get elapsed track time"""
        song_data = self.request_song_data()
        
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

###BASIC METHODS###

    def CreateButton(self,file_name, alt_file_name, location,  scale=1):
        """code creates button using the button_image class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/guess_tone_game/graphics'
        file_path = os.path.join(this_file_path, relative_path, file_name)
        alt_path = os.path.join(this_file_path, relative_path, alt_file_name)
        
        button = IB(file_path, alt_path, location, self.pygame, scale)
        return(button)
    
    def CreateToggleButton(self,file_name, alt_file_name, location,  scale=1, return_info="", when_toggle_on=object, when_toggle_off=object):
        """code creates button using the button_image class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/guess_tone_game/graphics'
        file_path = os.path.join(this_file_path, relative_path, file_name)
        alt_path = os.path.join(this_file_path, relative_path, alt_file_name)
        
        button = TIB(file_path, alt_path, location, self.pygame, scale, return_info = return_info, when_toggle_on=when_toggle_on, when_toggle_off=when_toggle_off)
        return(button)
        
    def CreateDragButton(self,file_name, alt_file_name, location,  scale=1, return_info="", when_toggle_on=object, when_toggle_off=object):
        """code creates button using the button_image class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/guess_tone_game/graphics'
        file_path = os.path.join(this_file_path, relative_path, file_name)
        alt_path = os.path.join(this_file_path, relative_path, alt_file_name)
        
        button = IDB(file_path, alt_path, location, self.pygame, scale, return_info = return_info, when_toggle_on=when_toggle_on, when_toggle_off=when_toggle_off)
        return(button)


    def CreateHorizontalSlider(self, slider_name, cursor_name, x_y_locations, slider_scale=1, cursor_scale=1, on_click= object, on_release = object):
        """Creates horizontal slider using the horizontal slider class"""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/guess_tone_game/graphics'
        slider_path = os.path.join(this_file_path, relative_path, slider_name)
        cursor_path = os.path.join(this_file_path, relative_path, cursor_name)
        
        slider = HorizontalSlider(slider_path, cursor_path, x_y_locations, slider_scale, cursor_scale, on_click, on_release)
        return slider
       
       
    def send_qt_command(self, command_type, command_content= "", command_blocking = False):
        """Neatens and simplifies sending commands to QT """
        rospy.wait_for_service('/qt_command_service')
        command_controller = rospy.ServiceProxy('/qt_command_service', qt_command)
        command_complete = command_controller(command_type, command_content, command_blocking)
        
        return command_complete


    def DrawBackground(self):
        "Draws the background, and loads it each frame"
        self.window.fill((100, 100, 100)) #Fill background grey
    
        
    def DrawText(self, message, location, font_size = 30):
        """handle drawing text"""
        font = pygame.font.Font('freesansbold.ttf', font_size)
        text = font.render(message, False, (255,255,255))
        textRect = text.get_rect()
        textRect.center = location
        self.window.blit(text, textRect)

    
    def DrawMouseCursor(self, screen, mouse_pos = 0):
        """draw new cursor where the cursor is"""
        if mouse_pos == 0 : # If mouse pos was not supplied, get it from pygame
            my_mouse_pos = self.pygame.mouse.get_pos()
        else:
            my_mouse_pos = mouse_pos
        
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/guess_tone_game/graphics'
        file_name = "mouse_cursor.png"
        file_path = os.path.join(this_file_path, relative_path, file_name) 
            
        mouse_cursor_image = pygame.image.load(file_path).convert_alpha() #Load image of mouse cursor
        scaled_w = int(mouse_cursor_image.get_width()*1.5)
        scaled_h = int(mouse_cursor_image.get_height()*1.5)
        scaled_size= (scaled_w, scaled_h)
        scaled_cursor_image = pygame.transform.scale(mouse_cursor_image, scaled_size)
        screen.blit(scaled_cursor_image, my_mouse_pos) 


    def DrawTextCentered(self, message):
        """handle drawing text"""
        font = pygame.font.Font('freesansbold.ttf', 30)
        text = font.render(message, False, (255,255,255))
        textRect = text.get_rect()
        textRect.center = self.window_center
        self.window.blit(text, textRect)


    #TODO Add these to a new lib, QT CONTROL
    def send_qt_command(self, command_type, command_content= "", command_blocking = False):
        """Neatens and simplifies sending commands to QT """
        rospy.loginfo("Waiting for qt_command_service")
        rospy.wait_for_service('/qt_command_service')
        command_controller = rospy.ServiceProxy('/qt_command_service', qt_command)
        command_complete = command_controller(command_type, command_content, command_blocking)
        rospy.loginfo("completed handshake with qt_command_service")
        
        return command_complete
        
    def qt_say_blocking(self, text):
        """Makes QT say something, then makes you wait until the speaking is done"""
        timer_len = len(text) * 0.08 #0.2s per letter 4 letter word is given 0.8s to be said
        self.Timer.CreateTimer("QT_SAY_BLOCKING", timer_len) #creates timer with ID 1 for 8s   
        self.send_qt_command("tts", text)
        talking = True
        while talking and not rospy.is_shutdown():
            if self.Timer.CheckTimer("QT_SAY_BLOCKING"): #if our timer is done
                talking = False           
    
    def qt_say(self, text):
        """Makes QT say something, then makes starts a timer until the speaking is done"""
        timer_len = len(text) * 0.08 #0.2s per letter 4 letter word is given 0.8s to be said
        self.Timer.CreateTimer("QT_SAY", timer_len) #creates timer with ID 1 for 8s   
        self.send_qt_command("tts", text)         
    
    def qt_gesture(self,gesture):
        """Make QT do gesture, non blocking """
        self.send_qt_command("gesture", gesture)
    
    def qt_emote(self,emote):
        """Make QT emote """
        self.send_qt_command("emote", emote)
    
###HIGHER LEVEL METHODS###

    def transition_screen(self, text):
        """Method displays screen with only a single large button in the centre """
        if self.run: #Dont start this screen if the previous screen wanted to close out the game
            
            #Create central_button
            loading_button = self.CreateButton("loading_screen_button.png", "loading_screen_button_depressed.png", (0,0)) 
            loading_button.rect.center = self.window_center     #Center button
            
            quit_button = False
            
            while not quit_button and not rospy.is_shutdown() and self.run:
            
                #Draw background and objects
                self.DrawBackground()
                loading_button.render(self.window)
                self.DrawTextCentered(text)         #draw current time
                self.DrawMouseCursor(self.window)   
                self.pygame.display.update()        #Update all drawn objects
                
                #Start event handling
                for event in self.pygame.event.get():    
                    if event.type == self.pygame.QUIT:
                        self.run = False        #Stops the program entirely
                        quit_button = True      #Get out of loop
                                            
                    #Check for button press
                    if not quit_button: #dont check for this anymore after its triggered once
                        mouse_pos = self.pygame.mouse.get_pos()
                        quit_button = loading_button.get_event(event, mouse_pos) #This is the button in the middle incase we want to use it at some point       
                
                #Cap fps to 30
                #self.clock.tick(self.fps)
        
    def transition_screen_blocking(self, text_display, qt_say, should_gesture = True, gesture = "explain_right"):
        """Method displays screen with only a single large button in the centre """
        if self.run: #Dont start this screen if the previous screen wanted to close out the game
            
            #Create central_button
            loading_button = self.CreateButton("loading_screen_button.png", "loading_screen_button_depressed.png", (0,0)) 
            loading_button.rect.center = self.window_center     #Center button
            
            quit_button = False
            qt_speaking = True
            
            self.qt_emote("talking")
            self.Timer.CreateTimer("QT_MOUTH", 3) #creates timer with ID 1 for 8s 
            self.qt_say(qt_say)
            if should_gesture:
                self.qt_gesture(gesture)
            
            while qt_speaking and not rospy.is_shutdown() and self.run:
                #Draw background and objects
                self.DrawBackground()
                loading_button.render(self.window)
                self.DrawTextCentered(text_display) #draw current time
                self.DrawMouseCursor(self.window)
                self.pygame.display.update() #Update all drawn objects
                
                #Start event handling
                loading_button.block = True
                for event in self.pygame.event.get():
                    if event.type == self.pygame.QUIT:  #They hit quit
                        self.run = False                #Stops the program entirely
                                                                    
                    #Check for button press
                    if not quit_button: #dont check for this anymore after its triggered once
                        mouse_pos = self.pygame.mouse.get_pos()
                        quit_button = loading_button.get_event(event, mouse_pos) #This is the button in the middle incase we want to use it at some point       
                if self.Timer.CheckTimer("QT_SAY"): #If our timer is done
                    qt_speaking = False
                
                if self.Timer.CheckTimer("QT_SAY"):
                     self.qt_emote("talking")
                     self.Timer.CreateTimer("QT_MOUTH", 3) #creates timer with ID 1 for 8s 
    
    
    
    def return_wav_lenth(self,song_path):
        with contextlib.closing(wave.open(song_path,'r')) as f:
            frames = f.getnframes()
            rate = f.getframerate()
            duration = frames / float(rate)
            return(duration)
    
    
    def create_segment(self, num_segments, song_to_split):
        """Method that chops up a song and creates segments from it, this uses the file name, so you can use 2 differenty files and append the returned variabels together to jumble multiple songs"""
        self.path_to_save = r"/home/qtrobot/catkin_ws/src/musi_care/src/game_assets/music/temp/"
        #TODO change this method so that the correct one can be 1st, middle or last, instead of always 1st
        segments = []
        song_path = os.path.join(self.music_filepath, song_to_split)
        total_wav_len = (self.return_wav_lenth(song_path))*1000 #convert to millisecond
        slice_size = total_wav_len / num_segments
        prev_slice = 0 
        for i in range(0,num_segments):
            audio_segment = AudioSegment.from_wav(song_path)
            audio_segment = audio_segment[prev_slice : prev_slice+slice_size]
            prev_slice += slice_size
            song_path_save = self.path_to_save + str(i) + song_to_split[-9:] #cut off the problematic parts TODO change this to look for the "/" and cut after the "/"
            audio_segment.export(song_path_save, format="wav") #Exports to a wav file in the current path.       
            segments.append(song_path_save) #list of all songs made
            rospy.loginfo("temp file saved")
        return segments
             
    #TODO, create function that chooses random file from the music database
    
    def play_seg_track(self,song_path):
        """Uses the stored info attribute to play music on button press """
        
        def play_track():
            self.start_track(song_path)
        
        return play_track

        
    def stop_seg_track(self,song_path):
        """Uses the stored info attribute to play music on button press """
        
        def stop_track():
            self.stop_track()
        
        return stop_track
    
    #TODO add function to stop other toggle buttons when a different one is clicked        
    def drag_and_drop(self,difficulty, level):
        """populate screenwith dragable buttons and check if theyre in the correct spots"""
        self.segment_graphics = {
        1:("music_segment_play_blue.png","music_segment_pause_blue.png"), 
        2:("music_segment_play_orange.png","music_segment_pause_orange.png"), 
        3:("music_segment_play_purple.png","music_segment_pause_purple.png")} #blue = 1 orange = 2 purple = 3 #1 is reserved for the ones at the top
        
        #create buttons for the randomised segments
        segment_num = 2 #TODO replace this 2 with saved data for how manu segmants we want
        song = self.level_data[difficulty][level]
        correct_segments = self.create_segment(segment_num, song)
        segment_x_y = { 0: (450,450), 1:(750,450), 2:(450,750), 3:(750,750) } #temp hard coded num TODO: change these to calculated pos for each
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
        song_half = self.CreateToggleButton(self.segment_graphics[2][0], self.segment_graphics[2][1], (song_segment_y,90),return_info =correct_segments[0],  when_toggle_on = self.play_seg_track(correct_segments[0]), when_toggle_off = self.stop_seg_track(correct_segments[0]))
        song_unknown = self.CreateButton( "music_segment_greyed_out.png", "music_segment_greyed_out.png",(song_segment_y+150,90)) #TODO make this auto scale and spawn according to difficulty
        #play_button = self.create_toggle_button("play_button.png","pause_button.png",(song_segment_y+250,130))
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
                    
                       

    def play_music_blocking(self, difficulty, level): 
        
        song = self.level_data[difficulty][level]
        song_path = self.music_filepath + song
        
        #Create slider
        slider_y = 125
        slider_x = 450
        song_duration_slider = self.CreateHorizontalSlider("track_duration_slider.png", "track_cursor.png", (slider_y,slider_x))
        
        #Start track then pause it
        self.start_track(song_path)
        self.pause_unpause() #pause the song
        
        music_playing = True
        qt_spoken = False
        while music_playing and not rospy.is_shutdown() and self.run:
            #Format song time elapsed to display on screen
            formatted_data = self.GetTrackInfo(formatted_output = True)
            current_track_time = formatted_data[0]
            track_total_time = formatted_data[1] #Total track time
            progress = self.elapsed_time_secs / self.total_track_secs #elapsed time in percentage completion, so slider can represent that on a bar
            
            #Draw background and objects
            self.DrawBackground()
            self.DrawText(str(current_track_time), (165, slider_x +100)) #draw current time
            self.DrawText(str(track_total_time), (1240, slider_x+100)) #draw total track time
            self.DrawText("Please listen to the song", (700, 100 ), 50)
            song_duration_slider.render(self.window, progress)
            self.DrawMouseCursor(self.window)
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


    def yes_or_no_screen(self,text):
        """Screen for Yes or No questions"""
        #Create buttons
        yes = self.CreateButton("Yes_button.png", "Yes_button.png", (225,300), scale= 1.0)
        no = self.CreateButton("No_button.png", "No_button.png", (725,300), scale= 1.0)
        qt_hasnt_spoken = True
        while not rospy.is_shutdown() and self.run:
            self.DrawBackground()
            yes.render(self.window)
            no.render(self.window)
            self.DrawText(text, (700, 100 ), 50)
            self.DrawMouseCursor(self.window)
            self.pygame.display.update() #Update all drawn objects
            
            if qt_hasnt_spoken: #in here so he speaks while yes and no are drawn
                self.qt_emote("talking")
                self.qt_say_blocking("Should I explain how to play this game called, 'Fix the song' ?")
                qt_hasnt_spoken = False #QT has spoken now
                
            for event in self.pygame.event.get():
                if event.type == self.pygame.QUIT:
                    self.run = False #Stops the program entirely
                                                            
                #Check for button press
                mouse_pos = self.pygame.mouse.get_pos()
                clicked_yes = yes.get_event(event, mouse_pos)
                if clicked_yes:
                    return True
                else:
                    clicked_no = no.get_event(event, mouse_pos)
                    if clicked_no:
                        return False

    def yes_or_no_screen_custom(self,text, tts):
        """Screen for Yes or No questions"""
        #Create buttons
        yes = self.CreateButton("Yes_button.png", "Yes_button.png", (225,300), scale= 1.0)
        no = self.CreateButton("No_button.png", "No_button.png", (725,300), scale= 1.0)
        qt_hasnt_spoken = True
        while not rospy.is_shutdown() and self.run:
            self.DrawBackground()
            yes.render(self.window)
            no.render(self.window)
            self.DrawText(text, (700, 100 ), 50)
            self.DrawMouseCursor(self.window)
            self.pygame.display.update() #Update all drawn objects
            
            if qt_hasnt_spoken: #in here so he speaks while yes and no are drawn
                self.qt_emote("talking")
                self.qt_say_blocking(tts)
                qt_hasnt_spoken = False #QT has spoken now
                
            for event in self.pygame.event.get():
                if event.type == self.pygame.QUIT:
                    self.run = False #Stops the program entirely
                                                            
                #Check for button press
                mouse_pos = self.pygame.mouse.get_pos()
                clicked_yes = yes.get_event(event, mouse_pos)
                if clicked_yes:
                    return True
                else:
                    clicked_no = no.get_event(event, mouse_pos)
                    if clicked_no:
                        return False
            
            
###MAIN FUNCTIONS###
    
    
    def qt_counting_down(self):
        """Qt saying 3 2 1 go! """
        self.qt_say("get ready to clap along!")
        self.qt_say("3, 2, 1 go!")
        
        
    def Main(self):
        """Main function"""

        difficulty = "easy"
        level_num = 3
        
        explain = self.yes_or_no_screen("Should I explain how to play 'Fix the song' ?")

        if explain: #if they dont know how to play
            self.transition_screen_blocking("Please listen to QT's Explanation", "First; you will hear a piece of music... Then I will split it up into 2 parts... You will then have to put the pieces of music back together, in the correct order!... Press start when you are ready to begin!") 
        else:
            self.qt_say("Okay great, then press start when you are ready!")

        self.transition_screen("START GAME!")
        
        for level in range(1, level_num+1):
            #play the music
            self.play_music_blocking(difficulty, level)

            #Play the level
            self.drag_and_drop(difficulty, level)
            
            #Next level screen
            if level != level_num:
                self.transition_screen_blocking("Please listen to QT", "Nicely done, you have completed level {}. Press the start button when you are ready to start level {}.".format(level, level+1))
                self.transition_screen("Start level {}".format(level+1))
                
                    
if __name__ == '__main__':
    #Initialise game
    rospy.init_node('fix_song_game', anonymous=False)
    rospy.loginfo("Node launched successfully")
    game_object = Fix_Song_Game()

    #Run the game
    try:
        game_object.Main()
    except():
        game_object.pygame.quit
        rospy.loginfo("Audio may not be stopped due to interrupt")
    finally:
        print("Shutting game down")
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
