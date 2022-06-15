#!/usr/bi   n/env python
#
#Library of useful classes that the other games use
#
#
import rospy
import pygame
import pygame.freetype
import os
from musi_care.msg import SongData
from musi_care.srv import sound_player_srv
from musi_care.srv import qt_command

#####################################################Renderer##################################################################

class Renderer():
    """Class to render common things, such as background """
    
    def __init__(self, window, window_center):
        self.window = window
        self.window_center = window_center
        
    def DrawBackground(self, colour):
        """takes window and colour to fill in the background of the window """
        self.window.fill(colour) #Fill background black
    
    
    def DrawText(self, message, location, font_size = 30, font_colour=(255,255,255) ):
        """handle drawing text"""
        font = pygame.font.Font('freesansbold.ttf', font_size)
        text = font.render(message, False, font_colour)
        textRect = text.get_rect()
        textRect.center = location
        self.window.blit(text, textRect)
    
    
    def DrawTextCentered(self, message, font_size = 30, font_colour=(255,255,255)):
        """Draws text that's centered in X and Y"""
        font = pygame.font.Font('freesansbold.ttf', font_size)
        text = font.render(message, False, font_colour)
        textRect = text.get_rect()
        textRect.center = self.window_center
        self.window.blit(text, textRect)


#####################################################SoundManager##################################################################

class SoundManager():
    """Class to manage communication with sound_player service """

    def __init__(self, music_filepath):
        self.music_filepath = music_filepath
    
    def load_track(self, track_title, track_time=0.0):
        """gives the sound player the song data, has it load it up to return information about the song, this is essentially "start_track" but betteer """
        track_path = os.path.join(self.music_filepath, track_title)
        print(track_path)
        #Start track
        operation = "load_track"
        song_data = self.call_sound_player(operation, track_path, track_time)
        return song_data.status
        

    def call_sound_player(self, operation, data_1 = "", data_2 = 0.0 ):
        """makes it easier to call sound_player"""
        rospy.wait_for_service('/sound_player_service')
        sound_player = rospy.ServiceProxy('/sound_player_service', sound_player_srv)
        song_data = sound_player(operation, data_1, data_2)
        return song_data
            

    def start_track(self, track_title, track_time=0.0):
        """Starts a track and also saves the information returned as previous song, so we can replay songs without sending a new request"""
        #store default path to music
        
        track_path = os.path.join(self.music_filepath, track_title)
        print(track_path)
        #Start track
        operation = "start_track"
        callback_data = self.call_sound_player(operation, track_path, track_time)
        song_data = self.request_song_data() 
        
        return song_data


    def stop_track(self):
        """Stop track, dont hold any data in memory"""
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
        operation = "request_data"
        data = self.call_sound_player(operation)
        return data.status
        
        
#####################################################QTManager/CommandManager##################################################################
        

class QTManager():
    """Handles sending communication to robot """

    def __init__(self):
        self.robo_timer = TimeFunctions()
    
    def send_qt_command(self, command_type, command_content= "", command_blocking = False):
        """Neatens and simplifies sending commands to QT """
        rospy.wait_for_service('/qt_command_service')
        command_controller = rospy.ServiceProxy('/qt_command_service', qt_command)
        command_complete = command_controller(command_type, command_content, command_blocking)
        return command_complete
    
    def qt_say_blocking(self, text):
        """Makes QT say something, then makes you wait until the speaking is done"""
        timer_len = len(text) * 0.08 #0.2s per letter 4 letter word is given 0.8s to be said
        timer_id = "QT_SAY_BLOCKING"
        self.robo_timer.CreateTimer(timer_id, timer_len) #creates timer with ID 1 for 8s   
        self.send_qt_command("tts", text)
        talking = True
        while talking and not rospy.is_shutdown():
            if self.robo_timer.CheckTimer("QT_SAY_BLOCKING"): #if our timer is done
                talking = False
        
    def qt_say(self, text):
        """Makes QT say something, then makes starts a timer until the speaking is done"""
        timer_len = len(text) * 0.08 #0.08s per letter 4 letter word is given 0.32 secs to be said
        timer_id = "QT_SAY"
        self.robo_timer.CreateTimer(timer_id, timer_len) #creates timer with ID 1 for 8s   
        self.send_qt_command("tts", text)
        return timer_id
        
    
    def qt_gesture(self,gesture):
        """Make QT do gesture, non blocking """
        self.send_qt_command("gesture", gesture)
    
    def qt_emote(self,emote):
        """Make QT emote """
        self.send_qt_command("emote", emote)

#####################################################AnimationManager##################################################################

class AnimationManager():
    """Class for misselaneous functions"""
    
    def __init__(self, pygame):
        self.pygame = pygame
        self.play_touch_animation = False
    
    def StartTouchAnimation(self, mouse_pos):
        """Start animation """
        #animation vars
        self.colour = (180,180,180) #light grey
        self.circle_radius = 0 #will be changed dynamically
        self.border_width = 10 #0 = filled circle
        self.start_animation_time = rospy.get_time()
        self.animation_location = mouse_pos
        self.play_touch_animation = True

    def DrawTouchAnimation(self, window):
        """Handle / play animation """
        if self.play_touch_animation:
            time_since_start = (rospy.get_time() - self.start_animation_time) * 1000  #seconds elapsed since start convert to milliseconds
            if time_since_start < 400: #animation time = under half a sec
                scalar = time_since_start / 4 #low numbers = faster growth time
                max_size = 50
                if scalar < max_size:
                    self.circle_radius = scalar
                else:
                    self.circle_radius = max_size
                
                self.pygame.draw.circle(window, self.colour, self.animation_location, self.circle_radius, self.border_width) 
            else:
                self.play_touch_animation = False
   
#####################################################TimeFunctions##################################################################

class TimeFunctions():
    """Class to handle general functions such as time keeping"""
    
    def __init__(self):
        self.timers = {}
    
    def CreateTimer(self, timer_id, time_to_wait):
        """Add's time goal to timers list or replaces old timer """
        #Let it happen, but if a timer overwrites an old one, print warning
        if timer_id in self.timers.keys():
            print("You have overwritten an older timer, make sure you're not stuck in a loop")
        self.timers[timer_id] = rospy.get_time() + time_to_wait

    def CheckTimer(self, timer_id):
        if timer_id in self.timers.keys():
            if self.timers[timer_id] <= rospy.get_time():
                self.timers.pop(timer_id)
                return True #timer is done
            else:
                return False #Timer is not done yet
        else: #timer doesnt exist
            print("Timer referenced does not exist, check timer ID given")
    
    def GetTimers(self):
        return self.timers #return the timer list
        
#####################################################Button##################################################################

class Button():
    """
    class used for the generation and management of buttons
    """
    
    def __init__(self, image_path, image_greyscale_path, x_y_locations, pygame, scale=1, on_click=object, on_release=object):
        
        if not os.path.exists(image_path):
            print("File does not exist path = ", image_path)
        else:
            print("File located at",image_path)
        self.pygame = pygame
        raw_image = self.pygame.image.load(image_path).convert_alpha()
        grey_scaled_raw_image = self.pygame.image.load(image_greyscale_path).convert_alpha()
        img_x = x_y_locations[0]
        img_y = x_y_locations[1]
        img_w = int(raw_image.get_width()*scale)
        img_h = int(raw_image.get_height()*scale)
        scaled_size = (img_w, img_h)
        self.image = self.pygame.transform.scale(raw_image, scaled_size)
        self.image_greyscale_path = self.pygame.transform.scale(grey_scaled_raw_image, scaled_size)
        self.rect = self.pygame.Rect(img_x, img_y, img_w, img_h) 
        self.pause = False 
        self.id = rospy.get_time() #unique ID for each button based on time when made

    def render(self, screen):
        if self.pause: #if we get a request to pause show greyscaled version
            screen.blit(self.image_greyscale_path, self.rect)
        else:
            screen.blit(self.image, self.rect)
    
    def get_event(self, event, mouse_pos):
        # If the mouse clicked this button
        if event.type == self.pygame.MOUSEBUTTONUP and event.button == 1:
            if self.rect.collidepoint(mouse_pos):
                return True
            else:
                return False   
        else:
            return False 
            
######################################################ToggleButton#################################################################

class ToggleButton():
    """Class to load images that serve as buttons """

    def __init__(self, image_path, toggled_image_path, x_y_locations, pygame, scale=1, return_info="", when_toggle_on=object, when_toggle_off=object):
        self.pygame = pygame
        raw_image = self.pygame.image.load(image_path).convert_alpha()
        toggled_raw_image = self.pygame.image.load(toggled_image_path).convert_alpha()
        img_x = x_y_locations[0]
        img_y = x_y_locations[1]
        img_w = int(raw_image.get_width()*scale)
        img_h = int(raw_image.get_height()*scale)
        
        scaled_size = (img_w, img_h)
        self.image = self.pygame.transform.scale(raw_image, scaled_size)
        self.toggled_image = self.pygame.transform.scale(toggled_raw_image, scaled_size)
        self.rect = self.pygame.Rect(img_x,img_y,img_w,img_h) 
        self.highlighted = False
        self.block = False 
        self.return_info = return_info
        self.toggle = False
        self.when_toggle_on = when_toggle_on
        self.when_toggle_off = when_toggle_off

    def render(self, screen):  
        """Draw image onto screen"""
        if self.toggle or self.block:   #TODO, change the functionality of self.block to automatically greyscale these images and disable clicks
            screen.blit(self.toggled_image, self.rect)
        else:
            screen.blit(self.image, self.rect)
        return screen

    def store_info(self, info):
        """Stores info into correct attribute """
        self.return_info = info
    
    def toggle_toggle(self):
        """Toggles the function 'self.toggle' """
        self.toggle = not self.toggle
        if self.toggle:
            self.when_toggle_on()
        else:
            self.when_toggle_off()
        
        return (self.toggle)
    
    def get_event(self, event, mouse_pos):
        """Button event handle, if mouse release, then toggle"""
        if not self.block:
            mouse_on_button = self.rect.collidepoint(mouse_pos)
            if mouse_on_button:
                if event.type == self.pygame.MOUSEBUTTONUP:
                    return self.toggle_toggle()
            return self.toggle

######################################################DragableButton#################################################################

class DragableButton():
    """Class to load images that serve as buttons that can be dragged and dropped """
    
    def __init__(self, image_path, toggled_image_path, x_y_locations, pygame, scale=1, return_info="", when_toggle_on=object, when_toggle_off=object):
        self.pygame = pygame
        raw_image = self.pygame.image.load(image_path).convert_alpha()
        toggled_raw_image = self.pygame.image.load(toggled_image_path).convert_alpha()
        img_x = x_y_locations[0]
        img_y = x_y_locations[1]
        self.img_w = int(raw_image.get_width()*scale)
        self.img_h = int(raw_image.get_height()*scale)
        
        scaled_size = (self.img_w*scale, self.img_h*scale)
        self.image = self.pygame.transform.scale(raw_image, scaled_size)
        self.toggled_image = self.pygame.transform.scale(toggled_raw_image, scaled_size)
        self.rect = self.pygame.Rect(img_x,img_y,self.img_w,self.img_h) 
        self.highlighted = False
        self.block = False 
        self.return_info = return_info
        self.toggle = False
        self.when_toggle_on = when_toggle_on
        self.when_toggle_off = when_toggle_off
        self.mouse_is_held = False

    def render(self, screen):  
        """Draw image onto screen"""
        if self.toggle or self.block:   #TODO, change the functionality of self.block to automatically greyscale these images and disable clicks
            screen.blit(self.toggled_image, self.rect)
        else:
            screen.blit(self.image, self.rect)
        return screen

    def store_info(self, info):
        """Stores info into correct attribute """
        self.return_info = info
    
    def toggle_toggle(self):
        """Toggles the function 'self.toggle' """
        self.toggle = not self.toggle
        if self.toggle:
            self.when_toggle_on()
        else:
            self.when_toggle_off()
        
        return (self.toggle)
    
    def get_event(self, event, mouse_pos):
        """Button event handle, if mouse release, then toggle"""
        if not self.block:
            mouse_on_button = self.rect.collidepoint(mouse_pos)
            if mouse_on_button:
                if event.type == self.pygame.MOUSEBUTTONDOWN and not self.mouse_is_held:
                    self.mouse_is_held = True #only triggers the 1st time the mouse is down
                    self.initial_pos = mouse_pos
                if self.mouse_is_held:
                    if event.type == self.pygame.MOUSEBUTTONUP and mouse_pos == self.initial_pos:
                        self.mouse_is_held = False
                        return self.toggle_toggle(), self.rect
                    elif event.type == self.pygame.MOUSEBUTTONUP and mouse_pos != self.initial_pos:
                        self.mouse_is_held = False #mouse was released somewhere else
                    else:
                        self.rect.x = mouse_pos[0] - self.img_w/2
                        self.rect.y = mouse_pos[1] - self.img_h/2
            
            return self.toggle, self.rect
            
######################################################HorizontalSlider#################################################################

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
        bar_x = 160
        bar_y = self.slider_range[0]
        if self.slider_being_held:
            bar_width = complete_bar_width*self.bar_overwrite #If the user is moving the slider, display their new slider 
        else:
            bar_width = complete_bar_width*progress
        
        self.cursor_y = bar_width + bar_y
        self.red_bar = pygame.draw.rect(screen, (255,0,0), pygame.Rect((bar_x,bar_y), (bar_width, bar_height)))
        
        
    def draw_cursor(self, screen):
        """uses progress to move cursor to where it should be. THIS SHOULD ALWAYS BE AFTER 'draw_progress_bar()' """
        cursor_x = 160 + int(self.half_cursor_height) - 10
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

###########################################################END OF LIBRARY############################################################

