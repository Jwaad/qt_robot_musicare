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
from musi_care_lib import TimeFunctions 

#TODO
#Change location code of buttons and sliders so that they scale with resolution (make a new option to allign center horizontal and vertical)
#pause and play button # use previous_track_data as this button needs to check if "track is playing", so that it can also act as a start track anew button, when track finishs or is stopped
#mute button
#Create slider for volume 
#Repeat button # use previous_track_data
#Hint button that tells you a hint    
    #Add database entry for each song that has a text based hint, QT should say these alloud.


class ImageButton():
    """Class to load images that serve as buttons """
    
    def __init__(self, image_path, alt_image_path, x_y_locations, scale=1):
        raw_image = pygame.image.load(image_path).convert_alpha()
        alt_raw_image = pygame.image.load(alt_image_path).convert_alpha()
        img_x = x_y_locations[0]
        img_y = x_y_locations[1]
        img_w = int(raw_image.get_width()*scale)
        img_h = int(raw_image.get_height()*scale)
        
        
        scaled_size = (img_w, img_h)
        self.image = pygame.transform.scale(raw_image, scaled_size)
        self.alt_image = pygame.transform.scale(alt_raw_image, scaled_size)
        self.rect = pygame.Rect(img_x,img_y,img_w,img_h) 
        self.highlighted = False

    def render(self, screen):
        if self.highlighted:   
            screen.blit(self.alt_image, self.rect)
        else:
            screen.blit(self.image, self.rect)
        
    def get_event(self, event, mouse_pos):
        mouse_on_button = self.rect.collidepoint(mouse_pos)
        if mouse_on_button:
            self.highlighted = True
        else:
            self.highlighted = False
            
        # If the mouse clicked while its highlighting this button
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if mouse_on_button:
                return True
            else:
                return False
                
                
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
        
    
class Guess_The_Mood_Game():

    def __init__(self):
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
        self.quit = False #Check to see if the game ended or it was quit
        self.block_events = False #Stop events except quit from happening, used when QT is talking.
        self.track_playing = False
        self.previous_track_data = None
        self.difficulty = "easy" #Default
        self.current_level = 1 #Default
        #self.music_data = {"easy":{1:{""}}, "medium":"", "hard":""} # {1:{"song_name":"title", "mood":"happy", "hint":"some text"}, 2:{"song_name":"title", "mood":"happy", "hint":"some text"}}
        self.get_song_database()
        #print(self.music_data)
        #self.default_volume = 1
        #self.volume_change(self.default_volume) # Set a default volume
        self.pygame.mouse.set_visible(False)
        self.timers = {}
        self.Timer = TimeFunctions()
        #self.music_filepath = "/home/qtrobot/catkin_ws/src/musi_care/src/game_assets/music/"
        self.music_filepath = "/game_assets/music/"
    
    def get_song_database(self):
        """read the database file and get the levels data"""
        #There are much better ways of doing this, this was the fastest way i knew how to do this. TODO redo this whole section with better coding perhaps use RegEX / someother database code

        #data_filepath = ("/home/qtrobot/catkin_ws/src/musi_care/src/game_assets/music/music_data.txt")
        data_filepath = ("game_assets/music/music_data.txt")
	
        with open (data_filepath, "r") as database:
            self.music_data = {"easy":{1:{""}}, "medium":{1:{""}}, "hard":{1:{""}}} #Reset data to overwrite it thouroughly
            raw_data = database.read().splitlines()

            #sort data into their difficulty tiers
            data = raw_data.copy() #Keep the raw data just in case we need it later
            for difficulty in self.music_data.keys():
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
            
        
    def check_track_ended(self):
        """Check if the track we played has ended, if it has, stop the track officially, this way variables are reset along with the ending of the track"""
        if self.elapsed_time_secs >= self.total_track_secs:
            self.track_playing = False
            return True
        return False
    
    
    def CreateButton(self,file_name, alt_file_name, location,  scale=1):
        """code creates button using the button_image class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/guess_tone_game/graphics'
        file_path = os.path.join(this_file_path, relative_path, file_name)
        alt_path = os.path.join(this_file_path, relative_path, alt_file_name)
        
        button = ImageButton(file_path, alt_path, location, scale)
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
    
    
    def qt_congradulations(self):
        """ Qt congradulates user for completing the game"""
        self.send_qt_command("emote", "grin")
        self.send_qt_command("tts","Congradulations, you have completed the  guess the mood game, now we will play a clapping game") #TODO remove this
        self.send_qt_command("gesture", "happy")
        
        
    def play_level(self,difficulty, level_num):
        """Sequence plays the levels"""
        if self.run: #Dont start this screen if the previous screen wanted to close out the game
            
            #Get the level's data
            level_data = self.music_data[difficulty][level_num] #{"song_name":"title", "mood":"happy", "hint":"some text"}
            track_name = level_data["song_name"]
            track_mood = level_data["mood"]
            track_hint = level_data["hint"]
            
            #Create buttons
            self.sad_button = self.CreateButton("sad_button.png", "sad_button_depressed.png", (750,450), scale=0.4) 
            self.happy_button = self.CreateButton("happy_button.png", "happy_button_depressed.png", (150,450), scale=0.4) 
            self.unsure_button = self.CreateButton("unsure_button.png", "unsure_button_depressed.png", (450,625), scale=0.4) 
            
            #Create sliders
            self.song_duration_slider = self.CreateHorizontalSlider("track_duration_slider.png", "track_cursor.png", (125,150), on_click = self.stop_track, on_release=self.start_track)
            
            #Group elements
            self.buttons = {self.happy_button:"happy", self.sad_button:"sad", self.unsure_button:"unsure"} #so we can use the buttons with less code, as they all do the same things
            self.sliders = {self.song_duration_slider:"track_duration"} #Will be relevant eventually
            
            ##QT introduces the level
            #self.send_qt_command("emote", "talking")
            #self.send_qt_command("tts","What mood does this song have?") #QT says, "What is the mood of this song?" #non
            #self.send_qt_command("gesture", "explain_right")
            
            #Define variables & start track
            #self.volume_change(0) # TODO after fixing service, remove this
            self.start_track(track_name) #Start the song to get song info back out
            self.pause_unpause() #Pause the song so QT can introduce it
            #self.volume_change(self.default_volume) # TODO after fixing service, remove this
            self.block_events = True #Also dont let any events run during this time (its a short time of around 0.4s)
            self.level_complete = False #Check when level has been finished
            qt_start_act_time = rospy.get_time()
            action_time = 2.5 #Time in seconds that action will take
            qt_is_busy = True
            correct_answer_given = False
            
            #Main game loop
            while self.level_complete == False and not rospy.is_shutdown() and self.run:
                
                #check if QT is busy to see if we block events or not
                if qt_is_busy:
                    self.block_events = True
                
                #Get variables that we will draw onto screen
                formatted_data = self.GetTrackInfo(formatted_output = True)
                if not self.song_duration_slider.slider_being_held: #If progress slider isn't being held just act as normal
                    current_track_time = formatted_data[0]          #Time gotten from sound_player node
                else:
                    current_track_time = current_track_time #Time when clicked on slider
                track_total_time = formatted_data[1] #Total track time
                progress = self.elapsed_time_secs / self.total_track_secs #elapsed time in percentage completion, so slider can represent that on a bar
                
                #Draw background and objects
                self.DrawBackground()
                self.DrawText("What mood does this song have?", (700, 100 ), 50)
                self.DrawText(str(current_track_time), (165, 250)) #draw current time
                self.DrawText(str(track_total_time), (1240, 250)) #draw total track time
                self.song_duration_slider.render(self.window, progress)
                for button in self.buttons.keys(): #Draw buttons using button dict
                    button.render(self.window)
                self.DrawMouseCursor(self.window)
                self.pygame.display.update() #Update all drawn objects
            
                #Check for key events
                self.check_track_ended() #must come after draw_text
            
                #At times it's convinient if the user cant click things. This will still have the screen update, it just wont respond. TODO add an indicator that the screen is locked.
                if self.block_events == False:
                    mouse_pos = self.pygame.mouse.get_pos()
                    track_data = self.GetTrackInfo() #Get data from the sound_player node
                    
                    #Start event handling
                    for event in self.pygame.event.get():    
                        #reset / init variables      
                        option_chosen = ""
                        
                        #handle slider events
                        for slider in self.sliders.keys():
                            slider_held = slider.get_event(event, mouse_pos, track_data) #Give the slider track info, so it can pause and play from there.
                            
                        #Check which button is pressed, if any.
                        for button in self.buttons.keys():
                            button_pressed = button.get_event(event, mouse_pos)
                            if button_pressed:
                                option_chosen = self.buttons[button]
                                break # since only one button would be clicked at once, break after the 1st one is clicked to save time
                        
                        #if clicked button is correct
                        if option_chosen == track_mood:
                            print("User has clicked the correct answer")
                            self.pause_unpause()
                            self.send_qt_command("emote", "happy")
                            self.send_qt_command("gesture", "nod")
                            self.send_qt_command("tts", "Good job, That is the right answer!") #QT reads out level's hint
                            action_time = 3.5
                            qt_start_act_time = rospy.get_time()
                            qt_is_busy = True
                            correct_answer_given = True # complete level after qt_is_free
                            
                        #if clicked button is unsure --> give hint
                        elif option_chosen == "unsure":
                            print("User has clicked unsure")
                            self.pause_unpause()
                            self.send_qt_command("emote", "talking")
                            self.send_qt_command("gesture", "explain_right")
                            self.send_qt_command("tts", "I will give you a clue... " + track_hint) #QT reads out level's hint
                            
                            action_time = 4.5
                            qt_start_act_time = rospy.get_time()
                            qt_is_busy = True
                           
                        #if clicked button is incorrect --> direct to hint if they want one.
                        elif option_chosen != "" and option_chosen != track_mood: 
                            print("User has clicked the wrong answer")
                            self.pause_unpause()
                            self.send_qt_command("emote", "sad")
                            self.send_qt_command("gesture", "shake_head")
                            self.send_qt_command("tts", "Sorry, that is not the right answer, click, i dont know, for a hint") #QT reads out level's hint
                            action_time = 6
                            qt_start_act_time = rospy.get_time()
                            qt_is_busy = True
                        
                        #Check if the user clicks the X
                        if event.type == self.pygame.QUIT:
                            self.run = False #Stops the program entirely
                            self.quit = True #Tells us that the game was quit out of, and it didn't end organically
                            self.level_complete = True # end level
                            self.stop_track() #Stop the music 
                         
                #If we block events, we should still check for quit
                else:
                    #still check for quit
                    for event in self.pygame.event.get():    
                        if event.type == self.pygame.QUIT:
                            self.run = False #Stops the program entirely
                            self.quit = True #Tells us that the game was quit out of, and it didn't end organically
                            self.level_complete = True # end level
                            self.stop_track() #Stop the music 
            
                    #See if qt is done so we can unpause game and song
                    if qt_is_busy:
                        if (rospy.get_time() - qt_start_act_time) >= action_time:
                            qt_is_busy = False
                            self.block_events = False
                            if correct_answer_given:
                                self.level_complete = True
                                print("Ending level")
                            else:
                                self.pause_unpause() #only unpause if we're not ending level
          
                # Cap fps to 30
                self.clock.tick(self.fps)
                    
            #Ending sequence after while loop
            if self.quit:
                print("You have quit the game.")
            else:
                print("You completed the level.")
                
            #close out before end
            #self.pygame.quit
            self.stop_track()
    
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
                self.qt_say_blocking("Should i explain the rules of the game called, 'Guess, the  mood'?")
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
                        
    def transition_screen_blocking(self, text_display, qt_say, should_gesture = True, gesture = "explain_right"):
        """Method displays screen with only a single large button in the centre """
        if self.run: #Dont start this screen if the previous screen wanted to close out the game
            
            #Create central_button
            loading_button = self.CreateButton("loading_screen_button.png", "loading_screen_button_depressed.png", (0,0)) 
            loading_button.rect.center = self.window_center     #Center button
            
            quit_button = False
            qt_speaking = True
            
            self.qt_emote("talking")
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
                        
    def Main(self):
        """Main loop to run the game"""

        explain = self.yes_or_no_screen("Should i explain the rules of 'Guess the mood'?")
        if explain:
            self.send_qt_command("emote", "talking")
            self.qt_say_blocking("You will hear a piece of music. You will have to choose whether the song seems happy, or it seems sad. Have fun!")
            self.send_qt_command("gesture", "explain_right")
        else:
            self.send_qt_command("emote", "talking")
            self.qt_say("Great, then when you are ready click start!")
            self.send_qt_command("gesture", "explain_right")
            
        self.transition_screen("Start game")
        
        difficulty = "easy"
        level_num = 2
        for level in range(1,level_num +1): #do levelnum amount of level
            print("Starting level", level, "of", difficulty, "mode")
            self.play_level(difficulty, level) # Play specified level
            
            #Next level screen
            if level != level_num:
                self.transition_screen_blocking("Please listen to QT", "Nicely done, you have completed level {}... Press the start button when you are ready to start level, {}.".format(level, level+1))
                self.transition_screen("Start level {}".format(level+1))
                self.qt_say_blocking("Okay, we are going to start level {}".format(level+1))
        
            if self.run == False:
                break
                
        #Close game
        self.pygame.quit
        
        if self.run:
            self.qt_congradulations()
        
        
        
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
    
