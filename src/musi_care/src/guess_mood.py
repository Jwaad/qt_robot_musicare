#!/usr/bin/env python

import pygame
import pygame.freetype
import time
import numpy as np
import sys
import rospy
import os
import functools
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

class Guess_The_Mood_Game():
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
        self.pygame.display.set_caption("Guess The Mood!") #Label window
        self.run = True
        #self.pygame.mouse.set_visible(False) #set to false when not testing
        self.quit = False #Check to see if the game ended or it was quit
        self.track_playing = False
        self.previous_track_data = None
        self.difficulty = "easy" #Default difficulty to play
        self.current_level = 1 #Default level to play
        self.music_data = {} #our database that we will write into 
        self.get_song_database() #From save file load all of the level data 
        self.music_filepath = "/game_assets/music/" #relative path to songs # "/home/qtrobot/catkin_ws/src/musi_care/src/game_assets/music/" 
        self.timer_manager = TimeFunctions()
        self.animation_manager = AnimationManager(self.pygame)
        self.sound_manager = SoundManager(self.music_filepath) #load soundplayer with sound file path
        self.command_manager = QTManager()
        self.renderer = Renderer(self.window,self.window_center)
        self.level_loader = StandardLevels(self.window, self.window_center, self.pygame)
        #self.music_vol = 1 # change volume of laptop
        #self.qt_voice_vol
        #self.sound_manager.volume_change(self.music_vol) # Set a default volume
        #self.set_robot_volume(qt_voice_vol) #TODO add this functionality  
                   
#################################################################Low level methods################################################################
                
    def get_song_database(self):
        """Read the database file and get the levels data"""

        #data_filepath = ("/home/qtrobot/catkin_ws/src/musi_care/src/game_assets/music/music_data.txt")
        data_filepath = ("/game_assets/data/gtm_level_data.txt") #gtm = guess the mood
        full_path = this_file_path = os.path.dirname(__file__) + data_filepath
	
        with open (full_path, "r") as database:
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


    def CreateButton(self,file_name, alt_file_name, location,  scale=1, unique_id=""):
        """code creates button using the button_image class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/graphics'
        file_path = os.path.join(this_file_path, relative_path, file_name)
        alt_path = os.path.join(this_file_path, relative_path, alt_file_name)
        
        button = Button(file_path, alt_path, location, self.pygame, scale, unique_id)
        return(button)         


    def CreateToggleButton(self, file_name, alt_file_name, file_grey, alt_file_grey, location,  scale=1, unique_id = "", return_info="", when_toggle_on=object, when_toggle_off=object):
        """code creates toggle button using the toggle button class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/graphics'
        file_path = os.path.join(this_file_path, relative_path, file_name)
        alt_path = os.path.join(this_file_path, relative_path, alt_file_name)
        file_path_grey = os.path.join(this_file_path, relative_path, file_grey)
        alt_path_grey = os.path.join(this_file_path, relative_path, alt_file_grey)
        
        button = ToggleButton(file_path, alt_path, file_path_grey, alt_path_grey, location, self.pygame, scale, unique_id, return_info, when_toggle_on, when_toggle_off)
        return(button)  
        
        
    def CreateHorizontalSlider(self, slider_name, cursor_name, x_y_locations, scale=1, on_click= object, on_release = object):
        """Creates horizontal slider using the horizontal slider class"""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/graphics'
        slider_path = os.path.join(this_file_path, relative_path, slider_name)
        cursor_path = os.path.join(this_file_path, relative_path, cursor_name)
        
        slider = HorizontalSlider(slider_path, cursor_path, x_y_locations, scale, on_click, on_release)
        return slider
        
        
#################################################################Level / screen code################################################################

    def guided_tut(self):
        """Code to play tut sequence for Guess the mood"""
        #Get the level's data
        level_data = self.music_data["tutorial"][1] #specific level data for our tutorial {"song_name":"title", "mood":"happy", "hint":"some text"}
        self.track_name = level_data["song_name"]
        track_hint = level_data["hint"]
        track_mood = level_data["mood"]
        

    def play_level(self, difficulty, level_num):
        """Sequence plays the levels"""
        if self.run: #Dont start this screen if the previous screen wanted to close out the game
            
            #Get the level's data
            level_data = self.music_data[difficulty][level_num] #{"song_name":"title", "mood":"happy", "hint":"some text"}
            self.track_name = level_data["song_name"]
            track_hint = level_data["hint"]
            track_mood = level_data["mood"]
            
            #Create buttons and slider
            self.sad_button = self.CreateButton("sad_button.png", "sad_button_depressed.png", (675,750), scale=1.3, unique_id="sad") 
            self.happy_button = self.CreateButton("happy_button.png", "happy_button_depressed.png", (675,1150), scale=1.3, unique_id="happy") 
            self.unsure_button = self.CreateButton("unsure_button.png", "unsure_button_depressed.png", (850,1550), scale=1, unique_id = "unsure") 
            self.play_button = self.CreateToggleButton("pause_button.png","play_button.png","pause_button_grey.png","play_button_grey.png", (self.cen_x-100, 175), scale = 4, when_toggle_on= self.sound_manager.pause, when_toggle_off = self.sound_manager.unpause) #create pause and play button
            slider_scale = 2 #used for slider and for text adjacent to slider
            slider_x = 275
            slider_y = 450
            self.song_duration_slider = self.CreateHorizontalSlider("track_duration_slider.png", "track_cursor.png", (slider_x,slider_y), on_click = self.sound_manager.stop_track, on_release = self.sound_manager.start_track, scale=slider_scale)
            
            #Group elements
            self.buttons = [self.sad_button, self.happy_button, self.unsure_button, self.play_button]
            self.sliders = [self.song_duration_slider] #Will be relevant eventually perhaps
            
            #Define variables & start track
            self.sound_manager.load_track(self.track_name) #load song to sound player and get data back
            self.track_data = self.GetTrackInfo() #Get data from the sound_player node for this track and save it
            self.level_complete = False #Check when level has been finished
            song_ended = False
            correct_answer_given = False
            track_stopped = True #this makes it play on start
            wrong_counter = 0
            first_iter = True # used to unpause for the 1st time
            graphics_list = []
            qt_message = ""
            
            self.sound_manager.unpause() #start track
            music_playing = True
            song_interrupt = False # track if we stopped song
            slider_was_held = False
            
            #Main game loop
            while self.level_complete == False and not rospy.is_shutdown() and self.run:

                #if the song ended, start player to the beginning and pause it.
                if song_ended: 
                    self.sound_manager.load_track(self.track_name) 
                    self.sound_manager.pause() #start song paused
                    music_playing = False
                    song_ended = False
        
                #Get variables that we will draw onto screen
                formatted_data = self.GetTrackInfo(formatted_output = True)
                if not self.song_duration_slider.slider_being_held: #If progress slider isn't being held just act as normal
                    current_track_time = formatted_data[0]          #Time gotten from sound_player node
                    track_total_time = formatted_data[1] #Total track time
                #else pass and dont update current time (ie use old time)
                progress = self.elapsed_time_secs / self.total_track_secs #elapsed time in percentage completion, so slider can represent that on a bar
                if progress >= 0.99: #if longer than 99% of the song has passed
                    song_ended = True
                
                #Draw background and objects
                self.renderer.DrawBackground(self.background_colour)
                self.renderer.DrawTextCentered("What mood does this song have?", font_size = 100, y = 100)
                self.renderer.DrawText(str(current_track_time), (slider_x - 75, slider_y +75), font_size = 50) #draw current time
                self.renderer.DrawText(str(track_total_time), (2650, slider_y +75), font_size = 50) #draw total track time
                self.song_duration_slider.render(self.window, progress)
                for button in self.buttons: #Draw buttons using button list
                    button.render(self.window)
                self.animation_manager.DrawTouchAnimation(self.window) #last so it shows up on top
                self.pygame.display.update() #Update all drawn objects
                
                #Copy each rendered obj to mem so we can draw them as grey
                self.rendered_graphics = {}
                self.rendered_graphics[1] = functools.partial(self.renderer.DrawTextCentered, "What mood does this song have?", font_size = 100, y = 100)
                self.rendered_graphics[2] = functools.partial(self.renderer.DrawText, str(current_track_time), (slider_x - 75, slider_y +75), font_size = 50) #draw current time
                self.rendered_graphics[3] = functools.partial(self.renderer.DrawText, str(track_total_time), (2650, slider_y +75), font_size = 50)
                self.rendered_graphics[4] = functools.partial(self.song_duration_slider.render, self.window, progress, grey = True)
                i = 4
                for button in self.buttons:
                    i+=1
                    self.rendered_graphics[i] = functools.partial(button.render, self.window, grey = True)
                
                #Start event handling
                for event in self.pygame.event.get():    
                    #reset / init variables      
                    option_chosen = ""
                    mouse_pos = self.pygame.mouse.get_pos()
                    
                    if event.type == self.pygame.MOUSEBUTTONUP:  #on mouse release play animation to show where cursor is
                        self.animation_manager.StartTouchAnimation(mouse_pos) #tell system to play animation when drawing
                    
                    #handle slider events
                    for slider in self.sliders:
                        slider_held = slider.get_event(event, mouse_pos, self.track_data) #Give the slider track info, so it can pause and play from there.
                        
                    #Events for pause button:
                    if slider_held: #since this will automatically resume the song, change our play button  & logic to match
                        self.play_button.toggle_state = False
                        music_playing = True
                        slider_was_held = False
                    else:                 
                        music_playing = not (self.play_button.get_event(event, mouse_pos))
                    
                    #Check which button is pressed, if any.
                    for button in self.buttons[:-1]: #for all buttons except the toggle button
                        button_pressed = button.get_event(event, mouse_pos)
                        if button_pressed:
                            button_pressed_id = button.id
                            self.sound_manager.pause() #pause if something is playing
                            if music_playing:
                                song_interrupt = True
                            track_stopped = True
                            #if clicked button is correct
                            if button_pressed_id == track_mood:
                                print("User has clicked the correct answer")
                                correct_answer_given= True
                                self.command_manager.send_qt_command("emote", "happy")
                                self.command_manager.send_qt_command("gesture", "nod")
                                qt_message = ("Good job, That is the right answer!") #QT reads out level's hint
                                self.level_loader.QTSpeakingPopupScreen(qt_message, self.rendered_graphics, self.run, self.background_colour) # this is blocking
                            #if clicked button is unsure --> give hint
                            elif button_pressed_id == "unsure":
                                print("User has clicked unsure")
                                self.command_manager.send_qt_command("emote", "talking")
                                self.command_manager.send_qt_command("gesture", "explain_right")
                                qt_message = ("I will give you a clue... " + track_hint) #QT reads out level's hint
                                self.level_loader.QTSpeakingPopupScreen(qt_message, self.rendered_graphics, self.run, self.background_colour) # this is blocking
                            #if clicked button is incorrect --> direct to hint if they want one.
                            elif button_pressed_id != track_mood: 
                                wrong_counter += 1 #how many time they have hit the wrong answer
                                if wrong_counter < 2:
                                    print("User has clicked the wrong answer")
                                    self.command_manager.send_qt_command("emote", "sad")
                                    self.command_manager.send_qt_command("gesture", "shake_head")
                                    qt_message = ( "Sorry, that is not the right answer, click, i dont know, for a hint") #QT reads out level's hint
                                    self.level_loader.QTSpeakingPopupScreen(qt_message, self.rendered_graphics, self.run, self.background_colour) # this is blocking
                                else:
                                    print("User has clicked the wrong answer for the 2nd time")
                                    self.command_manager.send_qt_command("emote", "sad")
                                    self.command_manager.send_qt_command("gesture", "shake_head")
                                    self.command_manager.send_qt_command("tts", "Sorry, that is not the right answer, here is a hint.") #QT reads out level's hint
                                    qt_message = (track_hint) #QT reads out level's hint
                                    self.level_loader.QTSpeakingPopupScreen(qt_message, self.rendered_graphics, self.run, self.background_colour) # this is blocking    
                            if song_interrupt: #if we had paused the music, resume it
                                self.sound_manager.unpause()
                                song_interrupt = False
                                    
                    #Check if the user clicks the X
                    if event.type == self.pygame.QUIT:
                        self.run = False #Stops the program entirely
                        self.quit = True #Tells us that the game was quit out of, and it didn't end organically
                        self.level_complete = True # end level
                        self.sound_manager.stop_track() #Stop the music 
                            
                #check if level won
                if correct_answer_given:
                    self.level_complete = True
                    print("Ending level")
          
                # Cap fps to 30
                #self.clock.tick(self.fps)
                    
            #Ending sequence after while loop
            if self.quit:
                print("You have quit the game.")
            else:
                print("You completed the level.")
                
            #close out before end
            self.pygame.quit
            self.sound_manager.stop_track()


#################################################################Main################################################################   

    def Main(self, difficulty = "easy", level =  1): #input what level and difficulty to play, the program will handle the rest
        """
        status = self.level_loader.QTSpeakingScreen("Lets play Guess the mood!", self.run, self.background_colour)
        if status == "QUIT": #if someone clicked quit during this screen then quit instead
            self.run = False
            self.quit = True
        tut = self.level_loader.yes_or_no_screen('Should I explain how to play "Guess The Mood" ?', self.run, self.background_colour)
        if tut:
            print("Playing guided tutorial")
        elif tut == "QUIT": #if someone clicked quit during this screen then quit instead
            self.run = False
            self.quit = True
        """
        self.play_level(difficulty, level)


#################################################################On execution################################################################      
                   
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
        SoundManager().stop_track()
        print("Audio may not be stopped due to interrupt")
