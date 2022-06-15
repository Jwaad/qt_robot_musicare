#!/usr/bin/env python

import pygame
import pygame.freetype
import time
import numpy as np
import sys
import rospy
import os
from random import randint
from musi_care.msg import SongData
from musi_care.srv import sound_player_srv
from musi_care.srv import qt_command
from musicare_lib import TimeFunctions 
from musicare_lib import Button
from musicare_lib import AnimationManager
from musicare_lib import SoundManager
from musicare_lib import QTManager
from musicare_lib import Renderer
from musicare_lib import HorizontalSlider

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
        self.window = pygame.display.set_mode( (self.window_x, self.window_y) ) #Create window and set size
        self.background_colour = (100,100,100) #background black by default
        self.pygame.display.set_caption("Testing Env!") #Label window
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
        relative_path = 'game_assets/guess_tone_game/graphics'
        file_path = os.path.join(this_file_path, relative_path, file_name)
        alt_path = os.path.join(this_file_path, relative_path, alt_file_name)
        
        button = Button(file_path, alt_path, location, self.pygame, scale, unique_id)
        return(button)         

    
    def CreateHorizontalSlider(self, slider_name, cursor_name, x_y_locations, scale=1, on_click= object, on_release = object):
        """Creates horizontal slider using the horizontal slider class"""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/guess_tone_game/graphics'
        slider_path = os.path.join(this_file_path, relative_path, slider_name)
        cursor_path = os.path.join(this_file_path, relative_path, cursor_name)
        
        slider = HorizontalSlider(slider_path, cursor_path, x_y_locations, scale, on_click, on_release)
        return slider
        
        
#################################################################Level / screen code################################################################


    def QTSpeakingScreen(self, qt_say, should_gesture = True, gesture = "explain_right"):
        """Method displays background and text in centre"""
        
        text_display = "Please listen to QT robot"
        
        if self.run: #Dont start this screen if the previous screen wanted to close out the game
            
            self.command_manager.qt_emote("talking") #show mouth moving
            speaking_timer_id = self.command_manager.qt_say(qt_say) #says text we give it, and starts an internal timer that we can check on
            qt_speaking = True# used to tell us when to stop blocking
            if should_gesture:
                self.command_manager.qt_gesture(gesture)
            
            while qt_speaking and not rospy.is_shutdown() and self.run:
                #Draw background and objects
                self.renderer.DrawBackground(self.background_colour)
                self.renderer.DrawTextCentered("Please listen to QT robot", font_size =70 )
                self.pygame.display.update() #Update all drawn objects
                
                if self.command_manager.robo_timer.CheckTimer(speaking_timer_id): #If our timer is done
                    qt_speaking = False



    def play_level(self, difficulty, level_num):
        """Sequence plays the levels"""
        if self.run: #Dont start this screen if the previous screen wanted to close out the game
            
            #Get the level's data
            level_data = self.music_data[difficulty][level_num] #{"song_name":"title", "mood":"happy", "hint":"some text"}
            self.track_name = level_data["song_name"]
            track_hint = level_data["hint"]
            track_mood = level_data["mood"]
            
            #Create buttons and slider
            self.sad_button = self.CreateButton("sad_button.png", "sad_button_depressed.png", (650,550), scale=1.3, unique_id="sad") 
            self.happy_button = self.CreateButton("happy_button.png", "happy_button_depressed.png", (650,950), scale=1.3, unique_id="happy") 
            self.unsure_button = self.CreateButton("unsure_button.png", "unsure_button_depressed.png", (650,1350), scale=1.3, unique_id = "unsure") 
            slider_scale = 2 #used for slider and for text adjacent to slider
            slider_x = 275
            slider_y = 200
            self.song_duration_slider = self.CreateHorizontalSlider("track_duration_slider.png", "track_cursor.png", (slider_x,slider_y), on_click = self.sound_manager.stop_track, on_release = self.sound_manager.start_track, scale=slider_scale)
            
            #Group elements
            self.buttons = [self.sad_button, self.happy_button, self.unsure_button]
            self.sliders = {self.song_duration_slider:"track_duration"} #Will be relevant eventually perhaps
            
            #Define variables & start track
            self.sound_manager.load_track(self.track_name) #load song to sound player and get data back
            self.level_complete = False #Check when level has been finished
            correct_answer_given = False
            track_stopped = True #this makes it play on start
            wrong_counter = 0
            first_iter = True # used to unpause for the 1st time
            
            #Main game loop
            while self.level_complete == False and not rospy.is_shutdown() and self.run:
                    
                if track_stopped: #if we had paused it, then unpause it here
                    if self.timer_manager.CheckTimer("delay_unpause") or first_iter: #if our timer finshes or we unpause
                        #self.sound_manager.unpause()
                        track_stopped = False
                        first_iter = False

                #Get variables that we will draw onto screen
                formatted_data = self.GetTrackInfo(formatted_output = True)
                if not self.song_duration_slider.slider_being_held: #If progress slider isn't being held just act as normal
                    current_track_time = formatted_data[0]          #Time gotten from sound_player node
                else:
                    current_track_time = current_track_time #Time when clicked on slider
                track_total_time = formatted_data[1] #Total track time
                progress = self.elapsed_time_secs / self.total_track_secs #elapsed time in percentage completion, so slider can represent that on a bar
                
                #Draw background and objects
                self.renderer.DrawBackground(self.background_colour)
                self.renderer.DrawText("What mood does this song have?", (1400, 100 ), 70)
                self.renderer.DrawText(str(current_track_time), (slider_x - 75, slider_y +75), font_size = 50) #draw current time
                self.renderer.DrawText(str(track_total_time), (2650, slider_y +75), font_size = 50) #draw total track time
                self.song_duration_slider.render(self.window, progress)
                for button in self.buttons: #Draw buttons using button list
                    button.render(self.window)
                self.animation_manager.DrawTouchAnimation(self.window) #last so it shows up on top
                self.pygame.display.update() #Update all drawn objects
            
                #Event handling
                self.track_data = self.GetTrackInfo() #Get data from the sound_player node
                
                #Start event handling
                for event in self.pygame.event.get():    
                    #reset / init variables      
                    option_chosen = ""
                    mouse_pos = self.pygame.mouse.get_pos()
                    
                    if event.type == self.pygame.MOUSEBUTTONUP:  #on mouse release play animation to show where cursor is
                        self.animation_manager.StartTouchAnimation(mouse_pos) #tell system to play animation when drawing
                    
                    #handle slider events
                    for slider in self.sliders.keys():
                        slider_held = slider.get_event(event, mouse_pos, self.track_data) #Give the slider track info, so it can pause and play from there.
                        
                    #Check which button is pressed, if any.
                    for button in self.buttons:
                        button_pressed = button.get_event(event, mouse_pos)
                        if button_pressed:
                            button_pressed_id = button.id
                            self.sound_manager.pause() #pause if something is playing
                            
                            track_stopped = True
                            #if clicked button is correct
                            if button_pressed_id == track_mood:
                                print("User has clicked the correct answer")
                                correct_answer_given= True
                                self.timer_manager.CreateTimer("delay_unpause", 3)#start a timer to let QT speak
                                self.command_manager.send_qt_command("emote", "happy")
                                self.command_manager.send_qt_command("gesture", "nod")
                                self.command_manager.send_qt_command("tts", "Good job, That is the right answer!") #QT reads out level's hint
                            #if clicked button is unsure --> give hint
                            elif button_pressed_id == "unsure":
                                print("User has clicked unsure")
                                self.timer_manager.CreateTimer("delay_unpause", 5)#start a timer to let QT speak
                                self.command_manager.send_qt_command("emote", "talking")
                                self.command_manager.send_qt_command("gesture", "explain_right")
                                self.command_manager.send_qt_command("tts", "I will give you a clue... " + track_hint) #QT reads out level's hint
                            #if clicked button is incorrect --> direct to hint if they want one.
                            elif button_pressed_id != track_mood: 
                                wrong_counter += 1 #how many time they have hit the wrong answer
                                if wrong_counter < 2:
                                    print("User has clicked the wrong answer")
                                    self.timer_manager.CreateTimer("delay_unpause", 4)#start a timer to let QT speak
                                    self.command_manager.send_qt_command("emote", "sad")
                                    self.command_manager.send_qt_command("gesture", "shake_head")
                                    self.command_manager.send_qt_command("tts", "Sorry, that is not the right answer, click, i dont know, for a hint") #QT reads out level's hint
                                else:
                                    print("User has clicked the wrong answer for the 2nd time")
                                    self.timer_manager.CreateTimer("delay_unpause", 6)#start a timer to let QT speak
                                    self.command_manager.send_qt_command("emote", "sad")
                                    self.command_manager.send_qt_command("gesture", "shake_head")
                                    self.command_manager.send_qt_command("tts", "Sorry, that is not the right answer, here is a hint.") #QT reads out level's hint
                                    self.command_manager.send_qt_command("tts", track_hint) #QT reads out level's hint
                                    
                                    
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

    def Main(self):
        self.play_level("easy", 1)
        self.QTSpeakingScreen("hello i am QT")
        

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
