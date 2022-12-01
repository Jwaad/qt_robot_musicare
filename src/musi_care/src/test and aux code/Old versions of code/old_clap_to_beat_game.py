#!/usr/bin/env python

##TODO GENERAL
#Extract shared methods from the games and put that into libraries that they call on instead, to neaten code IE SOUND PLAYER LIB
#Create level selector script that launches the games, and is returned to when they close
    #This would also be for the technitian, this screen should reccomend what they play, as well as give data on the individual
    #this screen could also have settings and other such niceities
#Add a difficulty selector at the begining of all the games
#change dir of music so that all games can use the same songs if need be, will have to change the other games to match this.
#add a set speed in all the games, this game uses fast speed, so the other games should set that back to default, incase theyre played afterwards.

#TODO THIS GAME:
#currrently clicking the bar will end the level, fix that
#add a bar that shows you how much you missed the beat by. Have it update live

#TODO: check file bpm uses the whole song, then finds where the song bpm is most consistent, then crops from there. - should cause for good consistent bpm


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
from musi_care_lib import TimeFunctions 

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


class clap_to_beat_game():
    
    def __init__(self, difficulty):
        #rospy.init_node('clap_to_beat_game',anonymous=False)
        self.right_arm_pos_pub = rospy.Publisher('/qt_robot/right_arm_position/command', Float64MultiArray, queue_size=10)
        self.left_arm_pos_pub = rospy.Publisher('/qt_robot/left_arm_position/command', Float64MultiArray, queue_size=10)
        self.right_arm_hit = False
        self.left_arm_hit = True #default to using right arm 1st  
        rospy.wait_for_service('/qt_robot/motors/setControlMode')
        self.set_mode = rospy.ServiceProxy('/qt_robot/motors/setControlMode', set_control_mode)
        mode_changed = self.set_mode(["right_arm", "left_arm"], 1)
        if mode_changed:
            print("Motors successfully set control mode 1")
        else:
            print("Motor control mode could not be changed")
        rospy.wait_for_service('/qt_robot/motors/setVelocity')
        set_vel = rospy.ServiceProxy('/qt_robot/motors/setVelocity', set_velocity)
        speed_changed = set_vel(["right_arm", "left_arm"], 100) #set arms to max speed for this game
        if speed_changed:
            print("Motors successfully set to max speed")
        else:
            print("Motor speed could not be changed")
        self.setup_mic() #sets up mic and creates and sets many attributes
        print("Mic successfully setup")
        self.track_start = None
        self.raised_arm =  [-26.700000762939453, -85.4000015258789, -58.29999923706055]#[-10.800000190734863, -59.29999923706055, -42.0] #motor pos for right arm being raised
        self.hitting_drum = [17.200000762939453, -80.0999984741211, -46.599998474121094] # [26.700000762939453, -52.099998474121094, -57.70000076293945]#[-10.699999809265137, -59.599998474121094, -40.70000076293945] #motor pos for right arm hitting drum using drum sticks
        #Vars for pygame
        self.pygame = pygame
        self.pygame.init() #start py engine
        self.pygame.freetype.init()
        self.window_x = 1400 #Width of window
        self.window_y = 800  #Height of window
        self.window_center = (int(self.window_x/2), int(self.window_y/2))
        self.window = pygame.display.set_mode( (self.window_x, self.window_y) ) #Create window and set size
        self.pygame.display.set_caption("Clap to the beat!") #Label window
        self.fps = 30# set fps
        self.clock = pygame.time.Clock() #enable clock that will cap fps
        self.run = True #Decides when the game should end
        #self.default_volume = 1
        #self.volume_change(self.default_volume) # Set a default volume
        self.pygame.mouse.set_visible(False)
        ##TODO TEMP DATA THAT SHOULD BE REPLACED WITH A SAVED DATA FILE
        self.level_data = {"easy":{1:["dont_fence_me_in_long.wav", 47.5, 2.8], 2:"happy/happy_2.wav", 3:"happy/happy_3.wav"},"normal":{},"hard":{}}
        if difficulty == "easy" or difficulty == "normal" or difficulty == "hard":
            self.difficulty = difficulty
        else:
           self.difficulty = "easy" #default to easy mode
        self.timers = {}
        self.Timer = TimeFunctions()
        self.music_filepath = "/home/qtrobot/catkin_ws/src/musi_care/src/game_assets/music/"
        
    ###Play sound###
 
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


    def CreateButton(self,file_name, alt_file_name, location,  scale=1):
        """code creates button using the button_image class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/guess_tone_game/graphics'
        file_path = os.path.join(this_file_path, relative_path, file_name)
        alt_path = os.path.join(this_file_path, relative_path, alt_file_name)
        
        button = ImageButton(file_path, alt_path, location, scale)
        return(button)


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
    
    
    ###CONTROL QT###
    
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
        
    ###GET BPM FROM TRACK###

    def beats_to_bpm(self, beats, path):
        # if enough beats are found, convert to periods then to bpm
        if len(beats) > 1:
            if len(beats) < 4:
                print("few beats found in {:s}".format(path))
            bpms = 60./diff(beats)
            return median(bpms), beats
        else:
            print("not enough beats found in {:s}".format(path))
            return 0, [0,0,0]
                
                
    def get_file_bpm(self, path, params=None):
        #Calculate the beats per minute (bpm) of a given file.
        #path: path to the file
        #param: dictionary of parameters
        if os.path.exists(path):
            if params is None:
                params = {}
                
            # default:
            samplerate, win_s, hop_s = 44100, 1024, 512

            # manual settings
            if 'samplerate' in params:
                samplerate = params["samplerate"]
            if 'win_s' in params:
                win_s = params["win_s"]
            if 'hop_s' in params:
                hop_s = params["hop_s"]

            s = source(path, samplerate, hop_s)
            samplerate = s.samplerate
            o = tempo("specdiff", win_s, hop_s, samplerate)
            # List of beats, in samples
            beats = []
            # Total number of frames read
            total_frames = 0

            while True:
                samples, read = s()
                is_beat = o(samples)
                if is_beat:
                    this_beat = o.get_last_s()
                    beats.append(this_beat)
                total_frames += read
                if read < hop_s:
                    break

            return self.beats_to_bpm(beats, path)
        else:   
            print("File does not exist, check spelling of directory")
            return False



    def get_bpm(self,song_path):
        beat_data = self.get_file_bpm(song_path)
        bpm = beat_data[0]
        self.beat_timings = beat_data[1]
        return 


    ###QT CONTROL###

    def move_right_arm(self, joint_angles):
        #Function that moves the right arm to specified locations
        arm_msg = Float64MultiArray()
        arm_msg.data = [joint_angles[0], joint_angles[1], joint_angles[2]]
        self.right_arm_pos_pub.publish(arm_msg)


    def move_left_arm(self, joint_angles):
        #Function that moves the left arm to specified locations
        arm_msg = Float64MultiArray()
        arm_msg.data = [-joint_angles[0], joint_angles[1], joint_angles[2]]
        self.left_arm_pos_pub.publish(arm_msg)


    def hit_drum(self): 
        #Function that uses left or right arm to hit drum
        if self.left_arm_hit and not self.right_arm_hit: #move left arm back up and move right arm down
            #print("Hitting with right")
            self.move_right_arm(self.hitting_drum)
            self.move_left_arm(self.raised_arm)
            self.right_arm_hit = True
            self.left_arm_hit = False
            return True
        elif self.right_arm_hit and not self.left_arm_hit: #move right arm back up and left arm down
            #print("Hitting with left")
            self.move_right_arm(self.raised_arm)
            self.move_left_arm(self.hitting_drum) 
            self.right_arm_hit = False
            self.left_arm_hit = True
            return True
        else:
            print("Somehow both arms have hit the drum at once")
            return False
        return False


    ###GET TIMINGS OF CLAPS FROM DEFAULT MIC###
    
    def get_rms(self, block):
        # RMS amplitude is defined as the square root of the 
        # mean over time of the square of the amplitude.
        # so we need to convert this string of bytes into 
        # a string of 16-bit samples...

        # we will get one short out for each 
        # two chars in the string.
        count = len(block)/2
        format = "%dh"%(count)
        shorts = struct.unpack( format, block )

        # iterate over the block.
        sum_squares = 0.0
        for sample in shorts:
            # sample is a signed short in +/- 32768. 
            # normalize it to 1.0
            n = sample * self.SHORT_NORMALIZE
            sum_squares += n*n

        return math.sqrt( sum_squares / count )
    
    
    def setup_mic(self):
        #Set Vars
        self.pa = pyaudio.PyAudio()
        self.INITIAL_TAP_THRESHOLD = 0.15
        self.FORMAT = pyaudio.paInt16 
        self.SHORT_NORMALIZE = (1.0/32768.0)
        self.CHANNELS = 1
        self.RATE = 44100  
        self.INPUT_BLOCK_TIME = 0.05
        self.INPUT_FRAMES_PER_BLOCK = int(self.RATE*self.INPUT_BLOCK_TIME) # if we get this many noisy blocks in a row, increase the threshold

        #Init mic
        self.tap_threshold = self.INITIAL_TAP_THRESHOLD
        
        #Find input device index
        self.device_index = None        
        for i in range( self.pa.get_device_count() ):     
            devinfo = self.pa.get_device_info_by_index(i)   
            #print( "Device %d: %s"%(i,devinfo["name"]) )

            for keyword in ["mic","input"]:
                if keyword in devinfo["name"].lower():
                    #print( "Found an input: device %d - %s"%(i,devinfo["name"]) )
                    self.device_index = i
                    
        #Start stream
        self.stream = self.open_mic_stream()
                 
                   
    def open_mic_stream(self):
        stream = self.pa.open(   format = self.FORMAT,
                                 channels = self.CHANNELS,
                                 rate = self.RATE,
                                 input = True,
                                 input_device_index = self.device_index,
                                 frames_per_buffer = self.INPUT_FRAMES_PER_BLOCK)
        return stream
    
    
    def listen_for_clap(self):
        try:
            block = self.stream.read(self.INPUT_FRAMES_PER_BLOCK)
        except Exception as e:
            #print("ERROR OCCURED:", e)
            return
        
        #get amplitude of 0.05s section and check if it peaks higher than our thresh
        amplitude = self.get_rms(block)
        if amplitude > self.tap_threshold:
            print("Clap detected")
            return (True)
        
        return False
    
    
    def shutdown(self):
        #turn off and end processes properly
        rospy.sleep(1)
        print("Shutting down game")
        self.stop_track()
        self.stream.close()
        #TODO URGENT! Replace this with a nicer move to a default arm pos.
        #mode_changed = self.set_mode(["right_arm", "left_arm"], 0)
        #if mode_changed:
        #    print("Motors successfully set control mode 0")
        #else:
        #    print("Motor control mode could not be changed")
            
           
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
        
    def transition_screen_blocking(self, text_display, qt_say):
        """Method displays screen with only a single large button in the centre """
        if self.run: #Dont start this screen if the previous screen wanted to close out the game
            
            #Create central_button
            loading_button = self.CreateButton("loading_screen_button.png", "loading_screen_button_depressed.png", (0,0)) 
            loading_button.rect.center = self.window_center     #Center button
            
            quit_button = False
            qt_speaking = True
            
            self.qt_emote("talking")
            self.qt_say(qt_say)
            self.qt_gesture("explain_right")
            
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
            
            
    def get_next_beat(self, beats, time_elapsed):
        #given current time and all beats in track, return next beat
        for beat in beats:
            if time_elapsed < beat:
                return beat
        #if time elapsed > any all of our beats, that means we've hit them all and we're done
        return "Done"


    def closeness_checker(self, claps, beats, bpm):
        #Check how close the clapping was to the beats in percentage TODO rename to "timing"
        beats_hit = 0 #number of beats hit
        total_beats = len(beats)
        miss_thresh = (60 / bpm) /2 #time allowed is 1/2 a beat before or after a beat timing ie, if the beat is at 1s in a 60bpm song, then you can clap at 0.5s - 1.5s to count as a caught beat
        timing_list = [] #scores how accurate the beats that were hit were
        for beat in beats: #check using the beats, and the closest clap
            beat_hit_min = beat - miss_thresh
            beat_hit_max = beat + miss_thresh
            claps_temp = claps.copy()
            for clap in claps_temp: #find a clap that caught our beat
                if beat_hit_min < clap < beat_hit_max:
                    beats_hit += 1
                    rounded_clap = round(clap, 3)
                    rounded_beat = round(beat, 3)
                    sec_diff = abs(rounded_clap - rounded_beat)
                    if sec_diff == 0:
                        beat_acc = 1
                    else:
                        inverted_acc = miss_thresh - sec_diff #gives the accuracy, inverted time, such that longer time is a lower % accuracy
                        beat_acc = round((inverted_acc / miss_thresh))
                    timing_list.append(beat_acc)
                    break #if 2 claps are made for 1 beat, the earlier is taken of the 2
        hit_rate = round((beats_hit / total_beats) *100) #get % accuracy        
        if len(timing_list) > 0:
            accuracy = round(numpy.mean(timing_list)*100)
        else:
            accuracy = 0
        return hit_rate, accuracy
        #return 90, 80
        
    def show_level_result(self, claps, song_path, bpm):
        #Evaluate players level results and have QT read them out
        #TODO make all instances of this pass bpm to it
        #print(claps)
        score = self.closeness_checker(claps, self.beat_timings, bpm)
        print("You hit {}% of the beats".format(score[0]) )
        print("Your timing on each beat was {}% accurate".format(score[1]) )
        #self.send_qt_command("emote", "happy")
        #self.send_qt_command("tts", "You hit {} percent of the beats and your timing was {}% on time.".format(score[0], score[1]) )
        #self.send_qt_command("gesture", "arms_up")
        return score[0], score[1]
        
        
    def game_intro(self):
        self.send_qt_command("emote", "grin")
        self.send_qt_command("tts","I will play you a song, all you have to do is clap to the beat!")
        self.send_qt_command("gesture", "explain_right")
        
    
    def CreateHorizontalSlider(self, slider_name, cursor_name, x_y_locations, slider_scale=1, cursor_scale=1, on_click= object, on_release = object):
        """Creates horizontal slider using the horizontal slider class"""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/guess_tone_game/graphics'
        slider_path = os.path.join(this_file_path, relative_path, slider_name)
        cursor_path = os.path.join(this_file_path, relative_path, cursor_name)
        
        slider = HorizontalSlider(slider_path, cursor_path, x_y_locations, slider_scale, cursor_scale, on_click, on_release)
        return slider
    
    
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
                self.qt_say_blocking("Should i explain how to play the game called, 'clap to the beat'?")
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
              
    def transition_screen_blocking(self, text_display, qt_say):
        """Method displays screen with only a single large button in the centre """
        if self.run: #Dont start this screen if the previous screen wanted to close out the game
            
            #Create central_button
            loading_button = self.CreateButton("loading_screen_button.png", "loading_screen_button_depressed.png", (0,0)) 
            loading_button.rect.center = self.window_center     #Center button
            
            quit_button = False
            qt_speaking = True
            
            self.qt_emote("talking")
            self.qt_say(qt_say)
            #self.qt_gesture("explain_right")
            
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
                    mouse_pos = self.pygame.mouse.get_pos()
                    quit_button = loading_button.get_event(event, mouse_pos) #This is the button in the middle incase we want to use it at some point
                if self.Timer.CheckTimer("QT_SAY"): #If our timer is done
                    qt_speaking = False
                             
                        
    def play_level(self, level_num):
        #Get level data
        song = self.level_data[self.difficulty][level_num]
        song_name = song[0]
        bpm = song[1]
        first_beat = song[2]
        self.song_path = self.music_filepath + song_name 

        ##Load GUI
        #Create slider
        self.song_duration_slider = self.CreateHorizontalSlider("track_duration_slider.png", "track_cursor.png", (125,150), on_click = self.stop_track, on_release=self.start_track)
        
        #start and pause song, so that we can load it into the sound player
        self.start_track(self.song_path)
        self.pause_unpause()
        self.level_complete = False
        self.block_events = False
        self.claps = []
        #self.get_bpm(self.song_path)# Sets self.beat_timings
        self.beat_timings = []
        time_between_beats = 60/bpm #time between beat in seconds
        beat_timings_done = False
        i = 0
        
        #create beat timings based on bpm and 1st beat
        formatted_data = self.GetTrackInfo(formatted_output = True)
        while not beat_timings_done:
            next_beat = time_between_beats * i + first_beat
            if next_beat >= self.total_track_secs:
                beat_timings_done = True #end loop
            else:
                self.beat_timings.append(next_beat)
                i += 1
                
        #print(self.beat_timings)
        
        wait_for_beat = True
        prev_beat = 0
        time_to_hit = 0.3 # time taken approx for QT to finish a thwack of the drum
        
        """    
        colour_ind = [0,255,0]
        circle_x = 700
        circle_y = 500
        circle_radius = 100
        border_width = 0
        """
        
        self.pause_unpause()
        while self.level_complete == False and not rospy.is_shutdown():
            #main loop
            formatted_data = self.GetTrackInfo(formatted_output = True)
            if not self.song_duration_slider.slider_being_held: #If progress slider isn't being held just act as normal
                current_track_time = formatted_data[0]          #Time gotten from sound_player node
            else:
                current_track_time = current_track_time #Time when clicked on slider
            track_total_time = formatted_data[1] #Total track time
            progress = self.elapsed_time_secs / self.total_track_secs #elapsed time in percentage completion, so slider can represent that on a bar
            
            #Draw background and objects
            self.DrawBackground()
            self.DrawText(str(current_track_time), (165, 250)) #draw current time
            self.DrawText(str(track_total_time), (1240, 250)) #draw total track time
            self.song_duration_slider.render(self.window, progress)
            self.DrawMouseCursor(self.window)
            self.DrawText("Clap along!", (700, 100 ), 50)
            #self.pygame.draw.circle(self.window, colour_ind, (circle_x,circle_y), circle_radius, border_width)
            self.pygame.display.update() #Update all drawn objects
            
            #Check for key events
            self.check_track_ended() #must come after draw_text
            
            if self.block_events == False:
                mouse_pos = self.pygame.mouse.get_pos()
                track_data = self.GetTrackInfo() #Get data from the sound_player node
                
                ########################
            
                #check audio snippets for a clap
                clap_detected = self.listen_for_clap()
                if clap_detected:
                    self.claps.append(track_data[1]) #record time of track when clap detected
                
                #have robot hit the drum when it's time to
                if wait_for_beat:
                    beat = self.get_next_beat(self.beat_timings, track_data[1])
                    if beat == "Done":
                        pass #just wait til song ends and code changes segment
                    elif beat != prev_beat:
                        next_beat = beat
                        prev_beat = beat
                        wait_for_beat = False
                else:
                    if track_data[1] >= next_beat - time_to_hit: #hit drum at specific times
                        drum_hit = self.hit_drum()
                        wait_for_beat = True #next beat should find the next beat time           
                                        
                #######################
                
                #Start event handling
                for event in self.pygame.event.get():    
                    #reset / init variables      
                    option_chosen = ""
                    
                    #handle slider events
                    slider_held = self.song_duration_slider.get_event(event, mouse_pos, track_data) #Give the slider track info, so it can pause and play from there.
                        
                    #Check if the user clicks the X
                    if event.type == self.pygame.QUIT:
                        self.run = False #Stops the program entirely
                        self.level_complete = True # end level
                        self.stop_track() #Stop the music 
                
                #if track finish, level is complete
                if self.elapsed_time_secs >= self.total_track_secs or self.elapsed_time_secs==0:
                    #print("song has ended")
                    self.level_complete = True
                
                # Cap fps to 30
                #self.clock.tick(self.fps)
                            
            #If we block events, we should still check for quit
            else:
                #still check for quit
                for event in self.pygame.event.get():    
                    if event.type == self.pygame.QUIT:
                        self.run = False #Stops the program entirely
                        self.quit = True #Tells us that the game was quit out of, and it didn't end organically
                        self.level_complete = True # end level
                        self.stop_track() #Stop the music 
            
            """
            time_to_next_beat = next_beat - self.elapsed_time_secs
            if time_to_next_beat < 0.3:
                colour_ind = [0,255,0]
            else:
                colour_ind = [0,0,0]
            """
            
    def qt_counting_down(self):
        """Qt saying 3 2 1 go! """
        self.qt_say_blocking("Get ready to Clap following the beat!")
        self.qt_say_blocking("3, 2, 1, go!")
        rospy.sleep(2.5)
        
    def Main(self):
        #self.game_intro()
        #rospy.sleep(3) #let qt finish speaking before we start the game, we need the game to open before this tho, but for now this is fine
        self.run = True
        level_num = 1
        if self.run:
        
          #explanation screen
            explain = self.yes_or_no_screen("Should i explain how to play, 'Clap to the beat'?")
            if explain:
                self.send_qt_command("emote", "talking")
                self.qt_say_blocking("I will play a song with you. And you will need to clap along to the beat. I will be drumming along  with you!")
            else:
                self.send_qt_command("emote", "talking")
                self.qt_say("Great, then when you are ready click start!")
            
            #Starting screen
            self.transition_screen("START GAME")
            
            for level in range(1, level_num+1): #play levels 1 and 2
                print("Now playing level", level)
                
                #Play the level
                self.qt_counting_down()
                self.play_level(level)
                
                #Post level screen, qt tells shows you your results and congrat's you
                #score = self.show_level_result(self.claps, self.song_path)
                
                #screen to show level ended, but wait here to give results
                #self.transition_screen_blocking("Please listen to QT","You hit {} percent of the beats and your timing was {}% on time.".format(score[0], score[1]))
                
                #Next level screen
                if level != level_num:
                    self.transition_screen_blocking("Please listen to QT", "Nicely done, you have completed level {}. Press the start button when you are ready to start level {}.".format(level, level+1))
                    self.transition_screen("Start level {}".format(level+1))
                    self.qt_say_blocking("Okay, we are going to start level {}".format(level+1))
                    
                if not self.run:
                    break
                
            self.stop_track()
            
            
            
if __name__ == "__main__":
    rospy.init_node('clap_to_beat_game',anonymous=False)
    
    game = clap_to_beat_game("easy")
    
    try:
        game.Main()
        #game.play_level(1)
    except Exception as e:
        print(e)
        game.shutdown()
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        

