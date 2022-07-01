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

#TODO REPLACE ALL GREY SCALE VERSIONS TO QUICK SCRIPT OF CV2.convert to greyscale

#####################################################General Levels##################################################################

class StandardLevels():
    """Class to draw basic screens multiple games use, such as yes or no screen """
    
    def __init__(self, window, window_center, pygame):
        self.window = window
        self.window_center = window_center
        self.pygame = pygame
        self.renderer = Renderer(window, window_center)
        self.animation_manager = AnimationManager(pygame)
        self.command_manager = QTManager()
    
    def yes_or_no_screen(self, text,run , background_colour):
        """Screen for Yes or No questions"""
        #Variables
        this_file_path = os.path.dirname(__file__)
        path_to_imgs = 'game_assets/graphics'
        yes_img_path = os.path.join(this_file_path, path_to_imgs, "Yes_button.png")
        no_img_path = os.path.join(this_file_path, path_to_imgs, "No_button.png")
        
        #Create buttons
        yes = Button(yes_img_path, yes_img_path, (225,500), self.pygame, scale= 2.2)
        no = Button(no_img_path, no_img_path, (1625,500), self.pygame, scale= 2.2)
        
        #Have QT act
        self.command_manager.qt_emote("talking")
        self.command_manager.qt_say("Should i explain the rules of the game called, 'Guess, the  mood'?")
        
        while not rospy.is_shutdown() and run:
            
            #Event handling
            for event in self.pygame.event.get():#Check if the user clicks the X
                if event.type == self.pygame.QUIT:
                    return "QUIT"
                elif(event.type == self.pygame.MOUSEBUTTONUP): #on mouse release play animation to show where cursor is
                    mouse_pos = self.pygame.mouse.get_pos() 
                    clicked_yes = yes.get_event(event, mouse_pos)
                    if clicked_yes:
                        return True
                    else:
                        clicked_no = no.get_event(event, mouse_pos)
                        if clicked_no:
                            return False
                    self.animation_manager.StartTouchAnimation(mouse_pos) #tell system to play animation when drawing
            
            #Draw graphics
            self.renderer.DrawBackground(background_colour)
            yes.render(self.window)
            no.render(self.window)
            self.renderer.DrawTextCentered(text, font_size = 100, y = 200) #top text
            self.animation_manager.DrawTouchAnimation(self.window) # also draw touches
            self.pygame.display.update() #Update all drawn objects


    def QTSpeakingScreen(self, qt_say, run, background_colour,  should_gesture = True, gesture = "explain_right", ):
        """Method displays background and text in centre"""
        text_display = "Please listen to QT robot"
        
        if run: #Dont start this screen if the previous screen wanted to close out the game
            
            self.command_manager.qt_emote("talking") #show mouth moving
            speaking_timer_id = self.command_manager.qt_say(qt_say) #says text we give it, and starts an internal timer that we can check on
            
            qt_speaking = True# used to tell us when to stop blocking
            if should_gesture:
                self.command_manager.qt_gesture(gesture)
            
            while qt_speaking and not rospy.is_shutdown() and run:
                #check for quit
                for event in self.pygame.event.get():
                    #Check if the user clicks the X
                    if event.type == self.pygame.QUIT:
                        return "QUIT"
                    elif(event.type == self.pygame.MOUSEBUTTONUP):#on mouse release play animation to show where cursor is
                        mouse_pos = self.pygame.mouse.get_pos() 
                        self.animation_manager.StartTouchAnimation(mouse_pos) #tell system to play animation when drawing
                        
                #Draw background and objects
                self.renderer.DrawBackground(background_colour)
                self.renderer.DrawTextCentered("Please listen to QT robot", font_size =70 )
                self.animation_manager.DrawTouchAnimation(self.window) # also draw touches
                self.pygame.display.update() #Update all drawn objects
                
                if self.command_manager.robo_timer.CheckTimer(speaking_timer_id): #If our timer is done
                    qt_speaking = False
                    return


    def QTSpeakingPopupScreen(self, qt_say, graphics, run, background_colour, should_gesture = True, gesture = "explain_right"):
        """Method displays all our gui with a popup infront with text in centre
           graphics = a dict of functions that are saved with the parameters needed to render them
        """
        #Variables
        this_file_path = os.path.dirname(__file__)
        path_to_imgs = 'game_assets/graphics'
        popup_path = os.path.join(this_file_path, path_to_imgs, "loading_screen_button.png")
        popup_path_grey = os.path.join(this_file_path, path_to_imgs, "loading_screen_button_depressed.png")

        text_display = "Please listen to QT robot"
        
        if graphics == {}:
            print("error: no graphics inputted into popup function")
            return
        
        if run or rospy.is_shutdown(): #Dont start this screen if the previous screen wanted to close out the game
                        
            #create popup button in center
            popup = Button(popup_path, popup_path_grey, (700,550), self.pygame, scale=1.5) 
            popup.rect.center = self.window_center
            
            self.command_manager.qt_emote("talking") #show mouth moving
            speaking_timer_id = self.command_manager.qt_say(qt_say) #says text we give it, and starts an internal timer that we can check on
            qt_speaking = True #used to tell us when to stop blocking
            if should_gesture:
                self.command_manager.qt_gesture(gesture)
            
            while qt_speaking and not rospy.is_shutdown() and run:
                #Track mouse button presses
                for event in self.pygame.event.get():    
                    #Check if the user clicks the X
                    if event.type == self.pygame.QUIT:
                        return "QUIT"
                    elif(event.type == self.pygame.MOUSEBUTTONUP):#on mouse release play animation to show where cursor is
                        mouse_pos = self.pygame.mouse.get_pos() 
                        self.animation_manager.StartTouchAnimation(mouse_pos) #tell system to play animation when drawing
                
                #Draw background and objects
                self.renderer.DrawBackground(background_colour)
                for key in graphics: # run each partial function
                    graphics[key]() #run as func             
                popup.render(self.window)
                self.renderer.DrawTextCentered("Please listen to QT robot", font_size =70 )
                self.animation_manager.DrawTouchAnimation(self.window) # also draw touches
                self.pygame.display.update() #Update all drawn objects
                
                if self.command_manager.robo_timer.CheckTimer(speaking_timer_id): #If our timer's internal timer is done
                    qt_speaking = False #unessecary but might as well 
                    return

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
    
    
    def DrawTextCentered(self, message, font_size = 30, font_colour=(255,255,255), x = -1, y = -1):
        """Draws text that's centered in X and Y"""
        font = pygame.font.Font('freesansbold.ttf', font_size)
        text = font.render(message, False, font_colour)
        textRect = text.get_rect()
        center = self.window_center
        if x < 0 and y < 0: #if no X and Y given center both
            textRect.center = center
        elif x > -1 and y < 0: #center Y but use the given X
            textRect.center = [x, center[1]]
        elif x < 0 and y > -1: #center x and use given Y
            textRect.center = [center[0], y]
        else: #if they gave both X and Y instead of using the other draw text func (for some reason?) use both x and y
            textRect.center = [x,y]

        self.window.blit(text, textRect)


#####################################################SoundManager##################################################################

class SoundManager():
    """Class to manage communication with sound_player service """

    def __init__(self, music_filepath):
        self.music_filepath = music_filepath
    
    def load_track(self, track_title, track_time=0.0):
        """gives the sound player the song data, has it load it up to return information about the song, this is essentially "start_track" but betteer """
        track_path = os.path.join(self.music_filepath, track_title)
        #print(track_path)
        #Start track
        operation = "load_track"
        song_data = self.call_sound_player(operation, track_path, track_time)
        rospy.sleep(0.15) #requires this to function consistently
        return song_data
        

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
        #print(track_path)
        #Start track
        operation = "start_track"
        callback_data = self.call_sound_player(operation, track_path, track_time)
        rospy.sleep(0.1) #requires this to function consistently
        song_data = self.request_song_data() 
        return song_data


    def stop_track(self):
        """Stop track, dont hold any data in memory"""
        operation = "stop_track"
        data = self.call_sound_player(operation) # we only need operation, the other variables can default, theyre ignored anyways
        rospy.sleep(0.1) #requires this to function consistently
        return data
        

    def pause_unpause(self):
        """Pause track and resume from the same point later"""
        operation = "pause_resume"
        data = self.call_sound_player(operation) # we only need operation, the other variables can default, theyre ignored anyways
        rospy.sleep(0.1) #requires this to function consistently
        return data
        
        
    def pause(self):
        """Pause track if it's playing"""
        operation = "pause"
        data = self.call_sound_player(operation) # we only need operation, the other variables can default, theyre ignored anyways
        rospy.sleep(0.1) #requires this to function consistently
        return data
        
        
    def unpause(self):
        """play track if it's paused"""
        operation = "resume"
        data = self.call_sound_player(operation) # we only need operation, the other variables can default, theyre ignored anyways
        rospy.sleep(0.1) #requires this to function consistently
        return data                


    def volume_change(self, volume_percentage):
        """change volume, 1.0 = 100%, 0.5 = 50% etc"""
        operation = "volume"
        volume = volume_percentage * 100 #other methods use decimal percentages, so for consistency this method does too, but then converts it to numerical percentage
        status = self.call_sound_player(operation, data_2 = volume).status # we only need operation and data_2 the other variable can default, it's ignored anyways
        rospy.sleep(0.1) #requires this to function consistently
        return status


    def request_song_data(self):
        """ask the service for data. TODO method needs reworking"""
        operation = "request_data"
        data = self.call_sound_player(operation)
        return data
        
        
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
            return None
    
    def GetTimers(self):
        return self.timers #return the timer list
        
#####################################################Button##################################################################

class Button():
    """
    class used for the generation and management of buttons
    """
    
    def __init__(self, image_path, image_greyscale_path, x_y_locations, pygame, scale=1, unique_id="", on_click=object, on_release=object):
        
        if not os.path.exists(image_path):
            print("File does not exist path = ", image_path)
        else:
            pass
            #print("File located at",image_path)
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
        if unique_id == "":
            self.id = rospy.get_time() #unique ID for each button based on time when made
        else:
            self.id = unique_id

    def render(self, screen, grey = False ):
        if grey: #if we get a request to pause show greyscaled version
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

    def __init__(self, default_image_path, toggled_image_path, default_image_grey, toggled_image_grey, x_y_locations, pygame, scale=1, unique_id = "", return_info="", when_toggle_on=object, when_toggle_off=object):
        #Set vars
        self.pygame = pygame
        self.highlighted = False
        self.return_info = return_info
        self.toggle_state = False
        self.when_toggle_on = when_toggle_on
        self.when_toggle_off = when_toggle_off

        #load imges
        raw_image = self.pygame.image.load(default_image_path).convert_alpha()
        raw_img_grey = self.pygame.image.load(default_image_grey).convert_alpha()
        toggled_raw_image = self.pygame.image.load(toggled_image_path).convert_alpha()
        toggled_raw_grey = self.pygame.image.load(toggled_image_grey).convert_alpha()
        
        #Scale and set pos of imgs
        img_x = x_y_locations[0]
        img_y = x_y_locations[1]
        img_w = int(raw_image.get_width()*scale)
        img_h = int(raw_image.get_height()*scale)
        scaled_size = (img_w, img_h)
        self.image = self.pygame.transform.scale(raw_image, scaled_size)
        self.image_grey = self.pygame.transform.scale(raw_img_grey, scaled_size)
        self.toggled_image = self.pygame.transform.scale(toggled_raw_image, scaled_size)
        self.toggled_image_grey = self.pygame.transform.scale(toggled_raw_grey, scaled_size)
        
        self.rect = self.pygame.Rect(img_x,img_y,img_w,img_h) 
        if unique_id == "":
            self.id = rospy.get_time() #unique ID for each button based on time when made
        else:
            self.id = unique_id

    def render(self, screen, grey = False):  
        """Draw image onto screen"""
        if self.toggle_state:
            if grey:
                screen.blit(self.toggled_image_grey, self.rect) #TODO replace this with the greyscaled version of this image
            else:
                screen.blit(self.toggled_image, self.rect)
        else:
            if grey:
                screen.blit(self.image_grey, self.rect) #TODO replace this with the greyscaled version of this image
            else:
                screen.blit(self.image, self.rect)
        return screen

    def store_info(self, info):
        """Stores info into correct attribute """
        self.return_info = info
    
    def toggle_toggle(self):
        """Toggles the function 'self.toggle' """
        self.toggle_state = not self.toggle_state
        if self.toggle_state:
            self.when_toggle_on()
        else:
            self.when_toggle_off()
        
        return (self.toggle_state)
    
    def toggle_img(self):
        """Toggle but only the img to render """
        self.toggle_state = not self.toggle_state
        return (self.toggle_state)
    
    def get_event(self, event, mouse_pos):
        """Button event handle, if mouse release, then toggle"""
        mouse_on_button = self.rect.collidepoint(mouse_pos)
        if mouse_on_button:
            if event.type == self.pygame.MOUSEBUTTONUP:
                return self.toggle_toggle()
        return self.toggle_state

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

    def render(self, screen, grey = False):  
        """Draw image onto screen"""
        if self.toggle:
            if grey:
                screen.blit(self.toggled_image, self.rect) #TODO replace this with the greyscaled version of this image
            else:
                screen.blit(self.toggled_image, self.rect)
        else:
            if grey:
                screen.blit(self.image, self.rect) #TODO replace this with the greyscaled version of this image
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
    def __init__(self, image_path_slider, image_path_cursor, x_y_locations, scale=1, on_click=object, on_release=object, music_filepath = "/game_assets/music/"):
        #init slider
        raw_slider_image = pygame.image.load(image_path_slider).convert_alpha()
        self.img_x = x_y_locations[0]
        self.img_y = x_y_locations[1]
        self.scale = scale
        slider_img_w = int(raw_slider_image.get_width()*self.scale)
        slider_img_h = int(raw_slider_image.get_height()*self.scale )
        scaled_size = (slider_img_w, slider_img_h)
        self.slider_image = pygame.transform.scale(raw_slider_image, scaled_size) #scale up the slider
        self.slider_rect = pygame.Rect(self.img_x,self.img_y,slider_img_w,slider_img_h) #make collision box around it according it it's size
        
        #init cursor
        raw_cursor_image = pygame.image.load(image_path_cursor).convert_alpha()
        cursor_img_w = int(raw_cursor_image.get_width()*self.scale)
        cursor_img_h = int(raw_cursor_image.get_height()*self.scale)
        self.half_cursor_height = cursor_img_h/2
        scaled_size = (cursor_img_w, cursor_img_h)
        self.cursor_image = pygame.transform.scale(raw_cursor_image, scaled_size)
        self.cursor_rect = pygame.Rect(self.img_x,self.img_y,cursor_img_w,cursor_img_h)
        
        #init variables
        self.slider_being_held = False
        self.bar_overwrite = 0.0 #percentage of total len, used to know how far along bar is being held
        self.slider_len = 1080 *self.scale  #total pixel length that the bar should extend to at the end of the track
        self.lower_bound_pix = 37 *self.scale #starting point of red bar (pixel_x)
        slider_min = self.img_x  + self.lower_bound_pix #starting point of red bar (pixel_x)
        slider_max = slider_min + self.slider_len #ending point of max red bar pixel X location of slider end point
        self.slider_range = (slider_min, slider_max) #min and max including space to left of bar
        self.on_click = on_click
        self.on_release = on_release
    
    def render(self, screen, progress, grey = False):
        """Draw slider, cursor and progress bar onto screen """
        if grey: #use grey graphics
            screen.blit(self.slider_image, self.slider_rect) #draw bar
            self.draw_progress_bar(screen, progress, grey) #draw the red progress bar
            self.draw_cursor(screen) #draw the cursor
        else: #render as normal
            screen.blit(self.slider_image, self.slider_rect) #draw bar
            self.draw_progress_bar(screen, progress) #draw the red progress bar
            self.draw_cursor(screen) #draw the cursor
     
    def draw_progress_bar(self, screen, progress, grey = False):
        """Uses a percentage to colour the completed relevant of the slider in red"""
        complete_bar_width = self.slider_len
        bar_height_pix = 57 *self.scale 
        bar_y_offset = 11 *self.scale #shifts progress bar to center of main bar
        bar_x = self.slider_range[0]  #starting x of progress bar
        bar_y = self.img_y + bar_y_offset  #starting y of bar - thickness of bar + given Y location
        if self.slider_being_held:
            bar_width = complete_bar_width*self.bar_overwrite #If the user is moving the slider, display their new slider 
        else:
            bar_width = complete_bar_width* progress
        if grey:
            bar_colour = (200,200,200)
        else:
            bar_colour = (255,0,0)
            
        self.cursor_x = bar_width + bar_x #starting X of bar + size of progress bar
        self.red_bar = pygame.draw.rect(screen, bar_colour, pygame.Rect((bar_x,bar_y), (bar_width, bar_height_pix)))
        
    def draw_cursor(self, screen):
        """uses progress to move cursor to where it should be. THIS SHOULD ALWAYS BE AFTER 'draw_progress_bar()' """
        cursor_y = self.img_y + int(self.half_cursor_height) #spawn cursor at half it's length to allign it
        self.cursor_rect.center = (self.cursor_x, cursor_y)
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
            #self.sound_manager.start_track(track_title, time_to_start)
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
                
        return self.slider_being_held

###########################################################END OF LIBRARY############################################################

