#!/usr/bin/env python

import rospy
import os
import pygame
import pygame.freetype
from musicare_lib import Button
from musicare_lib import AnimationManager
from musicare_lib import SoundManager

class test_pygame():
	
    def __init__(self):
        self.pygame = pygame
        self.pygame.init() #start py engine
        self.pygame.freetype.init()
        res = pygame.display.Info()
        self.window_x = res.current_w -150#Width of window -150 to account for the linux toolbar
        self.window_y = res.current_h  #Height of window
        self.window_center = (int(self.window_x/2), int(self.window_y/2))
        self.window = pygame.display.set_mode( (self.window_x, self.window_y) ) #Create window and set size
        self.pygame.display.set_caption("Testing Env!") #Label window
        self.run = True
        self.pygame.mouse.set_visible(False)
        self.sound_manager = SoundManager("/game_assets/music/") #load soundplayer with sound file path

    def DrawBackground(self):
        "Draws the background, and loads it each frame"
        self.window.fill((0, 0, 0)) #Fill background black


    def CreateButton(self,file_name, alt_file_name, location,  scale=1):
        """code creates button using the button_image class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/guess_tone_game/graphics'
        file_path = os.path.join(this_file_path, relative_path, file_name)
        alt_path = os.path.join(this_file_path, relative_path, alt_file_name)
        
        button = Button(file_path, alt_path, location, self.pygame, scale)
        return(button)
 

    def Main(self):
        song_name = "dont_fence_me_in_short.wav"
        
        
        animations = AnimationManager(self.pygame)
        test_button = self.CreateButton("loading_screen_button.png", "loading_screen_button_depressed.png", (300,0)) #create button
        test_button_2 = self.CreateButton("loading_screen_button.png", "loading_screen_button_depressed.png", (1700,0))
        buttons = [test_button, test_button_2] #add all buttons to list to control all at once
        pressed_button_id = 0 #Id to track which button is pressed
		
        while not rospy.is_shutdown() and self.run:
        
            #Event handle
            for event in self.pygame.event.get():
                if event.type == self.pygame.QUIT:  #They hit quit
                    self.run = False                #Stops the program entirely
                else:
                    mouse_pos = self.pygame.mouse.get_pos()
                    if event.type == self.pygame.MOUSEBUTTONUP:  #on mouse release play animation to show where cursor is
                        animations.StartTouchAnimation(mouse_pos) #tell system to play animation when drawing
                    for button in buttons:
                        button_pressed = button.get_event(event, mouse_pos)
                        if button_pressed:
                            pressed_button_id = button.id
                            break
        
            #Draw background and objects
            self.DrawBackground()
            for button in buttons:
                button.render(self.window)
            animations.DrawTouchAnimation(self.window)
            self.pygame.display.update() #Update all drawn objects


            #logic based on events
            if pressed_button_id == test_button.id:
                print("Button 1 pressed")
                self.sound_manager.load_track(song_name)
            elif  pressed_button_id == test_button_2.id:
                print("Button 2 pressed")
                self.sound_manager.pause_unpause()
            pressed_button_id = 0 #reset id
                    
                    
if __name__ == '__main__':
    #Initialise game
    rospy.init_node('pygame_test', anonymous=False)
    rospy.loginfo("Node launched successfully")
    game_object = test_pygame()

    #Run the game
    game_object.Main()
    game_object.pygame.quit
    
    
    
    
    
    
