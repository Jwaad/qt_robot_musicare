#!/usr/bi   n/env python
#
#Library of useful classes that the other games use
#
#
import rospy
import pygame
import pygame.freetype
import os

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

