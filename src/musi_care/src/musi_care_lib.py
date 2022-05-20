#!/usr/bin/env python
#
#Library of useful classes that the other games use
#
#
import rospy
import pygame
import pygame.freetype

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
            
            
class ImageButton():
    """Class to load images that serve as buttons """
    
    
    def __init__(self, image_path, mouse_over_image_path, x_y_locations, pygame, scale=1, on_click=object, on_release=object):
        self.pygame = pygame
        raw_image = self.pygame.image.load(image_path).convert_alpha()
        mouse_over_raw_image = self.pygame.image.load(mouse_over_image_path).convert_alpha()
        img_x = x_y_locations[0]
        img_y = x_y_locations[1]
        img_w = int(raw_image.get_width()*scale)
        img_h = int(raw_image.get_height()*scale)
        
        scaled_size = (img_w, img_h)
        self.image = self.pygame.transform.scale(raw_image, scaled_size)
        self.mouse_over_image = self.pygame.transform.scale(mouse_over_raw_image, scaled_size)
        self.rect = self.pygame.Rect(img_x,img_y,img_w,img_h) 
        self.highlighted = False
        self.block = False 

    def render(self, screen):
        if self.highlighted or self.block:   
            screen.blit(self.mouse_over_image, self.rect)
        else:
            screen.blit(self.image, self.rect)
    

    def get_event(self, event, mouse_pos):
        mouse_on_button = self.rect.collidepoint(mouse_pos)
        if mouse_on_button:
            self.highlighted = True
        else:
            self.highlighted = False
            
        # If the mouse clicked while its highlighting this button
        if event.type == self.pygame.MOUSEBUTTONDOWN and event.button == 1:
            if mouse_on_button:
                return True
            else:
                return False

class ToggleImageButton():
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


class DragablePlayableImageButton():
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
                
                
                
                
                
                
                
