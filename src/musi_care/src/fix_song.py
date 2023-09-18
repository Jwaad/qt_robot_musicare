#!/usr/bin/env python

import pygame
import pygame.freetype
import time
import numpy as np
import sys
import rospy
import os
import functools
import math
import random
from random import randint
from musi_care.msg import SongData
from musi_care.srv import sound_player_srv
from musi_care.srv import qt_command
from musicare_lib import TimeFunctions
from musicare_lib import Button
from musicare_lib import DraggableButton
from musicare_lib import ToggleButton
from musicare_lib import PausePlayButton
from musicare_lib import AnimationManager
from musicare_lib import SoundManager
from musicare_lib import QTManager
from musicare_lib import Renderer
from musicare_lib import HorizontalSlider
from musicare_lib import StandardLevels
from musicare_lib import Behaviours
from musicare_lib import TextObject

#################################################################Initialise#################################################################

class Fix_The_Song_Game():
    """ Class to generate and handle guess the mood game """

    def __init__(self, reduce_screen=False, debug=False, screen = None, my_pygame = None):
        """Initialise and take user_id, user_id helps us save the data to the specific profiles"""
        x = 145  # x pos of screen
        y = 0  # y pos of screen
        os.environ['SDL_VIDEO_WINDOW_POS'] = '%d,%d' % (x, y)  # move screen to x and y pos

        if my_pygame == None:
            self.pygame = pygame
            self.pygame.init()  # start py engine
            self.pygame.freetype.init()
        else:
            self.pygame = my_pygame
        res = pygame.display.Info()  # get our screen resolution
        if screen == None:
            if reduce_screen:
                self.window_x = res.current_w - 150  # Width of window -150 to account for the linux toolbar
            else:
                self.window_x = res.current_w
            self.window_y = res.current_h  # Height of window
            self.window_center = (int(self.window_x / 2), int(self.window_y / 2))
            self.cen_x = self.window_center[0]
            self.cen_y = self.window_center[1]
            self.window = pygame.display.set_mode((self.window_x, self.window_y))  # Create window and set size
        else:
            self.window = screen
        self.background_colour = (100, 100, 100)  # background black by default
        self.pygame.display.set_caption("Fix The Song!")  # Label window
        self.run = True
        self.quit = False  # Check to see if the game ended or it was quit
        self.track_playing = False
        self.previous_track_data = None
        self.difficulty = "easy"  # Default difficulty to play
        self.current_level = 1  # Default level to play
        self.music_filepath = "/game_assets/music/"  # relative path to songs # "/home/qtrobot/catkin_ws/src/musi_care/src/game_assets/music/"
        self.timer_manager = TimeFunctions()
        self.animation_manager = AnimationManager(self.pygame)
        self.sound_manager = SoundManager(self.music_filepath)  # load soundplayer with sound file path
        self.command_manager = QTManager()
        self.renderer = Renderer(self.window, self.window_center)
        self.level_loader = StandardLevels(self.window, self.window_center, self.pygame, self.music_filepath)
        self.segment_x_y = {0: (610, 850), 1: (1260, 850), 2: (1910, 850), 3: (610, 1400), 4: (1260, 1400),
                            5: (1910, 1400)}  # hard coded num locations of each segment
        self.behaviours_manager = Behaviours(self.pygame, self.music_filepath)
        self.t1 = 0  # t1 for FPS tracking
        self.debug = debug
        if not self.debug:
            self.pygame.mouse.set_visible(False) #set to false when not testing
        # self.music_vol = 1 # change volume of laptop
        # self.qt_voice_vol
        # self.sound_manager.volume_change(self.music_vol) # Set a default volume
        # self.set_robot_volume(qt_voice_vol) #TODO add this functionality
        self.segment_graphics = {
            0: ("music_segment_play_grey.png", "music_segment_pause_grey.png"),
            1: ("music_segment_play_blue.png", "music_segment_pause_blue.png"),
            2: ("music_segment_play_orange.png", "music_segment_pause_orange.png"),
            3: ("music_segment_play_purple.png", "music_segment_pause_purple.png"),
            4: ("music_segment_play_green.png", "music_segment_pause_green.png"),
            5: ("music_segment_play_pink.png", "music_segment_pause_pink.png")
        }
        self.e_clear_multiplier = 1.5 # track total time * this  = e_clear req time

    ########################################################Low level methods################################################################

    def empty_temp_dir(self, segments):
        """Takes a list of segments / dragable objects and deletes the sound file they play"""
        path = os.path.dirname(__file__) + self.music_filepath
        file_paths =  []
        for seg in segments:
            file_paths.append(seg.return_info["song_path"])
        # Delete files
        for file_path in file_paths:
            file = path + file_path
            if os.path.exists(file):
                os.remove(file)

        rospy.loginfo("Temp files deleted")

    def get_track_info(self, formatted_output=False):
        """Subscribe to sound_player publisher and get elapsed track time"""

        song_data = self.sound_manager.request_song_data()

        self.track_title = song_data.track_title  # track title
        self.total_track_secs = song_data.track_total_time  # track time in seconds
        self.elapsed_time_secs = song_data.track_elapsed_time  # current time in seconds

        # Either return the above, or format the secs to be displayed
        if formatted_output:
            total_mins, total_secs = self.format_elapsed_display(self.total_track_secs)
            elapsed_mins, elapsed_secs = self.format_elapsed_display(self.elapsed_time_secs)

            elapsed_time = elapsed_mins + ":" + elapsed_secs
            total_time = total_mins + ":" + total_secs

            return elapsed_time, total_time
        else:
            return self.track_title, self.elapsed_time_secs, self.total_track_secs

    def get_song_info(self):
        """ Talks to the music player and gets the data / information of the currently playing song"""
        formatted_data = self.get_track_info(formatted_output=True)
        self.current_track_time = formatted_data[0]  # Time gotten from sound_player node
        self.track_total_time = formatted_data[1]  # Total track time
        self.progress = self.elapsed_time_secs / self.total_track_secs  # elapsed time in percentage completion, so slider can represent that on a bar
        self.song_ended = self.progress >= 0.99  # if self.progress > 99% = song is finished, otherwise false
        return self.current_track_time, self.track_total_time, self.progress, self.song_ended

    def format_elapsed_display(self, time):
        "bit of code that converts secs to mins and secs"
        if time < 60:
            secs = int(time)
            mins = str(0)
        else:
            secs = int(time % 60)
            mins = str(int((time - secs) / 60))
        if secs < 10:
            secs = "0" + str(secs)
        return str(mins), str(secs)

    def create_button(self, file_name, location, return_info = {}, scale=1, unique_id="", should_grey= True):
        """code creates button using the button_image class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/graphics'
        file_path = os.path.join(this_file_path, relative_path, file_name)

        button = Button(file_path, location, self.pygame, return_info = {}, scale=scale, unique_id=unique_id, should_grey = should_grey)
        return (button)

    def create_toggle_button(self, file_name, alt_img, location, scale=2,
                             unique_id="", return_info="", when_toggle_on=object, when_toggle_off=object, should_grey = True):
        """code creates button using the button_image class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = '/game_assets/graphics/'
        file_path = this_file_path + relative_path + file_name
        alt_img_path = this_file_path + relative_path + alt_img

        button = ToggleButton(file_path, alt_img_path, location, self.pygame, scale,
                              unique_id, return_info, when_toggle_on, when_toggle_off, should_grey = should_grey)
        return (button)

    def create_drag_button(self, file_name, toggled_file, location, scale=2, return_info={},
                           when_toggle_on=object, when_toggle_off=object, unique_id="", should_grey = True):
        """code creates button using the button_image class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/graphics'

        # Form img paths
        file_path = os.path.join(this_file_path, relative_path, file_name)
        toggled_path = os.path.join(this_file_path, relative_path, toggled_file)

        button = DraggableButton(file_path, toggled_path, location, self.pygame, scale,
                                 return_info=return_info, when_toggle_on=when_toggle_on,
                                 when_toggle_off=when_toggle_off, unique_id=unique_id, should_grey = should_grey)
        return (button)

    def create_play_button(self, file_name, alt_file_name, rewind_name,
                           location, scale=1, unique_id="", on_pause=object, on_play=object, should_grey = True):
        """code creates toggle button using the toggle button class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/graphics'

        # Load imgs
        file_path = os.path.join(this_file_path, relative_path, file_name)
        alt_path = os.path.join(this_file_path, relative_path, alt_file_name)
        rewind_path = os.path.join(this_file_path, relative_path, rewind_name)

        button = PausePlayButton(file_path, alt_path, rewind_path,
                                 location, self.pygame, scale, unique_id, on_pause, on_play, should_grey = should_grey)
        return (button)

    def create_horizontal_slider(self, slider_name, cursor_name, x_y_locations, scale=1, on_click=object,
                                 on_release=object):
        """Creates horizontal slider using the horizontal slider class"""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/graphics'
        slider_path = os.path.join(this_file_path, relative_path, slider_name)
        cursor_path = os.path.join(this_file_path, relative_path, cursor_name)

        slider = HorizontalSlider(slider_path, cursor_path, x_y_locations, scale, on_click, on_release)
        return slider

    def create_text(self,window, window_center, text, location=None, cen_x=False, cen_y=False, font_size=30,
               font_colour=(255, 255, 255)):
        text_object = TextObject(window, window_center, text, location=location, cen_x=cen_x, cen_y=cen_y, font_size=font_size,
               font_colour=font_colour)
        return text_object

    def update_graphics(self, given_half, draggable_buttons, main_buttons):
        """Redraw graphics """
        self.renderer.DrawBackground(self.background_colour)
        given_half.render(self.window)
        for button in main_buttons:
            button.render(self.window)
        self.renderer.DrawTextCentered("Put the song back together", font_size=100, y=50)
        self.renderer.DrawTextCentered("Drag the segments below into the empty slot above", font_size=100, y=550, )
        for key in draggable_buttons:  # render draggable buttons
            draggable_buttons[key].render(self.window)  # draw the segments
        self.animation_manager.DrawTouchAnimation(self.window)  # last so it shows up on top

    def highlight_block(self, events, target_rect=None, msg="Click anywhere to continue ... ", timer_complete=None):
        """
        Highlight a certain object and let QT explain it, it will then move on on it's own
        target_graphic = the graphic that's in colour
        Graphics here are a dict of the partial functions for each renderer in grey
        target_rect the rect that we want to highlight around
        should be used by being kept in a while loop with other graphics to be drawn
        """

        # Handle events
        for event in events:
            # reset / init variables
            option_chosen = ""
            mouse_pos = self.pygame.mouse.get_pos()
            if event.type == self.pygame.MOUSEBUTTONUP:  # on mouse release play animation to show where cursor is
                self.animation_manager.StartTouchAnimation(mouse_pos)  # tell system to play animation when drawing
                # return True

        # Render graphics
        if target_rect != None:  # so we can have blocking functionality without highlighting
            self.renderer.HighlightRect(target_rect, self.pygame)  # draw arrow and box
        if msg != "":  # we can send an empty msg to msg instead to have it not display anything
            if timer_complete == None:  # if user didnt specify a timer, just show text like normal
                self.renderer.DrawTextCentered(msg, font_size=75, font_colour=(0, 0, 0))
            else:
                if timer_complete:  # only render text once timer done
                    self.renderer.DrawTextCentered(msg, font_size=75, font_colour=(0, 0, 0))

        # return False

    def load_list_graphics(self, graphics, keys):
        """takes graphics dict and loads them
            Graphics = {1: graphic_func_1, 2: graphic_func_2. etc}
            key = [1,3,4,5]
        """
        for key in graphics.keys():  # Draw each object
            if key in keys:  # only draw the graphics we ask for
                graphics[key]()  # run as func

    def randomise_colour(self, prev_colour):
        """Keep shuffling until we get a colour different from the last."""
        num_colours = len(self.segment_graphics) -1
        seg_colour = self.segment_graphics[random.randint(1, num_colours)]
        while seg_colour == prev_colour:
            seg_colour = self.segment_graphics[random.randint(1, num_colours)]
        return seg_colour

    def create_segments(self,segment_num, correct_track, distract_track):
        """Create and return a list of segments (draggable buttons) """
        # Get each segment position

        correct_segments, distract_segments = self.sound_manager.slice_song(segment_num, correct_track, distract_track)
        all_segs = []

        # Make objects of correct segments
        i = 0  # pos in correct segs
        prev_colour = 0
        for song_path in correct_segments:

            seg_colour = self.randomise_colour(prev_colour)
            prev_colour = seg_colour
            button_grey = self.segment_graphics[0][0]
            seg_pos = [0, 0]  # Default pos, this will be changed later
            seg_data = {
                "song_path": song_path,
                "correct_slot":False,
                "song_pos": i
            }
            segment = self.create_drag_button(seg_colour[0], seg_colour[1], seg_pos,
                                              return_info=seg_data,
                                              when_toggle_on=self.play_seg_track(song_path),
                                              when_toggle_off=self.stop_seg_track(song_path))
            all_segs.append(segment)
            i += 1

        # Make objects of distract segments
        for song_path in distract_segments:
            seg_colour = self.randomise_colour(prev_colour)
            prev_colour = seg_colour

            button_grey = self.segment_graphics[0][0]
            seg_pos = [0, 0]  # default pos, this will be changed later
            seg_data = {
                "song_path": song_path,
                "correct_slot":False,
                "song_pos": None
            }
            segment = self.create_drag_button(seg_colour[0], seg_colour[1], seg_pos,
                                              return_info=seg_data,
                                              when_toggle_on=self.play_seg_track(song_path),
                                              when_toggle_off=self.stop_seg_track(song_path))
            all_segs.append(segment)

        return all_segs, len(correct_segments)


    def create_graphics(self, segments, num_correct_slots, single_out = False):
        """Create the pygame objects that we will use """

        # Randomise the positions of our buttons and change their pos and store it
        #TODO: replace this with a fitting algorithm URGENT
        randomised_segments = segments # Make a copy, cause random shuffle doesnt have a return
        random.shuffle(randomised_segments)
        i = 0
        offset = 0
        for segment in randomised_segments:
            pos = [(self.segment_x_y[i][0] + offset), self.segment_x_y[i][1]]
            segment.set_pos(pos)
            segment.return_info["init_pos"] = pos
            i += 1
            # workaround / temp fix for having too many seg slots
            if i >= len(self.segment_x_y):
                i = 0 # Loop from first pos again
                offset += 50

        # Create loading button
        loading_button = self.create_button("loading_screen_button_depressed.png",
                                           (270, 550), scale=2.3, should_grey= False)

        # Create unknown button slots and have them scale according to how many there is
        unknown_y = 150
        unknown_slots = []
        # Create the unknown slots we need
        for i in range(num_correct_slots):
            unknown_seg = self.create_button("music_segment_greyed_out.png", (
                unknown_y, 0), scale = 2)

            unknown_slots.append(unknown_seg)
        # Change position of unknown slots
        step = (unknown_slots[0].get_rect()[2])  # step = width of slots
        starting_x = self.window_center[0] - (step * (len(unknown_slots) / 2) ) # the x of the left-most slot
        i = 0
        for slot in unknown_slots:
            unknown_x = starting_x + step*i
            slot.set_pos((unknown_x, unknown_y))
            slot.set_info( {"pos":i, "slot_full":False} ) # Store the order of the unknown segments in the button
            i += 1

        # Create text objects
        top_text = self.create_text(self.window, self.window_center, "Put the song back together", cen_x=True,
                                    font_size=90)
        middle_text = self.create_text(self.window, self.window_center, "Drag the pieces into the slots",
                                       location=[0, 680], cen_x=True, font_size=90)
        text_objs = [top_text, middle_text]

        # Create help button
        help_button = self.create_button("help_button.png", (2200, 100), scale=1,
                                            unique_id="help")
        if single_out:
            return [loading_button] +  unknown_slots + text_objs + [help_button] + randomised_segments
        else:
            return loading_button, unknown_slots,  text_objs, help_button, randomised_segments

    def play_seg_track(self, song_path):
        """Uses the stored info attribute to play music on button press """
        # Dont remember why this function is nested, but i remember that it has to be to work.
        def play_track():
            self.sound_manager.start_track(song_path)

        return play_track

    def stop_seg_track(self, song_path):
        """Uses the stored info attribute to play music on button press """

        def stop_track():
            self.sound_manager.stop_track()

        return stop_track

    def get_fps(self):
        """get time between the last execution of this function and the current """
        t2 = rospy.get_time()
        fps = "FPS: " + str(int(1 / (t2 - self.t1)))  # calculate fps  1 / (timenow - prevtime)
        self.t1 = t2  # store prev time
        return fps

    def draw_debug_info(self, fps):
        """Draw any debug features we want to see """
        if self.debug:
            self.renderer.DrawText(fps, (70, 50), font_size=30)  # draw fps

    def render_target_graphics(self, window, graphics, key):
        """draw graphics if they match the key"""
        #render graphics 1 by 1, only if they match the key
        for i in range(len(graphics)):
            if (i+1) in key:
                graphics[i].render(window)
        # print(i)

    def draw_tut_options(self, arrow_rect):
        """ Using arrow rect, decide where tut buttons should be drawn"""
        arrow_x = arrow_rect[0]
        arrow_y = arrow_rect[1]
        # if arrow is too close to right edge of screen
        if (arrow_x + 800) > self.window_x:
            self.tut_next.set_pos((arrow_x - 600, arrow_y))
            self.tut_repeat.set_pos((arrow_x - 600, arrow_y + 250))
        # if arrow is too close to left edge of screen
        elif (arrow_x - 800) < 0:
            self.tut_next.set_pos((arrow_x + 400, arrow_y))
            self.tut_repeat.set_pos((arrow_x + 400, arrow_y + 250))
        else:
            self.tut_next.set_pos((arrow_x + 450, arrow_y))
            self.tut_repeat.set_pos((arrow_x - 600, arrow_y))

    def create_check(self, seg):
        """Draws a check mark under the segment given"""
        mark = self.create_button("check_mark.png",
                           (seg.rect.centerx - 35, seg.rect.centery + 155),
                           scale=0.5)
        return mark

    def qt_reward(self, time_taken, wrong_answers, hints_needed ):
        """handles what QT should say and do on level end """
        #"Perfect clear"
        if time_taken < 15 and wrong_answers < 1 and hints_needed < 1:
            self.command_manager.send_qt_command(emote = "happy", gesture = "nod")
            qt_message = (self.behaviours_manager.get_praise())     #QT reads out level's hint
        else:
            self.command_manager.send_qt_command(emote = "happy", gesture = "nod")
            qt_message = (self.behaviours_manager.get_agreements()) #QT reads out level's hint
        return qt_message

    #######################################################Level / screen code###############################################################

    def guided_tut(self, correct_song, distract_song, slices):
        """Code to play tut sequence for fix the song"""

        # String of our keys so i can remember them
        """
        1 = basket
        2 - 3 = song slots
        4 = Top text
        5 = Middle text
        6 = help button
        7 - 8 = song segs
        """

        tut_graphics = {
            1: {"rect": None, "keys": [1, 2, 3, 4, 5, 6, 7, 8],
                "speech": "To play this game, first you will hear music... I will let you listen to it in full..."
                " After that I will split the song into parts... Then you will need to find the correct "
                "pieces and put them back together..."},
            2: {"rect": (800, 0, 1235, 100), "keys": [4],
                "speech": "This is a reminder of what you have to do."},
            3: {"rect": (1075, 125, 700, 350), "keys": [2, 3],
                "speech": "These are the slots where the pieces of the songs need to go..."
                          " Find the correct segments and drag them here... In the right order to"
                          " complete the level...."},
            4: {"rect": (770, 670, 1310, 110), "keys": [5],
                "speech": "This is another reminder of what you have to do."},
            5: {"rect": (550, 830, 1050, 350), "keys": [7, 8],
                "speech": "These are the segments... Click on them once to listen to them. I might put some in there "
                          "to confuse you... so make sure to listen to them all!"},
            6: {"rect": (2150, 50, 500, 475), "keys": [6], "speech": "This is the help button... Click this if you dont"
                                                                      "know which song piece is the right one."},
            7: {"rect": None, "keys": [1, 2, 3, 4, 5, 6, 7, 8], "speech": "That is everything for Fix the song! Have fun!"}
        }

        # Get the level's data
        self.track_name = correct_song
        self.distract_song = distract_song
        self.segment_num = slices

        # Create graphics and buttons
        segments, num_correct_segs = self.create_segments(self.segment_num, self.track_name, self.distract_song)
        graphics = self.create_graphics(segments, num_correct_segs, single_out=True)
        self.tut_next = self.create_button("tut_next.png", (0, 0), scale=1.5, unique_id="next", should_grey = False)
        self.tut_repeat = self.create_button("tut_repeat.png", (0, 0), scale=1.5, unique_id="repeat", should_grey = False)

        # Group graphics for easier processing
        tut_buttons = [self.tut_next, self.tut_repeat]

        if self.run:

            key = 1
            while key <= len(tut_graphics.keys()) and not rospy.is_shutdown() and self.run:
                #Get tut info for this section
                tut_key = tut_graphics[key]["keys"]  # Draw grey graphics of everything except for our focused graphic
                tut_speech = tut_graphics[key]["speech"]
                tut_rect = tut_graphics[key]["rect"]

                #reset vars
                qt_finished_talking = False  # Hold execution until user clicks somewhere
                speaking_timer = self.command_manager.qt_say(tut_speech)  # QT should say text out loud
                option_chosen = False # If user clicks next or repeat
                repeat_instruction = False
                prev_key = key - 1

                while not option_chosen and not rospy.is_shutdown() and self.run:

                    # Render graphics
                    self.renderer.DrawBackground(self.background_colour)
                    self.render_target_graphics(self.window,graphics,tut_key)
                    self.animation_manager.DrawTouchAnimation(self.window)
                    #render the arrow and get it's pos
                    if tut_rect != None:
                        arrow_rect = self.renderer.HighlightRect(tut_rect, self.pygame) #draw arrow and box
                        #since the arrow keeps moving, take it's location once for the next and repeat buttons.
                        if key != prev_key:
                            self.draw_tut_options(arrow_rect)
                            prev_key = key
                    if qt_finished_talking and tut_rect != None:
                        for button in tut_buttons:
                            button.render(self.window)
                    # if there's no box highlighted, draw the buttons in a default pos
                    elif qt_finished_talking and tut_rect == None:
                        self.tut_next.set_pos((1540, 1200))
                        self.tut_repeat.set_pos((840, 1200))
                        for button in tut_buttons:
                            button.render(self.window)

                    # Handle events
                    events = self.pygame.event.get()
                    for event in events:
                        mouse_pos = self.pygame.mouse.get_pos()
                        # print(mouse_pos) #TEMP
                        if event.type == self.pygame.QUIT:
                            self.run = False  # Stops the program entirely
                            self.quit = True  # Tells us that the game was quit out of, and it didn't end organically
                        if event.type == self.pygame.MOUSEBUTTONUP:  # On mouse release play animation to show where cursor is
                            self.animation_manager.StartTouchAnimation(mouse_pos)  # Play animation
                        # Keyboard override for testing NOTE: this wont work unless you comment out the timer check
                        if event.type == pygame.KEYDOWN :
                            if event.key == pygame.K_RIGHT:
                                qt_finished_talking = True
                            elif event.key == pygame.K_LEFT:
                                qt_finished_talking = False
                        #Detect button presses from buttons only after qt finishes speaking
                        if qt_finished_talking:
                            for button in tut_buttons:
                                button_pressed = button.get_event(event, mouse_pos)
                                if button_pressed:
                                    option_chosen = True #so we can choose when this goes to false
                                if button_pressed:
                                    button_pressed_id = button.id #get which button was pressed
                                    if button_pressed_id == "repeat":
                                        repeat_instruction = True

                    # Dont check if QT already finished talking
                    if not qt_finished_talking:
                        qt_finished_talking = self.command_manager.robo_timer.CheckTimer(speaking_timer)
                    self.pygame.display.update()  # Update all drawn objects

                #If they wanted to repeat it, run the same loop again, otherwise move on
                if not repeat_instruction:
                    key+=1
            return self.run


    def play_music_blocking(self, song_name):
        """Level with just music player"""
        if self.run:

            # Get the level's data
            self.track_name = song_name

            # Create buttons
            self.next_button = self.create_button("next_button.png", (self.cen_x - 300, 1200),
                                                 scale=1)
            self.play_button = self.create_play_button("pause_button.png", "play_button.png",
                                                     "rewind_button.png", (self.cen_x - 175, 550), scale=1.5,
                                                       on_play=self.sound_manager.unpause,
                                                       on_pause=self.sound_manager.pause)  # create pause and play button
            # Load track
            self.sound_manager.load_track(self.track_name)
            # VariablesFte
            music_playing = True
            self.song_ended = False
            self.current_track_time = 0
            self.track_total_time = 100
            self.progress = 0
            self.current_track_time, self.track_total_time, self.progress, self.song_ended = self.get_song_info()

            self.sound_manager.unpause()
            music_ended = False
            next_pressed = False

            while not next_pressed and not rospy.is_shutdown() and self.run:
                time = rospy.get_time()

                # get song data
                self.current_track_time, self.track_total_time, self.progress, self.song_ended = self.get_song_info()

                if self.song_ended:
                    self.sound_manager.load_track(self.track_name)  # reload the track at the start
                    music_ended = True  # use this var, so we have one that stays true for the rest of the loop
                    self.play_button.its_rewind_time()  # draw rewind symbol instead of play symbol

                # Draw background and objects
                self.renderer.DrawBackground(self.background_colour)
                self.renderer.DrawTextCentered("Please listen to the song.", font_size=100, y=300)
                if music_ended:  # render next button grey until music ended
                    self.next_button.render(self.window, grey=False)
                else:
                    self.next_button.render(self.window, grey=True)
                if music_ended:
                    self.play_button.render(self.window)
                else:
                    self.play_button.render(self.window, grey=True)
                self.animation_manager.DrawTouchAnimation(self.window)  # also draw touches
                self.pygame.display.update()  # Update all drawn objects

                # Check if the X was clicked
                for event in self.pygame.event.get():
                    if event.type == self.pygame.QUIT:
                        self.run = False  # Stops the program entirely
                        self.sound_manager.stop_track()
                    mouse_pos = self.pygame.mouse.get_pos()
                    if event.type == self.pygame.MOUSEBUTTONDOWN:
                        self.animation_manager.StartTouchAnimation(mouse_pos)  # draw mouse click animation
                    # Check for button press
                    if music_ended:  # if music ended start checking for next press, otherwise ignore it
                        next_button_clicked = self.next_button.get_event(event, mouse_pos)
                        if next_button_clicked:
                            next_pressed = True
                            self.sound_manager.stop_track()
                        self.play_button.get_event(event, mouse_pos)  # only check for button press on music end
                # print(rospy.get_time() - time)
            return self.run

    def compute_clear_type(self, time_taken, wrong_answers, hints_needed):
        e_time = self.track_total_time * self.e_clear_multiplier  # Total track time * 1.5
        if time_taken < e_time and wrong_answers < 1 and hints_needed < 1:
            return "e_clear"
        elif wrong_answers >= 2 or hints_needed >= 2:
            return "fail"
        else:
            return "clear"


    def play_level(self, correct_song, distract_song, slices):
        """Sequence plays the levels"""
        if self.run:  # Dont start this screen if the previous screen wanted to close out the game

            # Get the level's data
            self.track_name = correct_song
            self.distract_song = distract_song
            self.segment_num = slices
            fps = "0"  # for debug info

            # Create graphics and buttons
            segments, num_correct_segs = self.create_segments(self.segment_num, self.track_name, self.distract_song)
            loading_button, unknown_slots, text_objs, help_button, randomised_segments = self.create_graphics(
                segments, num_correct_segs, single_out=False)

            # Group graphics for easier rendering
            graphics = [loading_button] + unknown_slots + text_objs + [help_button] + randomised_segments
            check_marks = [] # We will populate this with check marks later

            reset_segs  = False # If we have just pressed the segment
            currently_playing = "" # The ID of the segment playing music
            song_restored = False # the song is fixed, so do ending sequence
            level_completed = False # End the gameplay loop
            music_time_id = ""
            blocking = False
            start_time = rospy.get_time()
            wrong_answers = 0
            hints_needed = 0
            while not level_completed and not rospy.is_shutdown() and self.run:
                #Combine the two render lists into one so when we block the screen we can send this alone
                combined_graphics = graphics + check_marks

                # End loop check at top, to let loop finish 1 last time
                if song_restored:
                    level_completed = True

                # Render graphics
                self.renderer.DrawBackground(self.background_colour)
                for graphic in graphics:
                    if blocking:
                        graphic.render(self.window, grey=True )
                    else:
                        graphic.render(self.window, grey=False)
                if check_marks != []:
                    for check_mark in check_marks:
                        if blocking:
                            check_mark.render(self.window, grey=True)
                        else:
                            check_mark.render(self.window, grey=False)

                self.animation_manager.DrawTouchAnimation(self.window)
                self.pygame.display.update()  # Update all drawn objects

                # Handle events
                events = self.pygame.event.get()
                for event in events:
                    mouse_pos = self.pygame.mouse.get_pos()
                    # print(mouse_pos) #TEMP
                    if event.type == self.pygame.QUIT:
                        self.run = False  # Stops the program entirely
                        self.quit = True  # Tells us that the game was quit out of, and it didn't end organically
                    # On mouse release play animation to show where cursor is
                    if event.type == self.pygame.MOUSEBUTTONUP:
                        self.animation_manager.StartTouchAnimation(mouse_pos)  # Play animation
                        reset_segs = True
                    # Dont do any event handling if we're blocking currently
                    if not blocking:
                        # Handle help button events
                        help_clicked = help_button.get_event(event, mouse_pos)
                        if help_clicked:
                            hints_needed += 1
                            #Find the next segment and put it in the right slot
                            slot_num = 999
                            for segment in segments:
                                # Skip distract songs. Not an and so we dont try to iterate with none
                                if segment.return_info["song_pos"] != None:
                                    if segment.return_info["song_pos"] < slot_num and not segment.return_info["correct_slot"] == True:
                                        slot_num = segment.return_info["song_pos"]
                                        given_seg = segment
                            # Get pos of correct slot
                            for slot in unknown_slots:
                                if slot.return_info["pos"] == slot_num:
                                    slot_pos = slot.get_pos()
                                    break
                            # Not efficient, but loop through all segs again, to see if any are in the slot.
                            for segment in segments:
                                if slot.rect.collidepoint(segment.rect.center):
                                    segment.set_pos(segment.return_info["init_pos"])
                            # Move given seg to the slot it's meant to be in
                            given_seg.set_pos(slot_pos)
                            check = self.create_check(given_seg)
                            check_marks.append(check)
                            given_seg.return_info["correct_slot"] = True
                            given_seg.disable_drag = True
                            qt_message = "I will give you one of the pieces of the song..."
                            self.level_loader.QTSpeakingPopupScreen(qt_message, combined_graphics, self.run,
                                                                    self.background_colour, partial_func=False)  # this is blocking
                        #Handle segment events
                        for segment in randomised_segments:
                            segment.get_event(event, mouse_pos)
                            # Handle segment events on mouse release
                            if event.type == self.pygame.MOUSEBUTTONUP:
                                if mouse_pos == segment.initial_mouse_pos:
                                    currently_playing = segment.id
                                    reset_segs = True
                                    # Start a timer telling us when the music should end
                                    song_time = (self.sound_manager.return_wav_lenth(segment.return_info["song_path"])) # get song len
                                    music_time_id = self.timer_manager.CreateTimer("music_finished",song_time,verbose=False)
                                # Snapback or new pos for seg
                                else:
                                    # If we hover over a slot, while holding a segment
                                    segment_init_pos = segment.return_info["init_pos"]
                                    hover = False
                                    for slot in unknown_slots:
                                        slot_pos = slot.get_pos()
                                        if slot.rect.collidepoint(segment.rect.center): #and slot.return_info["slot_full"] == False:
                                            # slot.return_info["slot_full"] = True
                                            segment.set_pos(slot_pos)
                                            hover = True
                                            if segment.return_info["song_pos"] == slot.return_info["pos"]:
                                                # If this is the first time we checked if this slot was correct
                                                if segment.return_info["correct_slot"] == False:
                                                    check = self.create_check(segment)
                                                    check_marks.append(check)
                                                    qt_message = self.behaviours_manager.get_praise()
                                                    self.command_manager.send_qt_command(speech = qt_message,gesture="nod", emote="talking")
                                                segment.return_info["correct_slot"] = True
                                                segment.disable_drag = True
                                            else:
                                                qt_message = self.behaviours_manager.get_disagreements()
                                                self.command_manager.send_qt_command(gesture="shake_head",
                                                                                     emote="talking")
                                                self.level_loader.QTSpeakingPopupScreen(qt_message, combined_graphics, self.run,
                                                                                        self.background_colour,
                                                                                        partial_func=False)
                                                segment.set_pos(segment_init_pos)
                                                wrong_answers += 1

                                    # If we're not above a slot when mouse button up, reset seg pos. Ignore segs that are in right slot
                                    if not hover and not segment.return_info["correct_slot"]:
                                        segment.set_pos(segment_init_pos)  # snap back
                    #Only reset once per mouse click
                    if reset_segs:
                        for segment in randomised_segments:
                            # If a segment that isn't playing is showing pause sign, set it to play sign.
                            if segment.id != currently_playing and segment.toggle:
                                segment.toggle = False
                        reset_segs = False
                #Check if song_playing stopped and seg still showing play
                if len(self.timer_manager.timers) > 0:
                    if self.timer_manager.CheckTimer(music_time_id):
                        # Set all toggles to false
                        for segment in randomised_segments:
                            if segment.toggle:
                                segment.toggle = False
                # Check if level completed
                i = 0
                for seg in randomised_segments:
                    if seg.return_info["correct_slot"]:
                        i+=1
                if i >= num_correct_segs:
                    song_restored = True

            time_taken = rospy.get_time() - start_time
            qt_message = self.qt_reward(time_taken, wrong_answers, hints_needed)
            self.level_loader.QTSpeakingPopupScreen(qt_message, combined_graphics, self.run,
                                                    self.background_colour, partial_func =  False)
            self.command_manager.send_qt_command(gesture="clap",
                                                 emote="talking")
            self.empty_temp_dir(randomised_segments)

            clear_type = self.compute_clear_type(time_taken, wrong_answers, hints_needed)
            e_clear_req = {"time_taken": self.track_total_time * self.e_clear_multiplier, "mistakes":0, "num_hints":0}
            performance = {"time_taken": time_taken, "mistakes": wrong_answers, "num_hints": hints_needed}
            level_data = {"game_name": "FTS", "clear_type": clear_type, "performance": performance,
                          "e_clear_req": e_clear_req}
            return self.run , level_data


    #################################################################Main####################################################################

    def Main(self, correct_song, distract_song, slices, ask_tut = True):
        # input what level and difficulty to play, the program will handle the rest
        """Main Func"""

        # Introduce game
        self.run = self.level_loader.QTSpeakingScreen("Lets play Fix The Song!", self.run, self.background_colour)

        # Ask if they want to play tutorial
        if ask_tut:
            self.run, tut = self.level_loader.yes_or_no_screen('Should I explain how to play "Fix The Song" ?', self.run, self.background_colour)
            if tut:
                self.guided_tut(correct_song, distract_song, slices)
        
        # Count in to the start of the game
        self.run = self.level_loader.tap_to_continue(self.run, self.background_colour)
        
        # Play the track and block
        #self.play_music_blocking(correct_song)
        self.track_total_time = 15

        # Count into level to slow pacing
        self.run = self.level_loader.countdown(3, self.run, self.background_colour,
                                               prelim_msg="Lets Fix The Song!")

        # Play main level
        self.run, level_data = self.play_level(correct_song, distract_song, slices)

        # Save user data
        return self.run, level_data

######################################################On execution#######################################################################

# If we run this node, run the game on it's own
if __name__ == '__main__':
    # Initialise game
    rospy.init_node('fix_song_game', anonymous=False)
    rospy.loginfo("Node launched successfully")
    game_object = Fix_The_Song_Game("jwaad")
    # Run the game
    try:
        game_object.Main()
    except(KeyboardInterrupt or rospy.exceptions.ROSInterruptException):
        game_object.pygame.quit
        SoundManager("").stop_track()
        print("Audio may not be stopped due to interrupt")

    # on exit delete music files and stop music
    SoundManager("").stop_track()
