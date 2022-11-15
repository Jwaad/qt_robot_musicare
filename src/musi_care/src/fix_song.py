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


#################################################################Initialise#################################################################

class Fix_The_Song_Game():
    """ Class to generate and handle guess the mood game """

    def __init__(self):
        """Initialise"""
        x = 145  # x pos of screen
        y = 0  # y pos of screen
        os.environ['SDL_VIDEO_WINDOW_POS'] = '%d,%d' % (x, y)  # move screen to x and y pos
        self.previous_screen = ""  # used so we can go backwards a screen
        self.next_screen = ""  # used to skip screen, low priority feature
        self.pygame = pygame
        self.pygame.init()  # start py engine
        self.pygame.freetype.init()
        res = pygame.display.Info()  # get our screen resolution
        self.window_x = res.current_w - 150  # Width of window -150 to account for the linux toolbar
        self.window_y = res.current_h  # Height of window
        self.window_center = (int(self.window_x / 2), int(self.window_y / 2))
        self.cen_x = self.window_center[0]
        self.cen_y = self.window_center[1]
        self.window = pygame.display.set_mode((self.window_x, self.window_y))  # Create window and set size
        self.background_colour = (100, 100, 100)  # background black by default
        self.pygame.display.set_caption("Fix The Song!")  # Label window
        self.run = True
        # self.pygame.mouse.set_visible(False) #set to false when not testing
        self.quit = False  # Check to see if the game ended or it was quit
        self.track_playing = False
        self.previous_track_data = None
        self.difficulty = "easy"  # Default difficulty to play
        self.current_level = 1  # Default level to play
        self.music_data = self.get_song_database()  # From save file load all of the level data
        # print(self.music_data)
        self.music_filepath = "/game_assets/music/"  # relative path to songs # "/home/qtrobot/catkin_ws/src/musi_care/src/game_assets/music/"
        self.timer_manager = TimeFunctions()
        self.animation_manager = AnimationManager(self.pygame)
        self.sound_manager = SoundManager(self.music_filepath)  # load soundplayer with sound file path
        self.command_manager = QTManager()
        self.renderer = Renderer(self.window, self.window_center)
        self.level_loader = StandardLevels(self.window, self.window_center, self.pygame, self.music_filepath)
        self.segment_x_y = {0: (600, 800), 1: (1250, 800), 2: (1900, 800), 3: (600, 1500), 4: (1250, 1500),
                            5: (1900, 1500)}  # hard coded num locations of each segment
        self.sayings = Behaviours(self.pygame, self.music_filepath)
        self.t1 = 0  # t1 for FPS tracking
        self.debug = True
        # self.music_vol = 1 # change volume of laptop
        # self.qt_voice_vol
        # self.sound_manager.volume_change(self.music_vol) # Set a default volume
        # self.set_robot_volume(qt_voice_vol) #TODO add this functionality

    ########################################################Low level methods################################################################

    def get_song_database(self):
        """Read the database file and get the levels data"""

        # data_filepath = ("/home/qtrobot/catkin_ws/src/musi_care/src/game_assets/music/music_data.txt")
        data_filepath = ("/game_assets/data/fsg_level_data.txt")  # gtm = guess the mood
        this_file_path = os.path.dirname(__file__)
        full_path = this_file_path + data_filepath
        music_data = []

        with open(full_path, "r") as database:
            difficulty_labels = ["tut", "easy", "medium", "hard"]
            music_data = {"tut": {1: {""}}, "easy": {1: {""}}, "medium": {1: {""}},
                          "hard": {1: {""}}}  # Reset data to overwrite it thouroughly

            # sort data into their difficulty tiers
            data = database.read().splitlines()  # read data and load into raw into "data"
            for difficulty in difficulty_labels:  # extract for each difficulty seperately
                open_bracket_line_num = 0
                open_bracket_found = False
                close_bracket_line_num = 0
                close_bracket_found = False
                line_num = 0

                # Look for brackets and get the data between them
                for line in data:
                    if line == "{":
                        open_bracket_line_num = line_num
                        open_bracket_found = True
                    elif line == "}":
                        close_bracket_line_num = line_num
                        close_bracket_found = True
                    # We have found the brackets, save the information and delete it from data
                    if open_bracket_found and close_bracket_found:
                        # Organise data and put it into a new dict
                        difficulty_data = data[open_bracket_line_num + 1:close_bracket_line_num]
                        new_song = False
                        level = 1
                        music_data[difficulty][level] = dict()  # init 1st level
                        for attribute in difficulty_data:
                            if attribute == "£#":
                                new_song = True
                            if not new_song:  # if all the atrributes describe the same song, add them to the same dict
                                split_attribute = attribute.split("=")  # split line by the =
                                attribute_label = split_attribute[0].replace(" ", "")  # get rid of any spaces now
                                if attribute_label == "distract_song":  # parse differently
                                    if split_attribute[1].replace(" ",
                                                                  "") == "none":  # if there's no distract songs, just return none
                                        attribute_value = None
                                    else:
                                        split_attribute.pop(0)
                                        split_attribute  # get rid of the label, the rest are songs
                                        songs = split_attribute[0].split(",")
                                        attribute_value = []
                                        for song in songs:
                                            distract_song = song.replace(" ", "")  # remove spaces
                                            attribute_value.append(distract_song)
                                else:
                                    attribute_value = split_attribute[1].replace(" ",
                                                                                 "")  # Get rid of the space at the start
                                music_data[difficulty][level][attribute_label] = attribute_value
                            else:
                                new_song = False
                                level += 1
                                music_data[difficulty][
                                    level] = dict()  # Create new song entry labeled as the correct level
                        data = data[close_bracket_line_num + 1:]
                        break
                    line_num += 1
            return music_data

    def empty_temp_dir(self):
        """empties temp folder after use, NOT IN USE BECAUSE IM SCARED OF DELETING THE WRONG FILES"""
        path = os.path.dirname(__file__) + self.music_filepath + "temp"
        song_names = os.listdir(path)
        # add part that deletes files here

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

    def get_song_info(self, prev_track_time="", prev_total_time=""):
        """Get variables that we will draw onto screen"""
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

    def create_button(self, file_name, alt_file_name, location,return_info = {}, scale=1, unique_id=""):
        """code creates button using the button_image class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/graphics'
        file_path = os.path.join(this_file_path, relative_path, file_name)
        alt_path = os.path.join(this_file_path, relative_path, alt_file_name)

        button = Button(file_path, alt_path, location, self.pygame, return_info = {}, scale=scale, unique_id=unique_id)
        return (button)

    def create_toggle_button(self, file_name, alt_file_name, default_image_grey, toggled_image_grey, location, scale=2,
                             unique_id="", return_info="", when_toggle_on=object, when_toggle_off=object):
        """code creates button using the button_image class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = '/game_assets/graphics/'
        file_path = this_file_path + relative_path + file_name
        alt_path = this_file_path + relative_path + alt_file_name
        file_path_grey = this_file_path + relative_path + default_image_grey
        alt_path_grey = this_file_path + relative_path + toggled_image_grey

        button = ToggleButton(file_path, alt_path, file_path_grey, alt_path_grey, location, self.pygame, scale,
                              unique_id, return_info, when_toggle_on, when_toggle_off)
        return (button)

        # default_image_path, toggled_image_path, default_image_grey, toggled_image_grey, x_y_locations, pygame, scale=1, unique_id = "", return_info="", when_toggle_on=object, when_toggle_off=object

    def create_drag_button(self, file_name, alt_file_name, file_grey, alt_file_grey, location, scale=2, return_info={},
                           when_toggle_on=object, when_toggle_off=object):
        """code creates button using the button_image class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/graphics'
        file_path = os.path.join(this_file_path, relative_path, file_name)
        alt_path = os.path.join(this_file_path, relative_path, alt_file_name)
        file_path_grey = os.path.join(this_file_path, relative_path, file_grey)
        alt_path_grey = os.path.join(this_file_path, relative_path, alt_file_grey)

        button = DraggableButton(file_path, alt_path, file_path_grey, alt_path_grey, location, self.pygame, scale,
                                 return_info=return_info, when_toggle_on=when_toggle_on,
                                 when_toggle_off=when_toggle_off)
        return (button)

    def create_play_button(self, file_name, alt_file_name, file_grey, alt_file_grey, rewind_name, rewind_name_grey,
                           location, scale=1, unique_id="", on_pause=object, on_play=object):
        """code creates toggle button using the toggle button class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/graphics'
        file_path = os.path.join(this_file_path, relative_path, file_name)
        alt_path = os.path.join(this_file_path, relative_path, alt_file_name)
        file_path_grey = os.path.join(this_file_path, relative_path, file_grey)
        alt_path_grey = os.path.join(this_file_path, relative_path, alt_file_grey)
        file_path = os.path.join(this_file_path, relative_path, file_name)
        alt_path = os.path.join(this_file_path, relative_path, alt_file_name)
        rewind_path = os.path.join(this_file_path, relative_path, rewind_name)
        rewind_path_grey = os.path.join(this_file_path, relative_path, rewind_name_grey)

        button = PausePlayButton(file_path, alt_path, file_path_grey, alt_path_grey, rewind_path, rewind_path_grey,
                                 location, self.pygame, scale, unique_id, on_pause, on_play)
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

    def update_grey_graphics(self, given_half, draggable_buttons, main_buttons):
        """reinitialise grey graphics taking with parameters they need """
        # Load graphics into dict # need to be in a specific order for this tut
        grey_graphics = {}
        grey_graphics[1] = functools.partial(self.renderer.DrawTextCentered, "Put the song back together",
                                             font_size=100, y=50)
        grey_graphics[2] = functools.partial(given_half.render, self.window, grey=True)
        grey_graphics[3] = functools.partial(self.renderer.DrawTextCentered,
                                             "Drag the segments below into the empty slot above", font_size=100, y=550)
        i = 4
        for button in main_buttons:
            grey_graphics[i] = functools.partial(button.render, self.window, grey=True)
            i += 1
        for key in draggable_buttons:  # render draggable buttons
            grey_graphics[i] = functools.partial(draggable_buttons[key].render, self.window,
                                                 grey=True)  # draw the segments
            i += 1
        return grey_graphics
        # self.animation_manager.DrawTouchAnimation(self.window) #last so it shows up on top

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

    def create_segments(self,segment_num, correct_track, distract_track):
        """Create and return a list of segments (draggable buttons) """
        # grey = 0 blue = 1 orange = 2 purple = 3.  #0, is for grey screen
        self.segment_graphics = {
            0: ("music_segment_play_grey.png", "music_segment_pause_grey.png"),
            1: ("music_segment_play_blue.png", "music_segment_pause_blue.png"),
            2: ("music_segment_play_orange.png", "music_segment_pause_orange.png"),
            3: ("music_segment_play_purple.png", "music_segment_pause_purple.png"),
            4: ("music_segment_play_green.png", "music_segment_pause_green.png"),
            5: ("music_segment_play_pink.png", "music_segment_pause_pink.png")
        }

        # Get each segment position
        correct_segments, distract_segments = self.sound_manager.slice_song(segment_num, correct_track, distract_track)
        all_segs = []

        # Make objects of correct segments
        i = 1  # pos in correct segs
        prev_colour = 0
        for song_path in correct_segments:
            colour_same = True
            # Keep shuffling until we get a colour different from the last.
            while colour_same:
                seg_colour = self.segment_graphics[random.randint(1, 5)]  # Ignore grey
                if seg_colour != prev_colour:
                    colour_same = False
                print(seg_colour)
            prev_colour = seg_colour

            button_grey = self.segment_graphics[0][0]
            seg_pos = [0, 0]  # Default pos, this will be changed later
            seg_data = {
                "song_path": song_path,
                "correct_seg": True,
                "order": i
            }
            segment = self.create_drag_button(seg_colour[0], seg_colour[1], button_grey, button_grey, seg_pos,
                                              return_info=seg_data,
                                              when_toggle_on=self.play_seg_track(song_path),
                                              when_toggle_off=self.stop_seg_track(song_path))
            all_segs.append(segment)
            i += 1

        # Make objects of correct segments
        for song_path in distract_segments:
            seg_colour = self.segment_graphics[random.randint(1, 3)]  # Ignore grey
            button_grey = self.segment_graphics[0][0]
            seg_pos = [0, 0]  # default pos, this will be changed later
            seg_data = {
                "song_path": song_path,
                "correct_seg": False,
                "order": None
            }
            segment = self.create_drag_button(seg_colour[0], seg_colour[1], button_grey, button_grey, seg_pos,
                                              return_info=seg_data,
                                              when_toggle_on=self.play_seg_track(song_path),
                                              when_toggle_off=self.stop_seg_track(song_path))
            all_segs.append(segment)

        return all_segs, len(correct_segments)


    def create_graphics(self, segments, num_correct_slots):
        """Create the pygame objects that we will use """

        # Randomise the positions of our buttons and change their pos
        randomised_segments = segments
        random.shuffle(randomised_segments)
        for seg_i in range(len(randomised_segments)):
            randomised_segments[seg_i].set_pos(self.segment_x_y[seg_i])

        # Create loading button
        loading_button = self.create_button("loading_screen_button_depressed.png", "loading_screen_button_depressed.png",
                                           (270, 550), scale=2.3)
        # Create unknown button slots and have them scale according to how many there is
        unknown_y = 200
        unknown_slots = []
        # Create the segments we need
        for i in range(num_correct_slots):
            unknown_seg = self.create_button("music_segment_greyed_out.png", "music_segment_greyed_out.png", (
                unknown_y, 0), scale = 2)
            unknown_slots.append(unknown_seg)

        # Change position of unknown slots
        step = (unknown_slots[0].get_rect()[2])  # step = width of slots
        starting_x = self.window_center[0] - (step * (len(unknown_slots) / 2) ) # the x of the left-most slot
        i = 0
        for seg in unknown_slots:
            unknown_x = starting_x + step*i
            seg.set_pos((unknown_x, unknown_y))
            seg.set_info( {"pos":i} ) # Store the order of the unknown segments in the button
            i += 1

        return randomised_segments, loading_button, unknown_slots

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

    def get_target_behaviour(self, key, given_half, draggable_buttons, main_buttons):
        """tells our tut what to draw and what to do with events"""
        target_graphics = []
        target_event_handler = None
        if key == 2:  # given seg
            target_graphics = [functools.partial(given_half.render, self.window)]  # given_half.render(self.window)
        elif key == 3:  # segs
            target_graphics = []
            for key in draggable_buttons:
                graphic = functools.partial(draggable_buttons[key].render, self.window, )
                target_graphics.append(graphic)
        elif key == 4:  # song slot
            target_graphics = [functools.partial(main_buttons[1].render, self.window)]

        return target_graphics, target_event_handler  # if our logic sifts failed

    #######################################################Level / screen code###############################################################

    def guided_tut(self):
        """Code to play tut sequence for fix the song"""

        # String of our keys so i can remember them
        """
        1 = top text 
        2 = given seg
        3 = instructions
        4 = the box behind segs
        5 = Slots for segs to go in
        6 & 7 = segs
        """

        # Create rect to highlight and text for QT to say
        # (250, 400, 2600-250, 650-400) #slider rect
        # TODO ADD MORE TO THIS -->
        # Lets try it now: listen to song --> this sounds happy to me. --> lets click "happy" --> highlight happy --> wait for press
        tut_graphics = {
            1: {"rect": None, "keys": [1, 2, 3, 4, 5, 6, 7],
                "speech": "To play this game, first you will hear music... I will let you listen to it in full... After that I will split the song into 2 parts... Then you will need to find the correct pieces and put them back together..."},
            # given rect
            2: {"rect": (1100, 100, 300, 350), "keys": [1, 3, 4, 5, 6, 7],
                "speech": "I have already given you the first half of the song... Click on it to play it and remind yourself of what the song sounds like."},
            # segs
            3: {"rect": (550, 750, 1050, 450), "keys": [1, 2, 3, 4, 5],
                "speech": "These are the segments... Click on them once to listen to them. Only 1 of these is the correct answer, so listen to them all to figure out which is correct!"},
            # Song slot
            4: {"rect": (1350, 100, 400, 350), "keys": [1, 2, 3, 4, 5, 6, 7],
                "speech": "This box is called the song slot... Once you have found the correct segment, drag it into this box to complete the level..."},
            5: {"rect": None, "keys": [1, 2, 3, 4, 5, 6, 7], "speech": "That is everything for Fix the song! Have fun!"}
        }

        # Get the level's data
        level_data = self.music_data["tut"][1]  # load tut song data
        self.track_name = level_data["song_name"]
        self.distract_song = level_data["distract_song"]  # will be None or a list of songs
        self.segment_num = int(level_data["seg_num"])  # split song into this many segs

        # Create graphics and buttons
        segments, num_correct_segs = self.create_segments(self.segment_num, self.track_name, self.distract_song)
        randomised_segments, loading_button, unknown_slots = self.create_graphics(segments, num_correct_segs)

        while self.run:

            # Render graphics
            self.renderer.DrawBackground(self.background_colour)
            loading_button.render(self.window, grey=False)
            for segment in randomised_segments:
                segment.render(self.window, grey=False)
            for slot in unknown_slots:
                slot.render(self.window, grey=False)
            self.animation_manager.DrawTouchAnimation(self.window)

            # Handle events
            events = self.pygame.event.get()
            for event in events:
                mouse_pos = self.pygame.mouse.get_pos()
                # print(mouse_pos)#TEMP
                if event.type == self.pygame.QUIT:
                    self.run = False  # Stops the program entirely
                if event.type == self.pygame.MOUSEBUTTONUP:  # on mouse release play animation to show where cursor is
                    self.animation_manager.StartTouchAnimation(mouse_pos)  # play animation

            self.pygame.display.update()  # Update all drawn objects

        """
        if self.run:

            # Get the level's data
            level_data = self.music_data["tut"][1]  # load tut song data
            self.track_name = level_data["song_name"]
            self.distract_song = level_data["distract_song"]  # will be None or a list of songs
            self.segment_num = int(level_data["seg_num"])  # split song into this many segs

            segments, num_correct_segs  = self.create_segments(self.segment_num, self.track_name, self.distract_song)
            randomised_segments, loading_button, unknown_slots = self.create_graphics(segments, num_correct_segs)

            # loop through each graphic that we care about
            for key in tut_graphics.keys():
                # Define some variables for the tut sequence
                tut_key = tut_graphics[key]["keys"]  # draw grey graphics of everything except for our focused graphic
                tut_speech = tut_graphics[key]["speech"]
                tut_rect = tut_graphics[key]["rect"]
                speaking_timer = self.command_manager.qt_say(tut_speech)  # QT should say text out loud
                qt_done_talking = False  # Hold execution until user clicks somewhere

                # set logic based on what graphic we focus on
                target_graphics, target_event = self.get_target_behaviour(key, given_half, draggable_buttons,
                                                                          main_buttons)

                while not qt_done_talking and not rospy.is_shutdown() and self.run:

                    grey_graphics = self.update_grey_graphics(given_half, draggable_buttons,
                                                              main_buttons)  # update all grey graphics

                    # Render graphics
                    self.renderer.DrawBackground(self.background_colour)  # draw background
                    self.load_list_graphics(grey_graphics, tut_key)  # load specified grey objects
                    if target_graphics != []:
                        for graphic in target_graphics:  # Render the target graphic
                            graphic()  # Render graphics each
                    self.animation_manager.DrawTouchAnimation(self.window)  # Draw touch animation

                    # Handle events
                    events = self.pygame.event.get()
                    for event in events:
                        # print(self.pygame.mouse.get_pos())#TEMP
                        if event.type == self.pygame.QUIT:
                            self.run = False  # Stops the program entirely
                            self.quit = True  # Tells us that the game was quit out of, and it didn't end organically
                    if target_event != None:
                        target_event(events)  # render the target graphic
                        pass

                    # Check for when QT is done speaking_continue
                    qt_done_talking = self.command_manager.robo_timer.CheckTimer(speaking_timer)

                    # highlight the box we want and draw it
                    self.highlight_block(events, target_rect=tut_rect, msg="", timer_complete=qt_done_talking)

                    self.pygame.display.update()  # Update all drawn objects
        """

    def play_music_blocking(self, difficulty, level):
        """Level with just music player"""
        if self.run:

            # Get the level's data
            level_data = self.music_data[difficulty][level]  # {"song_name":"title", "mood":"happy", "hint":"some text"}
            self.track_name = level_data["song_name"]

            # Create buttons
            self.next_button = self.CreateButton("next_button.png", "next_button_grey.png", (self.cen_x - 300, 1200),
                                                 scale=1)
            self.play_button = self.create_play_button("pause_button.png", "pause_button_grey.png", "play_button.png",
                                                     "play_button_grey.png", "rewind_button.png",
                                                     "rewind_button_grey.png", (self.cen_x - 175, 550), scale=1.5,
                                                       on_play=self.sound_manager.unpause,
                                                       on_pause=self.sound_manager.pause)  # create pause and play button

            # Load track
            self.sound_manager.load_track(self.track_name)

            # Variables
            music_playing = True
            self.song_ended = False
            self.current_track_time = 0
            self.track_total_time = 100
            self.progress = 0
            self.current_track_time, self.track_total_time, self.progress, self.song_ended = self.get_song_info(
                self.current_track_time, self.track_total_time)

            self.sound_manager.unpause()
            music_ended = False
            next_pressed = False

            while not next_pressed and not rospy.is_shutdown() and self.run:
                time = rospy.get_time()

                # get song data
                self.current_track_time, self.track_total_time, self.progress, self.song_ended = self.get_song_info(
                    self.current_track_time, self.track_total_time)

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
                    if event.type == self.pygame.MOUSEBUTTONUP:
                        self.animation_manager.StartTouchAnimation(mouse_pos)  # draw mouse click animation
                    # Check for button press
                    if music_ended:  # if music ended start checking for next press, otherwise ignore it
                        next_button_clicked = self.next_button.get_event(event, mouse_pos)
                        if next_button_clicked:
                            next_pressed = True
                            self.sound_manager.stop_track()
                        self.play_button.get_event(event, mouse_pos)  # only check for button press on music end

                # print(rospy.get_time() - time)

    def play_level(self, difficulty, level_num):
        """Sequence plays the levels"""
        if self.run:  # Dont start this screen if the previous screen wanted to close out the game

            # Get the level's data
            level_data = self.music_data[difficulty][
                level_num]  # {"song_name":"title", "mood":"happy", "hint":"some text"}
            self.track_name = level_data["song_name"]
            self.distract_song = level_data["distract_song"]  # will be None or a list of songs
            self.segment_num = int(level_data["seg_num"])  # split song into this many segs
            fps = "0"  # for debug info

            given_half, song_unknown, draggable_buttons, draggable_pos, main_buttons, correct_segments = self.create_graphics(
                self.segment_num, self.track_name, self.distract_song)

            song_restored = False

            # our correct slots
            slots = {}
            for slot in range(self.segment_num):
                slots[slot] = ""

            # Copy format of our previous data
            is_in_slot = draggable_pos.copy()  # the inital pos of each draggable
            for key in is_in_slot.keys():
                is_in_slot[key] = False

            #           -----------------------------------------------------------

            while not song_restored and not rospy.is_shutdown() and self.run:

                # Draw background and objects
                self.update_graphics(given_half, draggable_buttons, main_buttons)  # draw coloured graphics
                self.rendered_graphics = self.update_grey_graphics(given_half, draggable_buttons, main_buttons)

                # self.rendered_graphics = self.update_grey_graphics(current_track_time, track_total_time, progress, slider_x, slider_y)  #save updated grey graphics into attribute
                if self.debug:
                    self.draw_debug_info(fps)  # draw fps onto screen

                self.pygame.display.update()  # Update all drawn objects

                events = self.pygame.event.get()

                # Check if user is doing things
                self.sayings.qt_reminder(events)

                for event in events:

                    if event.type == self.pygame.QUIT:
                        self.run = False  # Stops the program entirely

                    if event.type == self.pygame.MOUSEBUTTONUP:  # on mouse release play animation to show where cursor is
                        self.animation_manager.StartTouchAnimation(
                            mouse_pos)  # tell system to play animation when drawing

                    # Check for button press
                    mouse_pos = self.pygame.mouse.get_pos()
                    for button in main_buttons:
                        button_press = button.get_event(event, mouse_pos)

                    given_press = given_half.get_event(event, mouse_pos)
                    if given_press:  # if the track was started, or stopped, set them all to false
                        for key in draggable_buttons:
                            draggable_buttons[key].toggle = False

                    for key in draggable_buttons:  # loop through dict of buttons and handle events on press
                        press, button_pos = draggable_buttons[key].get_event(event, mouse_pos)
                        # Check if toggle is pressed
                        if press:
                            given_half.toggle_state = False
                            for inner_key in draggable_buttons:  # if pressed, set all others to false
                                if inner_key != key:  # skip our pressed one
                                    draggable_buttons[inner_key].toggle = False

                                    # Check if seg is placed in the slot and allow only if the slot is free
                        if song_unknown.rect.collidepoint(button_pos.center) and not draggable_buttons[
                            key].mouse_is_held and slot_free:  # if our dragged part is released on top of the song slot
                            draggable_buttons[key].rect.x = song_unknown.rect.x
                            draggable_buttons[key].rect.y = song_unknown.rect.y
                            current_seg_order = current_seg_order + [
                                draggable_buttons[key].return_info]  # TODO change this so it's scaleable
                            is_in_slot[key] = True  # this is currently in the slot
                            slot_free = False

                            # check if answers are corect once one is dragged in
                            if current_seg_order == correct_segments:
                                self.command_manager.send_qt_command(emote="happy", gesture="nod")
                                qt_message = (self.sayings.get_agreements())  # QT reads out a randomised "good job"
                                self.level_loader.QTSpeakingPopupScreen(qt_message, self.rendered_graphics, self.run,
                                                                        self.background_colour)
                                song_restored = True
                            else:
                                self.command_manager.send_qt_command(emote="sad", gesture="shake_head")
                                qt_message = (self.sayings.get_disagreements())  # QT reads out level's hint
                                self.level_loader.QTSpeakingPopupScreen(qt_message, self.rendered_graphics, self.run,
                                                                        self.background_colour)
                        # if it was placed into the slot when something else is in the slot
                        elif song_unknown.rect.collidepoint(button_pos.center) and not draggable_buttons[
                            key].mouse_is_held and not slot_free:
                            draggable_buttons[key].rect.x = draggable_pos[key][0]
                            draggable_buttons[key].rect.y = draggable_pos[key][1]
                        # If it wasn't placed in the slot, place it back in it's original location, and track which seg has left the slot
                        elif not song_unknown.rect.collidepoint(button_pos.center) and not draggable_buttons[
                            key].mouse_is_held:  # move the button back to where it was if it was released somewhere random
                            if is_in_slot[key]:  # if we moved out from the slot, reset some variables
                                slot_free = True
                                current_seg_order.pop(1)  # TODO change this to be scalable
                                is_in_slot[key] = False
                            else:
                                draggable_buttons[key].rect.x = draggable_pos[key][0]
                                draggable_buttons[key].rect.y = draggable_pos[key][1]

                if self.debug:
                    fps = self.get_fps()  # calculate FPS of this loop and show it next loop

    #################################################################Main####################################################################

    def Main(self, difficulty="easy",
             level=1):  # input what level and difficulty to play, the program will handle the rest
        """Main Func"""

        """
        #Introduce game
        self.run = self.level_loader.QTSpeakingScreen("Lets play Fix The Song!", self.run, self.background_colour)

        #Ask if they want to play tutorial
        self.run, tut = self.level_loader.yes_or_no_screen('Should I explain how to play "Fix The Song" ?', self.run, self.background_colour)
        if tut:
            self.guided_tut()
        
        #Count in to the start of the game
        self.run = self.level_loader.tap_to_continue(self.run, self.background_colour)

        #Count into level to slow pacing
        self.run = self.level_loader.countdown(3, self.run, self.background_colour, prelim_msg = "Get ready to hear the song!")
        
        #Play the track and block
        self.play_music_blocking(difficulty, level)

        #Play main level
        self.play_level(difficulty, level)
        """

        # self.play_music_blocking(difficulty, level)
        # self.play_level(difficulty, 3)
        self.guided_tut()


######################################################On execution#######################################################################

# If we run this node, run the game on it's own
if __name__ == '__main__':
    # Initialise game
    rospy.init_node('fix_song_game', anonymous=False)
    rospy.loginfo("Node launched successfully")
    game_object = Fix_The_Song_Game()

    # Run the game
    try:
        game_object.Main()
    except(KeyboardInterrupt or rospy.exceptions.ROSInterruptException):
        game_object.pygame.quit
        SoundManager("").stop_track()
        print("Audio may not be stopped due to interrupt")

    # on exit delete music files and stop music
    #game_object.empty_temp_dir() # TODO finish this method
    SoundManager("").stop_track()
