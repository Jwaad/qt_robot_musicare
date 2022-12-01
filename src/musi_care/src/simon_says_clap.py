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
import logging
import threading


#################################################################Initialise#################################################################

class Simon_Says_Clap_Game():
    """ Class to generate and handle guess the mood game """

    def __init__(self):
        """Initialise"""
        x = 145  # x pos of screen
        y = 0  # y pos of screen
        os.environ['SDL_VIDEO_WINDOW_POS'] = '%d,%d' % (x, y)  # move screen to x and y pos
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
        self.pygame.display.set_caption("Simon says clap!")  # Label window
        self.run = True
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
        self.segment_x_y = {0: (610, 850), 1: (1260, 850), 2: (1910, 850), 3: (610, 1400), 4: (1260, 1400),
                            5: (1910, 1400)}  # hard coded num locations of each segment
        self.behaviours_manager = Behaviours(self.pygame, self.music_filepath)
        self.t1 = 0  # t1 for FPS tracking
        self.debug = True
        if not self.debug:
            self.pygame.mouse.set_visible(False)  # set to false when not testing
        #self.command_manager.init_robot(100) # Set robot arm speed to max for this game
        # self.music_vol = 1 # change volume of laptop
        # self.qt_voice_vol
        # self.sound_manager.volume_change(self.music_vol) # Set a default volume
        # self.set_robot_volume(qt_voice_vol) #TODO add this functionality

    ########################################################Low level methods################################################################

    def get_song_database(self):
        """Read the database file and get the levels data"""

        # data_filepath = ("/home/qtrobot/catkin_ws/src/musi_care/src/game_assets/music/music_data.txt")
        data_filepath = ("/game_assets/data/cbg_level_data.txt")  # gtm = guess the mood
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

    def get_song_info(self, prev_track_time="", prev_total_time="", song_comp_only=False):
        # Get variables that we will draw onto screen
        formatted_data = self.get_track_info(formatted_output=True)
        current_track_time = formatted_data[0]  # Time gotten from sound_player node
        track_total_time = formatted_data[1]  # Total track time
        progress = self.elapsed_time_secs / self.total_track_secs  # elapsed time in percentage completion, so slider can represent that on a bar
        song_ended = progress >= 0.99  # if progress > 99% = song is finished, otherwise false

        if song_comp_only:
            return song_ended
        else:
            return current_track_time, track_total_time, progress, song_ended

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

    def create_button(self, file_name, alt_file_name, location, return_info={}, scale=1, unique_id=""):
        """code creates button using the button_image class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/graphics'
        file_path = os.path.join(this_file_path, relative_path, file_name)
        alt_path = os.path.join(this_file_path, relative_path, alt_file_name)

        button = Button(file_path, alt_path, location, self.pygame, return_info={}, scale=scale, unique_id=unique_id)
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

    def create_text(self, window, window_center, text, location=None, cen_x=False, cen_y=False, font_size=30,
                    font_colour=(255, 255, 255)):
        text_object = TextObject(window, window_center, text, location=location, cen_x=cen_x, cen_y=cen_y,
                                 font_size=font_size,
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

    def create_graphics(self, segments, num_correct_slots, single_out=False):
        """Create the pygame objects that we will use """

        # Randomise the positions of our buttons and change their pos and store it
        randomised_segments = segments
        random.shuffle(randomised_segments)
        for seg_i in range(len(randomised_segments)):
            randomised_segments[seg_i].set_pos(self.segment_x_y[seg_i])
            randomised_segments[seg_i].return_info["init_pos"] = self.segment_x_y[seg_i]

        # Create loading button
        loading_button = self.create_button("loading_screen_button_depressed.png",
                                            "loading_screen_button_depressed.png",
                                            (270, 550), scale=2.3)

        # Create unknown button slots and have them scale according to how many there is
        unknown_y = 150
        unknown_slots = []
        # Create the unknown slots we need
        for i in range(num_correct_slots):
            unknown_seg = self.create_button("music_segment_greyed_out.png", "music_segment_greyed_out.png", (
                unknown_y, 0), scale=2)
            unknown_slots.append(unknown_seg)
        # Change position of unknown slots
        step = (unknown_slots[0].get_rect()[2])  # step = width of slots
        starting_x = self.window_center[0] - (step * (len(unknown_slots) / 2))  # the x of the left-most slot
        i = 0
        for slot in unknown_slots:
            unknown_x = starting_x + step * i
            slot.set_pos((unknown_x, unknown_y))
            slot.set_info({"pos": i, "slot_full": False})  # Store the order of the unknown segments in the button
            i += 1

        # Create text objects
        top_text = self.create_text(self.window, self.window_center, "Put the song back together", cen_x=True,
                                    font_size=90)
        middle_text = self.create_text(self.window, self.window_center, "Drag the pieces into the slots",
                                       location=[0, 680], cen_x=True, font_size=90)
        text_objs = [top_text, middle_text]

        # Create help button
        help_button = self.create_button("help_button.png", "help_button_grey.png", (2200, 100), scale=1,
                                         unique_id="help")
        if single_out:
            return [loading_button] + unknown_slots + text_objs + [help_button] + randomised_segments
        else:
            return loading_button, unknown_slots, text_objs, help_button, randomised_segments

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
        # render graphics 1 by 1, only if they match the key
        for i in range(len(graphics)):
            if (i + 1) in key:
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

    def qt_reward(self, temporal_accuracy, numerical_accuracy, should_gesture = False):
        """handles what QT should say and do on level end """
        # "Perfect clear"
        if temporal_accuracy > 0.8 and numerical_accuracy > 0.8:
            qt_message = (self.behaviours_manager.get_generic_praise()) # QT reads out level's hint
        else:
            qt_message = (self.behaviours_manager.get_generic_light_praise())  # QT reads out level's hint
        if should_gesture:
            self.command_manager.send_qt_command(emote="happy", gesture="nod")
        return qt_message

    #######################################################Level / screen code###############################################################

    def record_claps(self, level_data ):
        """ Multithreaded function that records claps"""
        # TODO add functionality that records audio and claps until a timer ends (timer is the song duration)
        while self.run:
            print("Pretending to record clapping")
            time.sleep(2) # TEMP
        print("Ended clap recording thread")

    def get_beat_timing(self, level_data, track_total_time):
        """
        Use level data and return a list of the timings of each beat
        track_total_time should be in seconds
        """
        print("Track time: ",track_total_time)
        first_beat = float(level_data["first_beat"])
        bpm = int(level_data["bpm"])
        beat = first_beat
        beat_timings = []
        while beat < track_total_time:
            beat_timings.append(beat)
            beat += 60 / bpm
        return beat_timings

    def hit_drum(self, beat_timing, elapsed_time, prev_arm,):
        """
        Use a list of beat times, to hit drum at the right time
        prev_arm will be either "left" or "right"
        It will then pop the list of beat_timings, so that we dont keep hitting the same beat.
        """
        # get out just the beat timing we want, and also the hard_coded time it takes to hit the drum
        this_beat = beat_timing[0]
        time_to_hit = 0.3
        raised_arm = [-26.700000762939453, -85.4000015258789, -58.29999923706055]
        hitting_drum = [17.200000762939453, -80.0999984741211, -46.599998474121094]

        #Check if we should hit the drum this loop
        if elapsed_time + time_to_hit > this_beat:
            # Function that hits the drum with either the left or right hand
            if prev_arm == "left":
                # Move left arm back up and move right arm down
                self.command_manager.move_right_arm(hitting_drum)
                self.command_manager.move_left_arm(raised_arm)
                beat_timing.pop(0)
                prev_arm = "right"
                print("Right arm")
                return beat_timing, prev_arm
            else:
                # Move right arm back up and move left arm down
                self.command_manager.move_right_arm(raised_arm)
                self.command_manager.move_left_arm(hitting_drum)
                beat_timing.pop(0)
                prev_arm = "left"
                print("Left arm")
                return beat_timing, prev_arm
        else:
            return beat_timing, prev_arm

    def analyse_performance(self):
        """Takes the recording / data from the recording of the clapping """
        #TODO complete this section
        temporal_accuracy = 0.9 #90%
        numerical_accuracy = 0.9 #90%
        return temporal_accuracy, numerical_accuracy

    def play_level(self, difficulty, level):
        """Have QT clap to beat and record user clapping"""

        if self.run:

            # Get the level's data
            level_data = self.music_data[difficulty][level]  # {"song_name":"title", "mood":"happy", "hint":"some text"}
            self.track_name = level_data["song_name"]

            # Start parallel thread to record claps simultaneously
            clap_recorder = threading.Thread(target=self.record_claps, args=(level_data,), daemon=True)
            clap_recorder.start()  # Start multi_threaded function

            # Define some parameters and vars
            self.sound_manager.load_track(self.track_name)
            song_done = False
            fade_time = 5
            time_of_darkness = rospy.get_time() + fade_time
            time_left = time_of_darkness - rospy.get_time()
            prev_arm = "left"
            track_total_time = self.get_track_info()[2]
            beat_timings = self.get_beat_timing(level_data, track_total_time)
            new_random = True
            freeze = False
            self.sound_manager.unpause()
            while not song_done and len(beat_timings) > 0 and self.run and not rospy.is_shutdown():
                # Get elapsed time
                elapsed_time = self.get_track_info()[1]

                # Stop beat every 5 - 12 seconds
                if new_random:
                    freeze_time = self.timer_manager.CreateTimer("freeze", random.randint(5, 12))
                    new_random = False
                if len(self.timer_manager.timers) > 0:
                    if self.timer_manager.CheckTimer(freeze_time):
                        self.command_manager.send_qt_command(speech="Stop!")
                        time_unfreeze = self.timer_manager.CreateTimer("unfreeze",random.randint(2, 5))
                        freeze = True
                        self.sound_manager.pause()


                # Render screen, will fade to black and stay black
                message = "Please Look At QT Robot"
                if time_left > 0:
                    time_left = time_of_darkness - rospy.get_time()
                else:
                    time_left = 0
                fade_scalar = (time_left / fade_time)
                self.run = self.level_loader.fade_to_black_screen(self.run, message, self.background_colour, fade_scalar )

                if not freeze:
                    # Hit the drum to the beat.
                    beat_timings, prev_arm = self.hit_drum(beat_timings, elapsed_time, prev_arm)

                    # Check if song done
                    song_done = self.get_song_info(song_comp_only=True)
                else:
                    if self.timer_manager.CheckTimer(time_unfreeze):
                        freeze = False
                        new_random = True
                        self.command_manager.send_qt_command(speech="Go!", gesture="grin")
                        self.sound_manager.unpause()

            # Stop recording clapping and log the user's score
            temporal_accuracy, numerical_accuracy = self.analyse_performance()

            # QT should praise user based on performance
            message = self.qt_reward(temporal_accuracy, numerical_accuracy)
            self.command_manager.send_qt_command(speech=message, gesture="arms_up", emote="Happy")

            return temporal_accuracy, numerical_accuracy



    #################################################################Main####################################################################

    def Main(self, difficulty="easy",
             level=1):  # input what level and difficulty to play, the program will handle the rest
        """Main Func"""
        """
        # Introduce game
        self.run = self.level_loader.QTSpeakingScreen("Lets play Clap To The Beat!", self.run, self.background_colour)

        # Ask if they want to play tutorial
        self.run, tut = self.level_loader.yes_or_no_screen('Should I explain how to play "Clap To The Beat" ?', self.run,
                                                           self.background_colour)
        if tut:
            tut_msg =
            "I will drum along to the beat. You need to watch what I am doing and clap along with me...
            If you want, you can click your fingers or tap the table instead of clapping."
            
            repeat = True
            while repeat:
                self.run = self.level_loader.QTSpeakingScreen(tut_msg, self.run, self.background_colour)
                self.run, repeat = self.level_loader.yes_or_no_screen('Should I repeat that explanation ?',
                                                                   self.run,
                                                                   self.background_colour)

        # Count in to the start of the game
        self.run = self.level_loader.tap_to_continue(self.run, self.background_colour)

        # Count into level to slow pacing
        self.run = self.level_loader.countdown(3, self.run, self.background_colour,
                                               prelim_msg="Get ready to clap along!")
        """
        # Play main level
        temporal_accuracy, numerical_accuracy = self.play_level(difficulty, level)

        # Save user data
        #print(temporal_accuracy, numerical_accuracy)



######################################################On execution#######################################################################

# If we run this node, run the game on it's own
if __name__ == '__main__':
    # Initialise game
    rospy.init_node('simon_says_clap', anonymous=False)
    rospy.loginfo("Node launched successfully")
    game_object = Simon_Says_Clap_Game()

    # Run the game
    try:
        game_object.Main()
    except(KeyboardInterrupt or rospy.exceptions.ROSInterruptException):
        game_object.pygame.quit
        SoundManager("").stop_track()
        print("Audio may not be stopped due to interrupt")

    SoundManager("").stop_track()
