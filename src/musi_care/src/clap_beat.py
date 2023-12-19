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

class Clap_To_Beat_Game():
    """ Class to generate and handle guess the mood game """

    def __init__(self,reduce_screen=False, debug=False, screen = None, my_pygame = None, inputMode = 2):
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

        # Set inputs to either touch or mouse
        self.input_mode = inputMode  # input mode 1 for touch, 2 for mouse
        if self.input_mode == 2:
            self.inputUp = self.pygame.MOUSEBUTTONUP
            self.inputDown = self.pygame.MOUSEBUTTONDOWN
            self.inputMotion = self.pygame.MOUSEMOTION
        else:
            self.inputUp = self.pygame.FINGERUP
            self.inputDown = self.pygame.FINGERDOWN
            self.inputMotion = self.pygame.FINGERMOTION

        res = pygame.display.Info()  # get our screen resolution
        if screen == None:
            if reduce_screen:
                self.window_x = 3000 - 150  # Width of window -150 to account for the linux toolbar
            else:
                self.window_x = 3000
            self.window_y = 2000  # Height of window
            self.window_center = (int(self.window_x / 2), int(self.window_y / 2))
            self.cen_x = self.window_center[0]
            self.cen_y = self.window_center[1]
            self.window = pygame.display.set_mode((self.window_x, self.window_y))  # Create window and set size
        else:
            self.window = screen
            self.window_x = self.window.get_width()
            self.window_y = self.window.get_height()
            self.window_center = (int(self.window_x / 2), int(self.window_y / 2))
            self.cen_x = self.window_center[0]
            self.cen_y = self.window_center[1]
        self.background_colour = (100, 100, 100)  # background black by default
        self.pygame.display.set_caption("Clap to the beat!")  # Label window
        self.run = True
        self.quit = False  # Check to see if the game ended or it was quit
        self.track_playing = False
        self.previous_track_data = None
        self.difficulty = "easy"  # Default difficulty to play
        self.music_filepath = "/game_assets/music/"  # relative path to songs # "/home/qtrobot/catkin_ws/src/musi_care/src/game_assets/music/"
        self.timer_manager = TimeFunctions()
        self.animation_manager = AnimationManager(self.pygame)
        self.sound_manager = SoundManager(self.music_filepath)  # load soundplayer with sound file path
        self.command_manager = QTManager(debug = debug)
        self.renderer = Renderer(self.window, self.window_center)
        self.level_loader = StandardLevels(self.window, self.window_center, self.pygame, self.music_filepath, debug = debug)
        self.segment_x_y = {0: (610, 850), 1: (1260, 850), 2: (1910, 850), 3: (610, 1400), 4: (1260, 1400),
                            5: (1910, 1400)}  # hard coded num locations of each segment
        self.behaviours_manager = Behaviours(self.pygame, self.music_filepath)
        self.t1 = 0  # t1 for FPS tracking
        self.debug = debug
        if not self.debug:
            self.pygame.mouse.set_visible(False)  # set to false when not testing
        self.command_manager.set_arm_vel(100) # Set robot arm speed to max for this game
        self.diy_box = False
        self.Finished = False # used to stop threads recording and QT clapping
        # self.music_vol = 1 # change volume of laptop
        # self.qt_voice_vol
        # self.sound_manager.volume_change(self.music_vol) # Set a default volume
        # self.set_robot_volume(qt_voice_vol) #TODO add this functionality

    ########################################################Low level methods################################################################

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
        # Do this first, since it's more time sensitive
        progress = self.elapsed_time_secs / self.total_track_secs  # elapsed time in percentage completion, so slider can represent that on a bar
        song_ended = progress >= 0.99  # if progress > 99% = song is finished, otherwise false
        if song_comp_only:
            return song_ended

        # Get variables that we will draw onto screen
        formatted_data = self.get_track_info(formatted_output=True)
        current_track_time = formatted_data[0]  # Time gotten from sound_player node
        track_total_time = formatted_data[1]  # Total track time

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

    def create_button(self, file_name, location, return_info={}, scale=1, unique_id=""):
        """code creates button using the button_image class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/graphics'
        file_path = os.path.join(this_file_path, relative_path, file_name)

        button = Button(file_path, location, self.pygame, return_info={}, scale=scale, unique_id=unique_id)
        return (button)

    def create_toggle_button(self, file_name, alt_file_name, location, scale=2,
                             unique_id="", return_info="", when_toggle_on=object, when_toggle_off=object):
        """code creates button using the button_image class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = '/game_assets/graphics/'
        file_path = this_file_path + relative_path + file_name
        alt_path = this_file_path + relative_path + alt_file_name

        button = ToggleButton(file_path, alt_path, location, self.pygame, scale,
                              unique_id, return_info, when_toggle_on, when_toggle_off)
        return (button)

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
            if event.type == self.inputUp:  # on mouse release play animation to show where cursor is
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
            unknown_seg = self.create_button("music_segment_greyed_out.png", (
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
        help_button = self.create_button("help_button.png", (2200, 100), scale=1,
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

    def record_claps(self):
        """ Multithreaded function that records claps"""
        # TODO add functionality that records audio and claps until a timer ends (timer is the song duration)
        while self.run and not self.Finished:
            print("Pretending to record clapping")
            time.sleep(2) # TEMP
        print("Ended clap recording thread")

    def get_beat_timing(self, first_beat, bpm, track_total_time):
        """
        Use level data and return a list of the timings of each beat
        track_total_time should be in seconds
        """
        print("Track time: ",track_total_time)
        beat = first_beat
        beat_timings = []
        while beat < track_total_time:
            beat_timings.append(beat)
            beat += 60 / bpm
        return beat_timings

    def hit_drum(self, beat_timings, start_time):
        """
        Use a list of beat times, to hit drum at the right time
        prev_arm will be either "left" or "right"
        It will then pop the list of beat_timings, so that we dont keep hitting the same beat.
        """
        # Set arm pos goals
        if self.diy_box:
            hitting_drum = [-50.5, -84.69999694824219, -39.400001525878906]
            raised_arm = [-16.299999237060547, -85.0, -25.700000762939453]
        else:
            raised_arm = [2.9000000953674316, -84.69999694824219, -49.20000076293945]
            hitting_drum = [-27.0, -85.4000015258789, -56.29999923706055]

        time_to_hit = 0.3 # est travel time taken for arms to hit drum
        i = 0
        while i < len(beat_timings) and not rospy.is_shutdown():
            # Update beat hit time
            beat_time = beat_timings[i]

            # Update elapsed time globally
            self.elapsed_time_secs = rospy.get_time() - start_time

            # Hit the drum if it's time
            if self.elapsed_time_secs + time_to_hit >= beat_time:
                # On even numbers use right arm else, left
                if i % 2 == 0 :
                    self.command_manager.move_right_arm(hitting_drum)
                    self.command_manager.move_left_arm(raised_arm)
                else:
                    self.command_manager.move_right_arm(raised_arm)
                    self.command_manager.move_left_arm(hitting_drum)
                # Move onto next beat
                i += 1

        print("QT has finished playing")


    def analyse_performance(self):
        """Takes the recording / data from the recording of the clapping """
        #TODO complete this section
        temporal_accuracy = 0.9 #90%
        numerical_accuracy = 0.9 #90%
        return temporal_accuracy, numerical_accuracy

    def play_level(self, file_name, first_beat, bpm):
        """Have QT clap to beat and record user clapping"""

        if self.run:

            # Get the level's data
            self.track_name = file_name

            # Define some parameters and vars
            self.sound_manager.load_track(self.track_name)
            song_done = False
            fade_time = 5
            time_of_darkness = rospy.get_time() + fade_time # what system time, the screen should be dark at
            time_left = time_of_darkness - rospy.get_time()
            prev_arm = "left"
            track_total_time = self.get_track_info()[2]

            # Start parallel thread to record claps simultaneously
            clap_recorder = threading.Thread(target=self.record_claps, args=(), daemon=True)
            clap_recorder.start()  # Start multi_threaded function

            # Start another parallel thread for QT to clap at right time
            beat_timings = self.get_beat_timing(first_beat, bpm, track_total_time)
            start_time = rospy.get_time()
            qt_clap = threading.Thread(target=self.hit_drum, args=(beat_timings, start_time), daemon=True)
            qt_clap.start()  # Start multi_threaded function

            # Start song
            self.sound_manager.unpause()

            # Render screen and wait til song done.
            while not song_done and len(beat_timings) > 0 and self.run and not rospy.is_shutdown():

                # Render screen, will fade to black and stay black
                message = "Please Look At QT Robot"
                if time_left > 0:
                    time_left = time_of_darkness - rospy.get_time()
                    fade_scalar = (time_left / fade_time)
                else:
                    fade_scalar = 0
                self.run = self.level_loader.fade_to_black_screen(self.run, message, self.background_colour, fade_scalar)

                # Update elapsed time
                self.elapsed_time_secs = rospy.get_time() - start_time

                # Check if song done
                if (self.elapsed_time_secs / self.total_track_secs) > 0.99:
                    song_done = True

            self.Finished = True

            # Stop recording clapping and log the user's score
            temporal_accuracy, numerical_accuracy = self.analyse_performance()

            # QT should praise user based on performance
            message = self.qt_reward(temporal_accuracy, numerical_accuracy)
            self.command_manager.send_qt_command(speech=message, gesture="arms_up", emote="happy")

            clear_type = "clear"
            e_clear_req = {"expected_num_claps": 100 , "average_error": "0.05"}
            performance = {"num_claps": 95, "average_error": "0.1"}
            level_data = {"game_name": "CTB", "clear_type": clear_type, "performance": performance,
                          "e_clear_req": e_clear_req}

            return self.run, level_data



    #################################################################Main####################################################################

    def Main(self, file_name, first_beat, bpm, ask_tut = True):  # input what level and difficulty to play, the program will handle the rest
        """Main Func"""
        # Introduce game
        #self.run = self.level_loader.QTSpeakingScreen("Lets play Clap To The Beat!", self.run, self.background_colour)

        if ask_tut:
            # Ask if they want to play tutorial
            self.run, tut = self.level_loader.yes_or_no_screen('Should I explain how to play "Clap To The Beat" ?', self.run,
                                                               self.background_colour)

            if tut:
                tut_msg = """ I will drum along to the beat... You need to watch what I am doing,,, and clap along with me...
                If you want,, you can click your fingers, or tap the table instead of clapping.. """

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
        # Play main level
        self.run, level_data = self.play_level(file_name, first_beat, bpm)

        return self.run, level_data



######################################################On execution#######################################################################

# If we run this node, run the game on it's own
if __name__ == '__main__':
    # Initialise game
    rospy.init_node('clap_beat_game', anonymous=False)
    rospy.loginfo("Node launched successfully")
    game_object = Clap_To_Beat_Game(debug=True)

    # Run the game
    try:
        game_object.Main("happy_3.wav", 0.1, 60, True)
    except(KeyboardInterrupt or rospy.exceptions.ROSInterruptException):
        game_object.pygame.quit
        SoundManager("").stop_track()
        print("Audio may not be stopped due to interrupt")

    SoundManager("").stop_track()