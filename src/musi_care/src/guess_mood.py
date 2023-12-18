#!/usr/bin/env python

import pygame
import pygame.freetype
import time
import numpy as np
import sys
import rospy
import os
import functools
import random
import math
from random import randint
from musi_care.msg import SongData
from musi_care.srv import sound_player_srv
from musi_care.srv import qt_command
from musicare_lib import TimeFunctions
from musicare_lib import Button
from musicare_lib import PausePlayButton
from musicare_lib import AnimationManager
from musicare_lib import SoundManager
from musicare_lib import QTManager
from musicare_lib import Renderer
from musicare_lib import HorizontalSlider
from musicare_lib import StandardLevels
from musicare_lib import Behaviours
from musicare_lib import General


#################################################################Initialise#################################################################

class Guess_The_Mood_Game():
    """ Class to generate and handle guess the mood game """

    def __init__(self, reduce_screen=False, debug=False, screen = None, my_pygame = None, inputMode = 2):
        """
        reduce_screen = bool, whether or not to reduce borders to account for linux toolbar
        debug = bool, whether or not to display things like FPS and minor helpful tools.
        screen = pygame_screen, should be a pygame window, can be used as param, to blit graphics onto a specific screen,
            instead of making a new one
        my_pygame = pygame lib, should be initiated pygame lib. Similar to screen, can be parameter, to use a pre init
            pygame, instead of init our own
        inputMode = Decide whether to use touch or mouse: 1 for touch, 2 for mouse
        """
        self.debug = debug
        if my_pygame == None:
            self.pygame = pygame
            self.pygame.init()  # start py engine
            self.pygame.freetype.init()
        else:
            self.pygame = my_pygame
        # Set inputs to either touch or mouse
        self.input_mode = inputMode # input mode 1 for touch, 2 for mouse
        if self.input_mode == 2:
            self.inputUp = self.pygame.MOUSEBUTTONUP
            self.inputDown = self.pygame.MOUSEBUTTONDOWN
            self.inputMotion = self.pygame.MOUSEMOTION
        else:
            self.inputUp = self.pygame.FINGERUP
            self.inputDown = self.pygame.FINGERDOWN
            self.inputMotion = self.pygame.FINGERMOTION

        if screen == None:
            x = 150  # x pos of screen
            y = 0  # y pos of screen
            os.environ['SDL_VIDEO_WINDOW_POS'] = '%d,%d' % (x, y)  # move screen to x and y pos
            res = pygame.display.Info()  # get our screen resolution
            self.window_x = 3000
            if reduce_screen:
                self.window_x -= x # Width of window -150 to account for the linux toolbar
                self.window_y -= y  # height of window - y, to account for top bar
            self.window_y = 2000  # Height of window
            self.window_center = (int(self.window_x / 2), int(self.window_y / 2))
            self.window = pygame.display.set_mode((self.window_x, self.window_y))  # Create window and set size
        else:
            self.window = screen
            self.window_x = self.window.get_width()
            self.window_y = self.window.get_height()
            self.window_center = (int(self.window_x / 2), int(self.window_y / 2))
            self.cen_x = self.window_center[0]
            self.cen_y = self.window_center[1]
            #self.window_center = (self.window.get_height(), self.window.get_width())
        self.background_colour = (100, 100, 100)  # background grey by default
        self.pygame.display.set_caption("Guess The Mood!")  # Label window
        self.run = True
        self.quit = False  # Check to see if the game ended or it was quit
        self.music_filepath = "/game_assets/music/"  # relative path to songs # "/home/qtrobot/catkin_ws/src/musi_care/src/game_assets/music/"
        self.timer_manager = TimeFunctions()
        self.animation_manager = AnimationManager(self.pygame)
        self.sound_manager = SoundManager(self.music_filepath)  # load soundplayer with sound file path
        self.command_manager = QTManager(debug = self.debug)
        self.renderer = Renderer(self.window, self.window_center)
        self.behaviours_manager = Behaviours(self.pygame, self.music_filepath)
        self.level_loader = StandardLevels(self.window, self.window_center, self.pygame, self.music_filepath, debug)
        self.gm = General()  # gm = general methods
        self.track_playing = False
        self.previous_track_data = None
        self.difficulty = "easy"  # Default difficulty to play
        self.current_level = 1  # Default level to play
        self.music_data = self.get_song_database()  # From save file load all of the level data
        # self.music_vol = 1 # change volume of laptop
        # self.qt_voice_vol
        # self.sound_manager.volume_change(self.music_vol) # Set a default volume
        # self.set_robot_volume(qt_voice_vol) #TODO add this functionality
        self.t1 = 0  # t1 for FPS tracking
        self.e_clear_time = 4 # amount of time after track ends, where you're still considered fast.
        if not self.debug:
            self.pygame.mouse.set_visible(False)  # set to false when not testing

    #############################################################Low level methods###########################################################

    def get_song_database(self):
        """Read the database file and get the levels data"""
        music_data = self.gm.Load_Song_Data("GTM")
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

    def create_button(self, file_name, location, return_info={}, scale=1.0, unique_id="", should_grey=True):
        """code creates button using the button_image class."""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/graphics'
        file_path = os.path.join(this_file_path, relative_path, file_name)

        button = Button(file_path, location, self.pygame, return_info={}, scale=scale, unique_id=unique_id,
                        should_grey=should_grey)
        return (button)

    def CreatePlayButton(self, pause_path, play_path, rewind_path, location, scale=1, unique_id="", on_pause=object,
                         on_play=object, should_grey=True):
        """code creates toggle button using the toggle button class."""
        # get path to imgs
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/graphics'

        # load imgs
        pause_path = os.path.join(this_file_path, relative_path, pause_path)
        play_path = os.path.join(this_file_path, relative_path, play_path)
        rewind_path = os.path.join(this_file_path, relative_path, rewind_path)

        button = PausePlayButton(pause_path, play_path, rewind_path, location, self.pygame, scale, unique_id, on_pause,
                                 on_play, should_grey=should_grey)
        return (button)

    def CreateHorizontalSlider(self, slider_name, cursor_name, x_y_locations, scale=1, on_click=object,
                               on_release=object):
        """Creates horizontal slider using the horizontal slider class"""
        this_file_path = os.path.dirname(__file__)
        relative_path = 'game_assets/graphics'
        slider_path = os.path.join(this_file_path, relative_path, slider_name)
        cursor_path = os.path.join(this_file_path, relative_path, cursor_name)

        slider = HorizontalSlider(slider_path, cursor_path, x_y_locations, scale, on_click, on_release)
        return slider

    def update_graphics(self):
        """Redraw graphics """
        self.renderer.DrawBackground(self.background_colour)
        self.renderer.DrawTextCentered("What mood does this song have?", font_size=100, y=100)
        for button in self.buttons:  # Draw buttons using button list
            button.render(self.window)
        self.animation_manager.DrawTouchAnimation(self.window)  # last so it shows up on top

    def update_grey_graphics(self):
        """reinitialise grey graphics taking with parameters they need """
        # Load graphics into dict # need to be in a specific order for this tut
        grey_graphics = {}
        grey_graphics[1] = functools.partial(self.renderer.DrawTextCentered, "What mood does this song have?",
                                             font_size=100, y=100)
        i = 1
        for button in self.buttons:
            i += 1
            grey_graphics[i] = functools.partial(button.render, self.window, grey=True)

        return grey_graphics

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

    def highlight_block(self, events, target_rect=None, msg="", timer_complete=None):
        """
        Highlight a certain object and check for click

        events = pygame.get_events()
        target_rect = the rect that we want to highlight around
        msg = text to be drawn in the center of the screen, will auto allign
        timer_complete = time until
        """
        # Init vars so we dont return something we dont expect
        arrow_rect = None

        # Handle events
        for event in events:
            # reset / init variables
            option_chosen = ""
            mouse_pos = self.pygame.mouse.get_pos()
            if event.type == self.inputUp:  # on mouse release play animation to show where cursor is
                self.animation_manager.StartTouchAnimation(mouse_pos)  # tell system to play animation when drawing
                return True

        # Render graphics
        if target_rect != None:  # so we can have blocking functionality without highlighting
            self.renderer.HighlightRect(target_rect, self.pygame)  # draw arrow and box
        if msg != "":  # we can send an empty msg to msg instead to have it not display anything
            if timer_complete == None:  # if user didn't specify a timer, just show text like normal
                self.renderer.DrawTextCentered(msg, font_size=75, font_colour=(0, 0, 0))
            else:
                if timer_complete:  # only render once timer done
                    self.renderer.DrawTextCentered(msg, font_size=75, font_colour=(0, 0, 0))

        return False

    def load_list_graphics(self, graphics, keys):
        """takes graphics dict and loads them
            Graphics = {1: graphic_func_1, 2: graphic_func_2. etc}
            key = [1,3,4,5]
        """
        for key in graphics.keys():  # Draw each object
            if key in keys:  # only draw the graphics we ask for
                graphics[key]()  # run as func

    def create_graphics(self):
        """Create the pygame objects that we will use """
        self.sad_button = self.create_button("sad_button.png", (625, 650), scale=1.3, unique_id="sad")
        self.happy_button = self.create_button("happy_button.png", (625, 1050), scale=1.3, unique_id="happy")
        self.unsure_button = self.create_button("unsure_button.png", (800, 1450), scale=1, unique_id="unsure")
        self.play_button = self.CreatePlayButton("pause_button.png", "play_button.png", "rewind_button.png",
                                                 (self.window_center[0] - 175, 195), scale=1.5, on_play=self.sound_manager.unpause,
                                                 on_pause=self.sound_manager.pause)  # create pause and play button

    def get_target_behaviour(self, key):
        """tells our tut what to draw and what to do with events"""
        # TODO THIS IS SO BADLY DONE, WHAT THE HECK??? I WROTE THIS? PTUI
        target_graphics = []
        target_event_handler = None
        # Highlight sad and happy
        if key == 2:
            target_graphics = [functools.partial(self.sad_button.render, self.window),
                               functools.partial(self.happy_button.render, self.window)]
        # highlight help button
        elif key == 3:
            target_graphics = [functools.partial(self.unsure_button.render, self.window)]
        # Highlight play button
        elif key == 4:
            target_graphics = [functools.partial(self.play_button.render, self.window)]

        return target_graphics, target_event_handler  # if our logic sifts failed

        # Example on how to define target_event_handler
        """
        #define what to do with events
        def event_handler(events):send_qt_command
            pass
        target_event_handler = event_handler #copy to this var
        """

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


    def compute_clear_type(self, time_taken, correct_ans, hints_needed):
        """
        Compute clear type from performance
            e_clear - Exceptional clear, fast and accurate
            clear - Correct answer, but not fast, or needed hint
            fail - Slow and hints needed or wrong answer

        time_taken = time from start of level til end (float, seconds)
        correct_ans = if they got the correct mood (boolean)
        hints_needed = how many times they hit the help button (integer)
        """
        e_time = self.total_track_secs + self.e_clear_time # Total track time + 4 seconds
        if time_taken < e_time and correct_ans == True and hints_needed < 1:
            return "e_clear"
        elif correct_ans == False or hints_needed >= 2 :
            return "fail"
        else:
            return "clear"

    def qt_reward(self, clear_type):
        """
        Uses clear type to get what to say
        """
        if clear_type == "e_clear":
            self.command_manager.send_qt_command(emote="happy", gesture="nod")
            qt_message = (self.behaviours_manager.get_praise())  # QT praises user with some oomph
        else:
            self.command_manager.send_qt_command(emote="happy", gesture="nod")
            qt_message = (self.behaviours_manager.get_agreements())  # QT
        return qt_message
        # print(time_taken, wrong_answers, hints_needed)

    def draw_tut_options(self, arrow_rect):
        """ Using arrow rect, decide where tut buttons should be drawn"""
        self.tut_next.set_pos((800,200))
        self.tut_repeat.set_pos((1200,200))

        """
        ARCHIVED SECTION
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
        """

    def get_GTM_phaseone_help(self, previous_saying=""):
        """ Sentences QT will say when giving phase 1 help """
        sayings = ["Try to focus on how this song makes you feel... If you want a better hint, click the clue button again",
                   "How does the song make you feel? Does it make you feel happy? If you need another hint, click the clue button one more time",
                   "Focus on how you are feeling... Does the song make you feel happy? Click the clue button again, for another hint" ]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        # If saying is the same, as the one previously used, re-randomise
        while saying == previous_saying:
            ind = random.randint(0, len(sayings) - 1)
            saying = sayings[ind]
        return saying

    def get_GTM_phasetwo_help(self, mood, previous_saying=""):
        """ Sentences QT will say when giving phase 2 help """
        happy_sayings = ["This song makes me want to dance",
                         "I feel so cheerful listening to this song",
                         "I would like to listen to this song on a sunny day",
                         "This song makes me want to bust a move!"]
        sad_sayings = ["This song makes me think about old times",
                         "I feel a bit down listening to this song",
                         "I would like to listen to this song on a rainy day",
                         "This song doesn't sound very cheerful..."]
        sayings = ["Jwaad, you have mistyped the mood of this song...",
                   "Jwaad, you messed up the mood of the song."]
        if mood == "happy":
            sayings = happy_sayings
        elif mood == "sad":
            sayings = sad_sayings
        else:
            sayings = happy_sayings
            print("TODO ADD THE OTHER EMOTIONS SAYINGS")
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        # If saying is the same, as the one previously used, re-randomise
        while saying == previous_saying:
            ind = random.randint(0, len(sayings) - 1)
            saying = sayings[ind]
        return saying

    #####################################################Level / screen code#################################################################

    def guided_tut(self, run):
        """Code to play tut sequence for Guess the mood"""

        # String of our keys so i can remember them
        """
        0 = title text
        1 = sad button
        2 = happy button
        3 = unsure button
        4 = play button
        """

        # String tut sequence
        """
        1 = Intro
        2 = options (sad happy)
        3 = hint
        4 = play button
        5 = outro
        """

        # Create rect to highlight and text for QT to say
        # Lets try it now: listen to song --> this sounds happy to me. --> lets click "happy" --> highlight happy --> wait for press
        tut_graphics = {
            1: {"rect": None, "keys": [1, 2, 3, 4, 5],
                "speech": "In this game, you will hear some music, and you need to select, whether it was happy, or sad! When you are ready for the next step,, tap the. Next.. button."},
            2: {"rect": (615, 600, 1675, 800), "keys": [2, 3],
                "speech": "These are your options.. Tap happy, if you think the song is happy, or sad if you think the song is sad."},
            3: {"rect": (800, 1400, 2050 - 800, 1850 - 1500), "keys": [4],
                "speech": "If you need a hint, click this button... I will help you out!"},
            4: {"rect": (1235, 180, 400, 400), "keys": [5],
                "speech": "This is the play button.. Use this to stop and start the song..."},
            5: {"rect": None, "keys": [1, 2, 3, 4, 5], "speech": "That is all for guess the mood, have fun!"}
        }

        if self.run:

            # Create buttons and slider
            self.create_graphics()
            self.tut_skip = self.create_button("tut_skip.png", (2600, 0), scale=1, unique_id="skip", should_grey=False)
            self.tut_next = self.create_button("tut_next.png", (0, 0), scale=1.5, unique_id="next", should_grey=False)
            self.tut_repeat = self.create_button("tut_repeat.png", (0, 0), scale=1.5, unique_id="repeat",
                                                 should_grey=False)

            # Group elements
            self.buttons = [self.sad_button, self.happy_button, self.unsure_button, self.play_button]
            tut_buttons = [self.tut_next, self.tut_repeat]

            # Define variables & start track
            qt_finished_talking = False

            # loop through each graphic that we care about
            key = 1  # iter var
            prev_key = key - 1  # to tell us if we've changed to next tut segment
            # Loop through all tut pages
            while key <= len(tut_graphics.keys()):
                # Define some variables for the tut sequence
                tut_key = tut_graphics[key]["keys"]  # draw grey graphics of everything except for our focused graphic
                tut_speech = tut_graphics[key]["speech"]
                tut_rect = tut_graphics[key]["rect"]
                speaking_timer = self.command_manager.qt_say(tut_speech)  # QT should say text out loud
                qt_finished_talking = False
                option_chosen = False  # Hold execution until user clicks on button

                # set logic based on what graphic we focus on
                target_graphics, target_event = self.get_target_behaviour(key)
                repeat_instruction = False
                while not option_chosen and not rospy.is_shutdown() and self.run:

                    # Update all grey graphics
                    grey_graphics = self.update_grey_graphics()

                    # Render graphics
                    self.renderer.DrawBackground(self.background_colour)  # draw background
                    self.load_list_graphics(grey_graphics, tut_key)  # load specified grey objects
                    if target_graphics != []:
                        for graphic in target_graphics:  # Render the target graphic
                            graphic()  # Render graphics each
                    self.animation_manager.DrawTouchAnimation(self.window)  # Draw touch animation
                    if tut_rect != None:
                        arrow_rect = self.renderer.HighlightRect(tut_rect, self.pygame)  # draw arrow and box
                        # since the arrow keeps moving, take it's location once for the next and repeat buttons.
                        if key != prev_key:
                            self.draw_tut_options(arrow_rect)
                            prev_key = key
                    if qt_finished_talking and tut_rect != None:
                        for button in tut_buttons:
                            button.render(self.window)
                    # if there's no box highlighted, draw the buttons in a default pos
                    elif qt_finished_talking and tut_rect == None:
                        self.tut_next.set_pos((1700, 250))
                        self.tut_repeat.set_pos((700, 250))
                        for button in tut_buttons:
                            button.render(self.window)
                    self.tut_skip.render(self.window)

                    # Handle events
                    events = self.pygame.event.get()
                    mouse_pos = self.pygame.mouse.get_pos()
                    for event in events:
                        if event.type == self.pygame.QUIT:
                            self.run = False  # Stops the program entirely
                            self.quit = True  # Tells us that the game was quit out of, and it didn't end organically
                        if event.type == self.inputUp:  # on mouse release play animation to show where cursor is
                            self.animation_manager.StartTouchAnimation(mouse_pos)  # play animation
                        skip = self.tut_skip.get_event(event, mouse_pos)
                        if skip:
                            return self.run
                        # handle tut button events
                        if qt_finished_talking:  # only handle events if buttons being rendered
                            for button in tut_buttons:
                                button_pressed = button.get_event(event, mouse_pos)
                                if button_pressed:
                                    option_chosen = True  # so we can choose when this goes to false
                                if button_pressed:
                                    button_pressed_id = button.id  # get which button was pressed
                                    if button_pressed_id == "repeat":
                                        repeat_instruction = True
                    if target_event != None:
                        target_event(events)  # render the target graphic

                    # Dont check if QT already finished talking
                    if not qt_finished_talking:
                        qt_finished_talking = self.command_manager.robo_timer.CheckTimer(speaking_timer)
                    self.pygame.display.update()  # Update all drawn objects

                # Either replay the same instruction, or move onto the next
                option_chosen = False
                if not repeat_instruction:
                    key += 1

        return self.run

    def play_level(self, run, track_mood, track_name):
        """Sequence plays the levels"""
        if self.run:  # Dont start this screen if the previous screen wanted to close out the game

            # Get the level's data
            self.track_name = track_name

            # Create buttons
            self.create_graphics()

            # Group elements
            self.buttons = [self.sad_button, self.happy_button, self.unsure_button, self.play_button]

            # Define variables & start track
            self.sound_manager.load_track(self.track_name)  # load song to sound player and get data back
            self.track_data = self.get_track_info()  # Get data from the sound_player node for this track and save it
            self.level_complete = False  # Check when level has been finished
            song_ended = False
            answer_given = False
            track_stopped = True  # this makes it play on start
            qt_message = ""
            current_track_time = ""
            track_total_time = ""
            fps = "0"

            self.sound_manager.unpause()  # start track
            music_playing = True
            song_interrupt = False  # track if we stopped song
            slider_was_held = False

            # Start recording time
            start_time = rospy.get_time()
            play_time = 0
            correct_answer = False  # If the answer they gave was correct or not
            hints_given = 0  # How many hints they needed
            full_listen = False  # Tracks if user has heard full song
            clear_type = "fail"

            # Main game loop
            while self.level_complete == False and not rospy.is_shutdown() and self.run:

                # if the song ended, start player to the beginning and pause it.
                if song_ended:
                    self.play_button.its_rewind_time()  # draw rewind symbol
                    self.sound_manager.load_track(self.track_name)
                    music_playing = False
                    song_ended = False
                    full_listen = True

                # Get variables that we will draw onto screen
                current_track_time, track_total_time, progress, song_ended = self.get_song_info(current_track_time,
                                                                                                track_total_time)  # get out some data from the current song playing

                # Draw background and objects
                self.update_graphics()  # draw coloured graphics
                self.rendered_graphics = self.update_grey_graphics()  # save updated grey graphics into attribute
                if self.debug: self.draw_debug_info(fps)  # draw fps onto screen
                self.pygame.display.update()  # Update all drawn objects

                # Start event handling
                events = self.pygame.event.get()

                # start checking if they're stuck after user has listened to the whole song once,
                #if full_listen:
                #    # TODO, check if QT is busy or not, before doing this.
                # TODO CURRENTLY THIS IS COMPLETELY BROKEN. FIX FIX FIX
                #    self.behaviours_manager.qt_reminder(events, music_playing=music_playing)

                for event in events:
                    # reset / init variables
                    option_chosen = ""
                    mouse_pos = self.pygame.mouse.get_pos()
                    # on mouse release play animation to show where cursor is
                    if event.type == self.inputUp:
                        self.animation_manager.StartTouchAnimation(mouse_pos)
                    # Events for pause button this will also return if we're paused or not :
                    music_playing = (self.play_button.get_event(event, mouse_pos))

                    # Check which button is pressed, if any.
                    for button in self.buttons[:-1]:  # For all buttons except the play_button
                        button_pressed = button.get_event(event, mouse_pos)
                        if button_pressed:
                            button_pressed_id = button.id
                            self.sound_manager.pause()  # pause if something is playing
                            if music_playing:
                                song_interrupt = True
                            track_stopped = True
                            # if clicked button is correct
                            if button_pressed_id == track_mood:
                                # print("User has clicked the correct answer")
                                correct_answer = True
                                answer_given = True

                            # if clicked button is unsure --> give hint
                            elif button_pressed_id == "unsure":
                                # print("User has clicked unsure")
                                hints_given += 1
                                if hints_given == 1:
                                    # Phase 1 hint, reminder of what to do
                                    self.command_manager.send_qt_command(emote="talking", gesture="explain_right")
                                    qt_message = self.behaviours_manager.get_help() + self.get_GTM_phaseone_help()
                                    self.level_loader.QTSpeakingPopupScreen(qt_message, self.rendered_graphics,
                                                                            self.run,
                                                                            self.background_colour)  # this is blocking
                                else:
                                    # Phase 2, suggestive language
                                    self.command_manager.send_qt_command(emote="talking", gesture="explain_right")
                                    qt_message = self.behaviours_manager.get_help() + self.get_GTM_phasetwo_help(track_mood)
                                    self.command_manager.send_qt_command(gesture="nod")
                                    self.level_loader.QTSpeakingPopupScreen(qt_message, self.rendered_graphics, self.run,
                                                                            self.background_colour) # this is blocking
                                # TODO maybe add 3 phase, where if again clue = finish level as fail

                            # Even if answer is wrong, move on
                            elif button_pressed_id != track_mood:
                                correct_answer = False
                                answer_given = True

                            if song_interrupt:  # if we had paused the music, resume it
                                self.sound_manager.unpause()
                                song_interrupt = False

                    # Check if the user clicks the X
                    if event.type == self.pygame.QUIT:
                        self.run = False  # Stops the program entirely
                        self.quit = True  # Tells us that the game was quit out of, and it didn't end organically
                        self.level_complete = True  # end level
                        self.sound_manager.stop_track()  # Stop the music

                if self.debug:
                    fps = self.get_fps()  # calculate FPS of this loop and show it next loop

                # Check if level won
                if answer_given:
                    # Record stats, and give message based on clear type
                    play_time = rospy.get_time() - start_time
                    clear_type = self.compute_clear_type(play_time, correct_answer, hints_given)
                    qt_message = self.qt_reward(clear_type)
                    self.level_loader.QTSpeakingPopupScreen(qt_message, self.rendered_graphics, self.run,
                                                            self.background_colour)  # this is blocking
                    # End level
                    self.level_complete = True
                    print("Ending level")

            # Ending sequence after while loop
            if self.quit:
                print("You have quit the game.")
            else:
                print("You completed the level.")

            self.sound_manager.stop_track()
            mood_selected = button_pressed_id
            e_clear_req = {"time_taken": self.total_track_secs + self.e_clear_time, "mood":track_mood}
            performance ={"time_taken": play_time, "mood":mood_selected, "num_hints":hints_given}
            level_data = {"game_name":"GTM", "clear_type":clear_type, "performance":performance, "e_clear_req": e_clear_req}
            return self.run, level_data

    #################################################################Main####################################################################

    def Main(self, track_mood, track_name, ask_tut = True):  # input what level and difficulty to play, the program will handle the rest
        """
        Main loop of guess the mood.
        Runs relevant build up screens, aswell as handles all the in game logic
        track_mood = string, mood of the song that will be played in this level
        track_name = string, file name of the song that will be played
        ask_tut = bool, whether to ask if the tut should be played.

        Returns:
            self.run = bool, whether we tried to quit or not during the game
            level_data, = {"performance":performance, "e_clear_req": e_clear_req}, dict, performamce = time taken,
                mood chosen, hints needed. e_clear_req

        """
        # Show starting screen
        #self.run = self.level_loader.QTSpeakingScreen("Lets play Guess the mood!", self.run, self.background_colour, )

        if ask_tut:
            # Ask if they want tutorial
            self.run, tut = self.level_loader.yes_or_no_screen('Should I explain how to play "Guess The Mood" ?', self.run,
                                                               self.background_colour)
            if tut:
                self.run = self.guided_tut(self.run)

        # Tap to continue screen to slow pacing
        self.run = self.level_loader.tap_to_continue(self.run, self.background_colour)

        # Countdown
        # self.run = self.level_loader.countdown(3, self.run, self.background_colour, prelim_msg="Get ready to play!")

        # Run game code
        self.run, level_data = self.play_level(self.run, track_mood, track_name)

        # Save user data
        return self.run, level_data

    ######################################################On execution#######################################################################


# If we run this node, run the game on it's own
if __name__ == '__main__':
    # Initialise game
    rospy.init_node('guess_the_mood_game', anonymous=False)
    rospy.loginfo("Node launched successfully")
    game_object = Guess_The_Mood_Game()

    # Run the game
    try:
        #game_object.Main("happy", "happy_3.wav")
        game_object.guided_tut(True)
    except(KeyboardInterrupt or rospy.exceptions.ROSInterruptException):
        game_object.pygame.quit()
        game_object.sound_manager.stop_track()
        print("Audio may not be stopped due to interrupt")
