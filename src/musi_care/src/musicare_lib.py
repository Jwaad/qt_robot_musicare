
# !/usr/bin/env python
#
# Library of useful classes that the other games use
#
#
import rospy
import pygame
import pygame.freetype  # some reason fixes events repeating themselves
import os
import math
import random
import wave
import contextlib
import pickle
from pydub import AudioSegment
import numpy as np
from musi_care.msg import SongData
from musi_care.srv import sound_player_srv
from musi_care.srv import qt_command
import threading
from qt_motors_controller.srv import set_control_mode, set_velocity
from qt_robot_interface.srv import setting_setVolume
from std_msgs.msg import Float64MultiArray
import cv2
import time

# rospy.init_node('musicare_lib', anonymous=False)
# rospy.loginfo("musicare_lib launched successfully")

######################################################Uncategorised#################################################################
class General():
    """
    Class for methods that are hard to / are too few to categorise.
    """

    def __init__(self):
        pass

    def convert_to_grey(self, image):
        image_grey = pygame.transform.grayscale(image)  # .convert_alpha()
        return image_grey

    def Load_Song_Data(self, game, verbose = False):
        """
        game = string of the game name options: gtm, fts, ctb, ssc
        """
        # Load the save data in full
        this_path = os.path.dirname(__file__)
        save_name = "song_database.p"
        save_path = os.path.join(this_path, 'game_assets/data/', save_name)
        song_data = {}
        with open(save_path, "rb") as f:
            if verbose:
                print("attempting to open {0}".format(save_path))
            song_data = pickle.load(f)

        easy = {}
        medium = {}
        hard = {}

        for song_name, song_data in song_data.items():
            if song_data[game] == "easy":
                easy[song_name] = song_data
            elif song_data[game] == "medium":
                medium[song_name] = song_data
            elif song_data[game] == "hard":
                hard[song_name] = song_data

        level_data = {"easy":easy, "medium":medium, "hard":hard}
        return level_data


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Threading~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
class ThreadWithReturnValue()  :  # Thread):

    # TODO come back to this, and give thread objects the functionality to return data, and be stopped from the main
    # TODO possibly self.run within this class
    # https://stackoverflow.com/questions/323972/is-there-any-way-to-kill-a-thread

    def __init__(self, group=None, target=None, name=None, args=(), kwargs={}, Verbose=False):
        Thread.__init__(self, group, target, name, args, kwargs)
        self._return = None
    def run(self):

        if self._target is not None:
            self._return = self._target(*self._args ,**self._kwargs)

    def join(self, *args):
        Thread.join(self, *args)
        return self._return



######################################################DraggableButton#################################################################

class Behaviours():
    """Class to store and return saved / repeatable behaviours other than builtin with QT
        When using methods to get randomised frequent messages, you can pass in the previous message to
        ensure there's no repeat.
    """

    def __init__(self, pygame, path_to_music, inputMode = 1):
        self.timeout_started = False
        self.timer = TimeFunctions()
        self.sound_manager = SoundManager(path_to_music)
        self.command_manager = QTManager()
        self.pygame = pygame
        self.paused = False
        self.speaking_timer_id = ""

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


    def qt_reminder(self, events, music_playing=True, timeout_message=None):
        """If x seconds of no inputs pass, qt should pause music and say something"""
        if timeout_message == None:
            timeout_message = self.get_refocus()
        timer_id = "timeout"
        timeout_time = 7  # give some seconds until timeout
        # If first run of this method, start timer
        if not self.timeout_started:
            self.timer.CreateTimer(timer_id, timeout_time, verbose=False)
            self.talking = False
            self.timeout_started = True
        if self.timer.CheckTimer(timer_id):
            if music_playing and not self.talking:
                self.sound_manager.pause()
                self.talking = True
            self.speaking_timer_id = self.command_manager.qt_say(timeout_message)
        else:
            for event in events:
                if event.type == self.inputDown:
                    print("starting timer")
                    self.timer.CreateTimer(timer_id, timeout_time, verbose=False)  # restart timer
                    self.talking = False
                    # TODO change this so instead of new timer each click, we save time at click.
                    #       and check if time from last click > timeout_time
        # check if we're still paused and if QT is still speaking
        if self.talking and music_playing:
            if self.command_manager.robo_timer.CheckTimer(self.speaking_timer_id):
                self.talking = False
                self.sound_manager.unpause()
                self.timeout_started = False  # so that we start a new timer

    def get_refocus(self, previous_saying=""):
        """ Things QT can say, when bringing user's attention back to game """
        sayings = ["If you are stuck. please click the clue button",
                   "Hey, if you need help, click the. Clue button",
                   "Click the clue button if you need help",
                   "I can help if you need it ! Just click the clue button"]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        # If saying is the same, as the one previously used, re-randomise
        while saying == previous_saying:
            ind = random.randint(0, len(sayings) - 1)
            saying = sayings[ind]
        return saying

    def get_agreements(self, previous_saying=""):  # List of all behaviours_manager when QT agrees EG yes, correct
        sayings = ["Yes!",
                   "Correct!",
                   "That's right!",
                   "That's correct!",
                   "Great, that's right!",
                   "Good job, That is the right answer!"]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        # If saying is the same, as the one previously used, re-randomise
        while saying == previous_saying:
            ind = random.randint(0, len(sayings) - 1)
            saying = sayings[ind]
        return saying

    def get_disagreements(self, previous_saying=""):  # List of all behaviours_manager when QT disagree EG no, sorry.
        sayings = ["No, Please try again...",
                   "Sorry, no, please try again...",
                   "No, that's not correct...",
                   "I'm sorry that is not right, Please try again...",
                   "Sorry, that is not the right answer..."]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        # If saying is the same, as the one previously used, re-randomise
        while saying == previous_saying:
            ind = random.randint(0, len(sayings) - 1)
            saying = sayings[ind]
        return saying

    def get_praise(self, previous_saying=""):
        """ Sayings for when user does very well, relating to answers"""
        sayings = ["Amazing, that is the right answer!",
                   "Great job!",
                   "Alright, that's correct!",
                   "Wow, well done!",
                   "Nice! Well done!",
                   "You did really well on that one!",
                   "You are a natural",
                   "You made that one look easy!",
                   "Wow! You did that perfectly!"]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        # If saying is the same, as the one previously used, re-randomise
        while saying == previous_saying:
            ind = random.randint(0, len(sayings) - 1)
            saying = sayings[ind]
        return saying

    def get_generic_praise(self, previous_saying=""):
        """ Generic sayings for when user does very well, for non question types """
        sayings = ["Amazing !",
                   "Great job!",
                   "Wow, well done!",
                   "Nice! Well done!",
                   "You did really well on that one!",
                   "You are a natural",
                   "You made that one look easy!"]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        # If saying is the same, as the one previously used, re-randomise
        while saying == previous_saying:
            ind = random.randint(0, len(sayings) - 1)
            saying = sayings[ind]
        return saying

    def get_generic_light_praise(self, previous_saying=""):
        """ Level end sayings for when user does okay """
        sayings = ["Good job!",
                   "Great job!",
                   "Alright, well done!",
                   "Well done!",
                   "Nice! you did it!",
                   "You did well on that one!",
                   "You made that one look easy!"]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        # If saying is the same, as the one previously used, re-randomise
        while saying == previous_saying:
            ind = random.randint(0, len(sayings) - 1)
            saying = sayings[ind]
        return saying

    def get_incorrect(self, previous_saying=""):
        """ Gentle ways to move onto next task, if user is not correct"""
        sayings = ["Okay, next question",
                   "On to the next question",
                   "Lets move on",
                   "Lets go the next task" ]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        # If saying is the same, as the one previously used, re-randomise
        while saying == previous_saying:
            ind = random.randint(0, len(sayings) - 1)
            saying = sayings[ind]
        return saying

    def get_next_level(self, previous_saying=""):
        """ Different ways to move onto the next task"""
        sayings = ["Okay, next level",
                   "On to the next level",
                   "Lets play the next level!",
                   "Now onto the next one!",
                   "Lets play another one!"]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        # If saying is the same, as the one previously used, re-randomise
        while saying == previous_saying:
            ind = random.randint(0, len(sayings) - 1)
            saying = sayings[ind]
        return saying

    def get_start_games(self, previous_saying=""):
        """ Different ways to move onto the next task"""
        sayings = ["Then! Lets start playing some games!",
                   "I can't wait anymore, lets start playing!",
                   "Lets start playing. I hope you're ready!",
                   "Okay. Now.. lets play!",
                   "Get ready, i am going to start the games now!"
                   ]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        # If saying is the same, as the one previously used, re-randomise
        while saying == previous_saying:
            ind = random.randint(0, len(sayings) - 1)
            saying = sayings[ind]
        return saying

    def get_closing_remark(self, previous_saying=""):
        """ Sentences QT can say, before giving a hint """
        sayings = ["We are all done with the games today. That was fun!",
                   "Phew, We're done for today. That was so much fun",
                   "We have finished... I had a great time playing today.",
                   "That was really fun. We're all out of games for today",
                   "We're done for the day. I enjoyed myself so much!"]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        # If saying is the same, as the one previously used, re-randomise
        while saying == previous_saying:
            ind = random.randint(0, len(sayings) - 1)
            saying = sayings[ind]
        return saying

    def get_goodbye(self, previous_saying=""):
        """ Sentences QT can say, before giving a hint """
        sayings = ["I had so much fun today! I look forward to next week",
                   "I wish we could play forever, but i have to rest now. See you next week !",
                   "I really enjoyed hanging out with you! I can't wait to see you next week!",
                   "Bye Bye. I'm really excited to see you next week",
                   "Goodbye for now! See you next week!"
                   ]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        # If saying is the same, as the one previously used, re-randomise
        while saying == previous_saying:
            ind = random.randint(0, len(sayings) - 1)
            saying = sayings[ind]
        return saying

    def get_help(self, previous_saying=""):
        """ Sentences QT can say, before giving a hint """
        sayings = ["Okay! I will try my best to help!",
                   "I will give you a hint",
                   "I can help!" ]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        # If saying is the same, as the one previously used, re-randomise
        while saying == previous_saying:
            ind = random.randint(0, len(sayings) - 1)
            saying = sayings[ind]
        return saying

    def get_next_difficulty(self, previous_saying=""):
        """ Sentences QT can say, before increasing difficulty """
        sayings = ["You have done so well on those last games! I am going to make it harder now, hehe",
                   "Wow! I'm so impressed, you made such short work of those levels, the next level will have to be harder!",
                   "This is way too easy for you. The next levels of this game will be harder ! Be ready !",
                   "You are too good at this. I am increasing the difficulty now. The next game will be harder"
                   ]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        # If saying is the same, as the one previously used, re-randomise
        while saying == previous_saying:
            ind = random.randint(0, len(sayings) - 1)
            saying = sayings[ind]
        return saying

    def get_greetings(self, previous_saying=""):
        """ This QT should say at the start of sessions 2 - n """
        sayings = ["It's been so long... Hello again!",
                   "Hey! I've missed you.",
                   "It has been too long since I've seen you!",
                   "Hello again, it feels like it's been forever since i've seen you.",
                   "Hello again, lets have fun this week too!",
                   "Are you ready to have fun? I am!",
                   "Can you believe it's been so long since i have seen you?",
                   "I am so happy to see you again!"
                   ]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        # If saying is the same, as the one previously used, re-randomise
        while saying == previous_saying:
            ind = random.randint(0, len(sayings) - 1)
            saying = sayings[ind]
        return saying

    def get_how_was_song(self, previous_saying= ""):
        """ Different ways QT can ask if you liked the song """
        sayings = ["Did you like the song?",
                   "How much did you like that song?",
                   "What do you think of the song you just heard?",
                   "Did you enjoy that song?",
                   "Tell me how much you liked that song",
                   "Did you enjoy listening to that song?"
                   ]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        # If saying is the same, as the one previously used, re-randomise
        while saying == previous_saying:
            ind = random.randint(0, len(sayings) - 1)
            saying = sayings[ind]
        return saying
    
    def get_transition_to_question(self, previous_saying= ""):
        """ Different ways QT can ask how are you"""
        sayings = ["I am really curious though...",
                   "I think i already asked, but,",
                   "I know i've asked this before, but...",
                   "I really want to know",
                   ]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        # If saying is the same, as the one previously used, re-randomise
        while saying == previous_saying:
            ind = random.randint(0, len(sayings) - 1)
            saying = sayings[ind]
        return saying
    def get_how_are_you(self, previous_saying= ""):
        """ Different ways QT can ask how are you"""
        sayings = ["How are you today?",
                   "How are you?",
                   "How are you doing today?",
                   "How have you been?",
                   "How do you feel today",
                   "Please tell me how you are feeling today",
                   "Tell me how you feel today"
                   ]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        # If saying is the same, as the one previously used, re-randomise
        while saying == previous_saying:
            ind = random.randint(0, len(sayings) - 1)
            saying = sayings[ind]
        return saying

#####################################################General Levels##################################################################


class StandardLevels():
    """Class to draw basic screens multiple games use, such as yes or no screen """

    def __init__(self, window, window_center, pygame, path_to_music, debug = False, inputMode = 1):
        self.window = window
        self.window_center = window_center
        self.pygame = pygame
        self.renderer = Renderer(window, window_center)
        self.animation_manager = AnimationManager(pygame)
        self.command_manager = QTManager(debug = debug)
        self.sound_manager = SoundManager(path_to_music)
        self.behaviour_manager = Behaviours(pygame, path_to_music)
        self.timer = TimeFunctions()
        self.path_to_imgs = 'game_assets/graphics'
        self.this_file_path = os.path.dirname(__file__)
        self.path_to_music = path_to_music
        rospy.wait_for_service('/sound_player_service')
        self.sound_player = rospy.ServiceProxy('/sound_player_service', sound_player_srv, persistent=True)

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

    def MeasureBPM(self, song_database, save_function, run=True, background_colour=(100, 100, 100)):
        """
        Method that plays the song and then measures the sound of user clapping.
        A BPM is calculated from the 5 second recording.
        You can then slide the BPM left and right, to get the first beat.
        Clicking Save will directly overwrite the data in the song_database
        You can also scroll left and right, to move from one song_entry to another
        song_data = dict, database in format {song_title: {"file_name":"filename", bpm:bpm, etc:etc }}
        save_function = func, pass in what function to run, when clicking the save button
        """
        def BPMFromHits(hits):
            """ Using times of each beat, calculate the bpm"""
            #Generate a new list of time differences per beat
            time_gaps = []
            for i in range(0, len(hits) - 2): # -2 since we dont want to loop at len
                time_gap = hits[i+1] - hits[i]
                time_gaps.append(time_gap)
            if len(time_gaps) < 2:
                return 0.0, 999
            avg_time_gap = np.mean(time_gaps)
            bpm = 60 / avg_time_gap
            bpm = int(round(bpm,0))
            first_beat = hits[0]
            return first_beat, bpm

        def get_beats_list(bpm, track_len, first_beat):
            bps = float(60 / bpm)
            total_beats = math.floor(((track_len - first_beat) / bps) + 1)  # +1 to include first beat
            beat_timings = np.zeros([total_beats], dtype=float)
            beat_timings[0] = float(first_beat)
            # loop through and populate list of beat timings, skipping the first
            for i in range(beat_timings.shape[0]-1):
                beat_timings[i + 1] = beat_timings[i] + (1 / bps)

            #print(beat_timings)
            return beat_timings

        def generateBeatMarkers(rect, bpm, first_beat, track_total_time):
            """
            function for method measure BPM.
            Function takes in a rect, bpm and the time of first beat, then evenly distributes
            lines based on the BPM and first beat.
            returns a list of line objects
            """
            # Get measure of the space to draw lines onto
            x, y, w, h = rect

            # Generate beats as a percentage of the track time
            bps = 60 / bpm
            total_beats = int((track_total_time - first_beat) / bps) + 1 # +1 to include first beat
            beat_timings = np.zeros([total_beats],dtype=float)

            beat_time = first_beat # time between each beat in secs
            for i in range(0, beat_timings.size):
                beat_percent = beat_time / track_total_time
                beat_timings[i] = beat_percent
                beat_time += bps # time between each beat in secs
            #print("printing beat times", beat_timings)

            lines = []
            for beat in beat_timings:
                line = LineObject((x + (w * beat), y), (x + (w * beat), y + h), width = 5 )
                lines.append(line)

            print("This song has a bpm of {}, is {}s long, and has {} beats. First beat is at {}s".format(bpm, track_total_time, beat_timings.size , round(first_beat,3) ))
            return lines

        if run:  # Don't start this screen if the previous screen wanted to close out the game

            # List of song_titles
            song_titles = list(song_database.copy().keys())
            #print(song_titles)

            # Create buttons
            path_to_png = os.path.join(self.this_file_path, self.path_to_imgs)
            prev_song_button = Button(path_to_png + "/tut_back.png", (925, 150), self.pygame, scale=1, return_info="prev")
            next_song_button = Button(path_to_png + "/tut_next.png", (1625, 150), self.pygame, scale=1, return_info="next")
            record_button = Button(path_to_png + "/record_button.png", (1300, 100), self.pygame, scale=1, return_info="record")
            play_button = ToggleButton(path_to_png + "/play_button.png", path_to_png + "/pause_button.png", (300, 950), self.pygame, scale=1, return_info="play")
            save_button = Button(path_to_png + "/blank_button.png", (1150, 1550), self.pygame, scale=0.5, text="save",
                                 return_info="save")

            buttons = [next_song_button, prev_song_button, record_button, save_button, play_button]

            lines = []

            song_ind = 0

            # Init metronome sound
            pygame.mixer.init()
            metronome = pygame.mixer.Sound(self.path_to_music + "metronome.wav")

            # Keep looping through all the data, when they click a button
            while not rospy.is_shutdown() and run:
                #print(song_ind)
                # Get song title and data
                song_title = song_titles[song_ind]
                song_data = song_database[song_title]

                # Try to read track data up to 3 times, if we got a bad read
                i = 0
                track_len = 999
                while track_len > 900 and i < 3:
                    track_stats = self.sound_manager.load_track(("/game_assets/music/" + song_data["file_name"]), 0.0)
                    track_len = track_stats.track_total_time
                    i += 1

                # Create title and labels for emotions
                title_text = TextObject(self.window, self.window_center, song_title, location=(0, 500), font_size=150,
                                        cen_x=True)
                text_objs = [title_text]

                # Generate wav_image todo
                song_waveform = Button(path_to_png + "/blank_button.png", (625, 800), self.pygame, scale=1.5)

                #Generate beat markers based off of data
                bpm = float(song_data["bpm"]) # Calculated as avg time between claps
                first_beat = float(song_data["first_beat"])  # time that the user started clapping
                wav_rect = song_waveform.get_rect()
                time.sleep(0.3)
                lines = generateBeatMarkers(wav_rect, bpm, first_beat, track_total_time=track_len)

                # Reset vars for the loop
                change_song = False

                # Vars for recording claps / space hits
                self.recording = False
                self.playing_track = False
                song_beats = []# generate beats list
                hits = []
                t0 = 0
                tx = 0
                temp = []
                song_snippet_time = 7 # how long the song should play for

                # While loop until they click next or prev
                while not change_song and not rospy.is_shutdown() and run:

                    # So we can use events twice seperately
                    events = self.pygame.event.get()

                    # handle GUI when recording
                    if self.recording:
                        # Record claps
                        for event in events:
                            if event.type == pygame.KEYDOWN:
                                if event.key == pygame.K_SPACE:
                                    hits.append(rospy.get_time() - t0)
                        # check if time up
                        if (rospy.get_time() - t0) > song_snippet_time:
                            self.recording = False
                            self.sound_manager.stop_track()  # Stop the music
                            first_beat, bpm = BPMFromHits(hits)
                            song_data["first_beat"] = first_beat
                            song_data["bpm"] = bpm
                            lines = generateBeatMarkers(wav_rect, bpm, first_beat, track_total_time=track_len)
                            song_database[song_title] = song_data


                    # Metronome playing logic
                    if self.playing_track:
                        # TODO TEMP
                        print("fps =", 1 / (rospy.get_time() - tx))
                        tx = rospy.get_time()
                        # TODO TEMP
                        next_beat = song_beats[0]
                        if (rospy.get_time() - t0) > next_beat:
                            # Play metronome sound on each beat
                            metronome.play()
                            temp.append(rospy.get_time())
                            song_beats = np.delete(song_beats, 0)
                        if song_beats.shape[0] == 0:
                            print(temp, temp2)
                            self.playing_track = False
                            play_button.toggle_toggle() # Toggle back off

                    # Handle events
                    for event in events:
                        # Check if the user clicks the X
                        if event.type == self.pygame.QUIT:
                            run = False
                        mouse_pos = self.pygame.mouse.get_pos()
                        if event.type == self.inputUp:
                            self.animation_manager.StartTouchAnimation(mouse_pos)
                        # if any button is pressed, return it's value
                        for button in buttons:
                            pressed = button.get_event(event, mouse_pos)
                            if pressed:
                                if button.get_info() == "record":
                                    if not self.recording:
                                        self.sound_manager.start_track(("/game_assets/music/" + song_data["file_name"]),
                                                                      0.0)
                                        self.recording = True
                                        hits = []
                                        t0 = rospy.get_time()
                                    else:
                                        self.sound_manager.stop_track()
                                        self.recording = False
                                # Ignore all button presses when recording, just in case
                                if not self.recording:
                                    if button.get_info() == "play":
                                        # Stop playing and reload track to 0
                                        if self.playing_track:
                                            self.playing_track = False
                                            play_button.toggle_off() # Toggle off
                                            self.sound_manager.stop_track()
                                        else:
                                            # If not already playing, then start the song and get the beats ready for metronome
                                            song_beats = get_beats_list(song_data["bpm"], track_len,
                                                                        song_data["first_beat"])
                                            self.playing_track = True
                                            self.sound_manager.start_track(
                                                ("/game_assets/music/" + song_data["file_name"]),
                                                0.0)
                                            t0 = rospy.get_time()
                                            # todo temp
                                            temp2 = song_beats.copy()
                                    else:
                                        # If any other button is pressed, stop the track playing
                                        if self.playing_track:
                                            self.sound_manager.stop_track()
                                            self.playing_track = False
                                            play_button.toggle_off()
                                    if button.get_info() == "next":
                                        change_song = True
                                        song_ind += 1
                                        # Wrap to end
                                        if song_ind >= len(song_titles):
                                            song_ind = 0
                                    elif button.get_info() == "prev":
                                        change_song = True
                                        song_ind -= 1
                                        # Wrap to start
                                        if song_ind < 0 :
                                            song_ind = len(song_titles) - 1
                                    elif button.get_info() == "save":
                                        save_function(song_database)

                    # Draw background and objects
                    self.renderer.DrawBackground(background_colour)
                    for button in buttons:
                        if self.recording:
                            #Grey all except record button
                            if button.get_info() == "record":
                                button.render(self.window)
                            else:
                                button.render(self.window, grey=True)
                        else:
                            button.render(self.window)
                    for text in text_objs:
                        text.render(self.window)
                    song_waveform.render(self.window)
                    for line in lines:
                        line.render(self.window)
                    self.animation_manager.DrawTouchAnimation(self.window)  # Also draw touches
                    self.pygame.display.update()  # Update all drawn objects

            return song_database


    def get_drum(self, run = True, background_colour = (100,100,100)):
        """ 
        QT needs help putting his drum under him.
        This method QT asks for assitsacnec and waits or you to but the drum under him
        """
        # init vars
        arm_up_right = [20, -59.599998474121094, -40.70000076293945]  # motor pos for right arm to be in air
        arm_up_left = [-20, -59.599998474121094, -40.70000076293945]  # motor pos for right arm to be in air
        arm_right_msg = Float64MultiArray()
        arm_left_msg = Float64MultiArray()
        arm_right_msg.data = arm_up_right
        arm_left_msg.data = arm_up_left
        arms_up = '[["right_arm", "left_arm"], [{}, {}]]'.format(arm_up_right, arm_up_left)

        sayings = ["Can you please help me reach my drum?",
                   "Can you put my drum under me please.",
                   "I need help. Please put my drum in my reach",
                   "Help me grab my drum please!",
                   "Can you please grab my drum", 
                   "please, put my drum in my reach so we can keep playing!"
                   ]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]

        # QT say msg
        self.command_manager.qt_say(saying)
        self.command_manager.qt_emote("talking")
        
        # Have QT lift it's arms
        self.command_manager.qt_actuate(arms_up)

        # Ask i drums in place
        self.yes_or_no_screen("Is QT's drum in place?", run, background_colour=background_colour, silent = True)

    def put_away_drum(self, run=True, background_colour=(100, 100, 100)):
        """
        QT needs help putting his drum under him.
        This method QT asks for assitsacnec and waits or you to but the drum under him
        """
        # init vars
        arm_up_right = [20, -59.599998474121094, -40.70000076293945]  # motor pos for right arm to be in air
        arm_up_left = [-20, -59.599998474121094, -40.70000076293945]  # motor pos for right arm to be in air
        arm_right_msg = Float64MultiArray()
        arm_left_msg = Float64MultiArray()
        arm_right_msg.data = arm_up_right
        arm_left_msg.data = arm_up_left
        arms_up = '[["right_arm", "left_arm"], [{}, {}]]'.format(arm_up_right, arm_up_left)

        sayings = ["Can you please help me put away my drum?",
                   "Can you please put my drum away",
                   "Please put my drum out of my reach",
                   "Help me put away my drum please!",
                   "Can you please remove my drum",
                   "please, put my drum away so we can keep playing!",
                   "My arms arent long enough to move my drum... Can you please help me"
                   ]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]

        # QT say msg
        self.command_manager.qt_say(saying)
        self.command_manager.qt_emote("talking")

        # Have QT lift it's arms
        self.command_manager.qt_actuate(arms_up)

        # Ask i drums in place
        self.yes_or_no_screen("Is QT's drum out of the way??", run, background_colour=background_colour, silent=True)

        self.command_manager.qt_say("Thank you very much!")
        self.command_manager.qt_emote("talking")
        self.command_manager.qt_gesture("arms_up")

    def did_you_like_song(self, run = True, on_screen_text = "Did you like the song?", prev_dialogue = "", background_colour = (200, 200, 200)):
        """ 
            Displays 5 point emoji scale. EAch emoji is a clickable button, which has text underneath it,
            aswell as a main question hovering at the top
            returns "run", and a single number, 5 = loved it, 1 = hated it. 3 = neutral.
        """

        if run:  # Don't start this screen if the previous screen wanted to close out the game

            # Ask how was the song
            question = self.behaviour_manager.get_how_was_song(previous_saying=prev_dialogue)
            self.command_manager.qt_emote("talking")  # show mouth moving
            self.command_manager.qt_say(question)  # says text we give it, and starts an internal timer that we can check on
            self.command_manager.qt_gesture("explain_right")

            # Create emoji scale buttons
            path_to_png = os.path.join(self.this_file_path, self.path_to_imgs)
            loved_it_button = Button(path_to_png + "/emoji_5.png", (15, 800), self.pygame, scale=0.9, return_info=5)
            liked_it_button = Button(path_to_png + "/emoji_4.png", (595, 800), self.pygame, scale=0.9, return_info=4)
            neutral_button = Button(path_to_png + "/emoji_3.png", (1180, 800), self.pygame, scale=0.9, return_info=3)
            disliked_it_button = Button(path_to_png + "/emoji_2.png", (1755, 800), self.pygame, scale=0.9, return_info=2)
            hated_it_button = Button(path_to_png + "/emoji_1.png", (2350, 800), self.pygame, scale=0.9, return_info=1)
            buttons = [loved_it_button, liked_it_button, neutral_button, disliked_it_button, hated_it_button]
            
            # Create title and labels for emotions
            title_text =  TextObject(self.window,self.window_center,on_screen_text, location = (0,400), font_size=150, cen_x=True)
            
            loved_it_text = TextObject(self.window,self.window_center, "I Loved it", location= (115, 1350), font_size= 70)
            liked_it_text = TextObject(self.window,self.window_center, "I Liked it", location= (695, 1350), font_size= 70)
            neutral_text = TextObject(self.window,self.window_center, "It was Okay", location= (1230, 1350), font_size= 70)
            disliked_it_text = TextObject(self.window,self.window_center, "I did not like it", location= (1765, 1350),font_size= 70)
            hated_it_text = TextObject(self.window,self.window_center, "I Hated it", location= (2450, 1350), font_size= 70)
            text_objs = [title_text, loved_it_text, liked_it_text, neutral_text, disliked_it_text, hated_it_text]

            chosen_emoji = False
            # While loop until they click an emoji
            while not chosen_emoji and not rospy.is_shutdown() and run:
                # check for quit
                for event in self.pygame.event.get():
                    # Check if the user clicks the X
                    if event.type == self.pygame.QUIT:
                        return False, -1
                    mouse_pos = self.pygame.mouse.get_pos()
                    if event.type == self.inputUp:
                        self.animation_manager.StartTouchAnimation(mouse_pos)  # Tell system to play animation when drawing
                    # if any button is pressed, return it's value
                    for button in buttons:
                        pressed = button.get_event(event, mouse_pos)
                        if pressed:
                            return True, button.get_info()

                # Draw background and objects
                self.renderer.DrawBackground(background_colour)
                for button in buttons:
                    button.render(self.window)
                for text in text_objs:
                    text.render(self.window)
                self.animation_manager.DrawTouchAnimation(self.window)  # Also draw touches
                self.pygame.display.update()  # Update all drawn objects
            return False, -1 # impossible to reach here

    def how_are_you(self, run=True, on_screen_text="How are you today?", prev_dialogue="",
                          background_colour=(200, 200, 200)):
        """
            Displays 5 point emoji scale. EAch emoji is a clickable button, which has text underneath it,
            aswell as a main question hovering at the top
            returns "run", and a single number, 5 = loved it, 1 = hated it. 3 = neutral.
        """

        if run:  # Don't start this screen if the previous screen wanted to close out the game

            # Ask how was the song
            question = self.behaviour_manager.get_how_are_you(previous_saying=prev_dialogue)
            self.command_manager.qt_emote("talking")  # show mouth moving
            self.command_manager.qt_say(question)  # says text we give it, and starts an internal timer that we can check on
            #self.command_manager.qt_gesture("explain_right")

            # Create emoji scale buttons
            path_to_png = os.path.join(self.this_file_path, self.path_to_imgs)
            loved_it_button = Button(path_to_png + "/emoji_5.png", (15, 800), self.pygame, scale=0.9, return_info=5)
            liked_it_button = Button(path_to_png + "/emoji_4.png", (595, 800), self.pygame, scale=0.9, return_info=4)
            neutral_button = Button(path_to_png + "/emoji_3.png", (1180, 800), self.pygame, scale=0.9, return_info=3)
            disliked_it_button = Button(path_to_png + "/emoji_2.png", (1755, 800), self.pygame, scale=0.9,
                                        return_info=2)
            hated_it_button = Button(path_to_png + "/emoji_1.png", (2350, 800), self.pygame, scale=0.9, return_info=1)
            buttons = [loved_it_button, liked_it_button, neutral_button, disliked_it_button, hated_it_button]

            # Create title and labels for emotions
            title_text = TextObject(self.window, self.window_center, on_screen_text, location=(0, 400), font_size=150,
                                    cen_x=True)

            loved_it_text = TextObject(self.window, self.window_center, "Great", location=(165, 1350),
                                       font_size=70)
            liked_it_text = TextObject(self.window, self.window_center, "Good", location=(745, 1350),
                                       font_size=70)
            neutral_text = TextObject(self.window, self.window_center, "Okay", location=(1350, 1350),
                                      font_size=70)
            disliked_it_text = TextObject(self.window, self.window_center, "Not Good", location=(1840, 1350),
                                          font_size=70)
            hated_it_text = TextObject(self.window, self.window_center, "Terrible", location=(2475, 1350),
                                       font_size=70)
            text_objs = [title_text, loved_it_text, liked_it_text, neutral_text, disliked_it_text, hated_it_text]

            chosen_emoji = False
            # While loop until they click an emoji
            while not chosen_emoji and not rospy.is_shutdown() and run:
                # check for quit
                for event in self.pygame.event.get():
                    # Check if the user clicks the X
                    if event.type == self.pygame.QUIT:
                        return False, -1
                    mouse_pos = self.pygame.mouse.get_pos()
                    if event.type == self.inputUp:
                        self.animation_manager.StartTouchAnimation(
                            mouse_pos)  # tell system to play animation when drawing
                    # if any button is pressed, return it's value
                    for button in buttons:
                        pressed = button.get_event(event, mouse_pos)
                        if pressed:
                            return True, button.get_info()

                # Draw background and objects
                self.renderer.DrawBackground(background_colour)
                for button in buttons:
                    button.render(self.window)
                for text in text_objs:
                    text.render(self.window)
                self.animation_manager.DrawTouchAnimation(self.window)  # Also draw touches
                self.pygame.display.update()  # Update all drawn objects
            return False, -1  # impossible to reach here
        
    def yes_or_no_screen(self, text, run, background_colour, silent = False):
        """Screen for Yes or No questions"""
        # Variables
        this_file_path = self.this_file_path
        path_to_imgs = self.path_to_imgs
        yes_img_path = os.path.join(this_file_path, path_to_imgs, "Yes_button.png")
        no_img_path = os.path.join(this_file_path, path_to_imgs, "No_button.png")

        # Create buttons and text
        yes = Button(yes_img_path, (225, 600), self.pygame, scale=2.2 ,should_grey=False)
        no = Button(no_img_path, (1625, 600), self.pygame, scale=2.2 ,should_grey=False)
        text_obj = TextObject(self.window, self.window_center, text, wrap_text=True, location=(0 ,200), cen_x=True,
                              cen_y=False, font_size=150, font_colour=(255, 255, 255))

        # Have QT say the text out loud if not silent mode
        if not silent:
            self.command_manager.qt_emote("talking")
            self.command_manager.qt_say(text)

        while not rospy.is_shutdown() and run:

            # Event handling
            for event in self.pygame.event.get():  # Check if the user clicks the X
                if event.type == self.pygame.QUIT:
                    return False, False
                elif (
                        event.type == self.inputUp):  # on mouse release play animation to show where cursor is
                    mouse_pos = self.pygame.mouse.get_pos()
                    clicked_yes = yes.get_event(event, mouse_pos)
                    if clicked_yes:
                        return True, True  # returns error and button pressed
                    else:
                        clicked_no = no.get_event(event, mouse_pos)
                        if clicked_no:
                            return True, False
                    self.animation_manager.StartTouchAnimation(mouse_pos)  # tell system to play animation when drawing

            # Draw graphics
            self.renderer.DrawBackground(background_colour)
            yes.render(self.window)
            no.render(self.window)
            text_obj.render(self.window)
            self.animation_manager.DrawTouchAnimation(self.window)  # also draw touches
            self.pygame.display.update()  # Update all drawn objects

    def QTSpeakingScreen(self, qt_say, run, background_colour, should_gesture=False, gesture="explain_right", rand_gest = True, on_screen_text = "Please listen to QT robot"):
        """Method displays background and text in centre"""

        if run:  # Don't start this screen if the previous screen wanted to close out the game

            self.command_manager.qt_emote("talking")  # show mouth moving
            speaking_timer_id = self.command_manager.qt_say(
                qt_say)  # says text we give it, and starts an internal timer that we can check on

            qt_speaking = True  # used to tell us when to stop blocking
            if should_gesture and not (gesture == "" or gesture == None):
                self.command_manager.qt_gesture(gesture)

            while qt_speaking and not rospy.is_shutdown() and run:
                # check for quit
                for event in self.pygame.event.get():
                    # Check if the user clicks the X
                    if event.type == self.pygame.QUIT:
                        return False
                    elif (event.type == self.inputUp):
                        mouse_pos = self.pygame.mouse.get_pos()
                        self.animation_manager.StartTouchAnimation(
                            mouse_pos)  # tell system to play animation when drawing

                # Draw background and objects
                self.renderer.DrawBackground(background_colour)
                self.renderer.DrawTextCentered(on_screen_text, font_size=70)
                self.animation_manager.DrawTouchAnimation(self.window)  # Also draw touches
                self.pygame.display.update()  # Update all drawn objects

                if self.command_manager.robo_timer.CheckTimer(speaking_timer_id):  # If our timer is done
                    qt_speaking = False
                    return True

    def QTSpeakingPopupScreen(self, qt_say, graphics, run, background_colour, should_gesture=False,
                              gesture="explain_right", partial_func = True, rand_gest = True):
        """
           Method displays all our gui with a popup in front with text in centre
           graphics = a dict of functions that are saved with the parameters needed to render them
           can work with (to be archived method) saved particial functions.
           or a list of ojbects with the "render" method
        """
        # Variables
        this_file_path = os.path.dirname(__file__)
        path_to_imgs = 'game_assets/graphics'
        popup_path = os.path.join(this_file_path, path_to_imgs, "loading_screen_button.png")
        popup_path_grey = os.path.join(this_file_path, path_to_imgs, "loading_screen_button_depressed.png")

        text_display = "Please listen to QT robot"

        if graphics == {}:
            print("error: no graphics inputted into popup function")
            return

        if run and not rospy.is_shutdown():  # Dont start this screen if the previous screen wanted to close out the game

            # create popup button in center
            popup = Button(popup_path, (700, 550), self.pygame, scale=1.5)
            popup.rect.center = self.window_center

            self.command_manager.qt_emote("talking")  # show mouth moving
            speaking_timer_id = self.command_manager.qt_say(
                qt_say)  # says text we give it, and starts an internal timer that we can check on
            qt_speaking = True  # used to tell us when to stop blocking
            if should_gesture:
                self.command_manager.qt_gesture(gesture)

            while qt_speaking and not rospy.is_shutdown() and run:
                # Track mouse button presses
                for event in self.pygame.event.get():
                    # Check if the user clicks the X
                    if event.type == self.pygame.QUIT:
                        return False
                    elif (
                            event.type == self.inputUp):  # on mouse release play animation to show where cursor is
                        mouse_pos = self.pygame.mouse.get_pos()
                        self.animation_manager.StartTouchAnimation(
                            mouse_pos)  # tell system to play animation when drawing

                # Draw background and objects
                self.renderer.DrawBackground(background_colour)

                if partial_func:
                    for key in graphics:  # run each partial function
                        graphics[key]()  # run as func
                else:
                    for graphic in graphics:
                        graphic.render(self.window, grey = True)

                popup.render(self.window)
                self.renderer.DrawTextCentered("Please listen to QT robot", font_size=70)
                self.animation_manager.DrawTouchAnimation(self.window)  # also draw touches
                self.pygame.display.update()  # Update all drawn objects

                if self.command_manager.robo_timer.CheckTimer(
                        speaking_timer_id):  # If our timer's internal timer is done
                    qt_speaking = False  # unessecary but might as well
                    return True

    def tap_to_continue(self, run, background_colour,
                        text_display="Please tap the screen when you are ready to start.", qt_say=None,
                        should_gesture=False, gesture="explain_right" ,rand_gest = True):
        """Screen that waits until tap, non-blocking even if QT speaking"""

        if run:  # Dont start this screen if the previous screen wanted to close out the game

            # Handle speaking
            if qt_say != None:
                self.command_manager.qt_emote("talking")  # show mouth moving
                speaking_timer_id = self.command_manager.qt_say(qt_say)

            # Handle gesturing
            if should_gesture:
                self.command_manager.qt_gesture(gesture)

            font_size = 200
            clicked = False
            text_ob = TextObject(self.window, self.window_center, text_display, wrap_text=True, cen_x=True,
                                 cen_y=True, font_size=font_size, font_colour=(255, 255, 255))
            while not clicked and not rospy.is_shutdown() and run:
                # Check for quit
                for event in self.pygame.event.get():
                    # Check if the user clicks the X
                    if event.type == self.pygame.QUIT:
                        return False
                    # On mouse release play animation to show where cursor is
                    elif (event.type == self.inputUp):
                        clicked = True
                        # Tell system to play animation when drawing
                        self.animation_manager.StartTouchAnimation(self.pygame.mouse.get_pos())
                # Draw background and objects
                self.renderer.DrawBackground(background_colour)
                speed_coefficient = 1  # half sine freq
                dot_decider = math.sin(speed_coefficient * rospy.get_time())
                # Change the text of the image depending on the time
                if dot_decider < -1 / 3:
                    display = text_display
                    text_ob.set_text(display)
                elif dot_decider < 1 / 3:
                    display = text_display + "."
                    text_ob.set_text(display)
                else:
                    display = text_display + ".."
                    text_ob.set_text(display)
                text_ob.render(self.window)
                self.animation_manager.DrawTouchAnimation(self.window)  # Also draw touches
                self.pygame.display.update()  # Update all drawn objects

            return True

    def countdown(self, seconds, run, background_colour, prelim_msg=None):
        """Code that counts down to start next screen.
        Use prelim msg to have a msg readout before the count down"""

        if run:  # Dont start this screen if the previous screen wanted to close out the game

            if prelim_msg != None:
                seconds += 1  # add another screen

            for second in range(seconds, -1, -1):  # countdown from seconds to 0
                if prelim_msg != None and second == seconds:  # if this is the 1st loop
                    qt_speech = prelim_msg
                    self.command_manager.qt_say(qt_speech)
                    timer_id = "QT_SAY"
                else:
                    if second == 0:  # if last countdown
                        qt_speech = "Go!"
                        timer_id = self.timer.CreateTimer("wait_time", 0.6)  # wait for shorter time after go
                    else:
                        qt_speech = str(second)  # convert second to string and add !
                        timer_id = self.timer.CreateTimer("wait_time",
                                                          1.3)  # wait for 1.3 seconds to let qt catch up and label the timer

                    self.command_manager.send_qt_command(speech=(qt_speech))  # QT should read out the second

                second_passed = False

                # For each second passed render the screen
                while not second_passed and not rospy.is_shutdown():
                    # Check for quit
                    for event in self.pygame.event.get():
                        if event.type == self.pygame.QUIT:  # Check if the user clicks the X
                            return False
                        elif (
                                event.type == self.inputUp):  # on mouse release play animation to show where cursor is
                            self.animation_manager.StartTouchAnimation(
                                self.pygame.mouse.get_pos())  # tell system to play animation when drawing
                    # Draw background and objects
                    self.renderer.DrawBackground(background_colour)
                    if prelim_msg != None and second == seconds:
                        self.renderer.DrawTextCentered(qt_speech, font_size=100)
                    else:
                        self.renderer.DrawTextCentered(qt_speech, font_size=500, font_colour=(150, 250, 150))

                    self.animation_manager.DrawTouchAnimation(self.window)  # Also draw touches
                    self.pygame.display.update()  # Update all drawn objects

                    if prelim_msg != None and second == seconds:  # if 1st loop, check a different timer
                        second_passed = self.command_manager.robo_timer.CheckTimer(timer_id)
                    else:
                        second_passed = self.timer.CheckTimer(timer_id)

            return True


    def blocking_fade(self, run, background_colour, fade_time, message, speak_message = False, gesture = ""):
        """ blank screen with text in middle that slowly fades to grey """

        if run:  # Dont start this screen if the previous screen wanted to close out the game

            # Handle speaking
            if speak_message:
                self.command_manager.qt_emote("talking")  # show mouth moving
                speaking_timer_id = self.command_manager.qt_say(
                    message)  # says text we give it, and starts an internal timer that we can check on
            # Handle gesturing
            if gesture != "":
                self.command_manager.qt_gesture(gesture)
            # Create text object
            text_object = TextObject(self.window, self.window_center, message , cen_x=True, cen_y=True,
                                     font_size=70, font_colour = (255, 255, 255))

            time_of_darkness = rospy.get_time() + fade_time
            original_bg_col = background_colour
            background_colour = list(background_colour) # Background var we will change

            # Get how much time is left till full darkness
            time_left = time_of_darkness - rospy.get_time()
            if time_left < 0.2:
                still_fading = False

            # Check for quit
            for event in self.pygame.event.get():
                # Check if the user clicks the X
                if event.type == self.pygame.QUIT:
                    return False
                # On mouse release play animation to show where cursor is
                elif (event.type == self.inputUp):
                    # Tell system to play animation when drawing
                    self.animation_manager.StartTouchAnimation(self.pygame.mouse.get_pos())

            # Scale background and text brightness
            fade_scalar = (time_left / fade_time)
            for chan_idx in range(len(background_colour)):
                background_colour[chan_idx] = original_bg_col[chan_idx] * fade_scalar
            new_text_col = int(255 * fade_scalar)
            font_col = (new_text_col ,new_text_col ,new_text_col)
            text_object.set_colour(font_col)

            # Draw background and objects
            self.renderer.DrawBackground(background_colour)
            text_object.render(self.window)
            self.animation_manager.DrawTouchAnimation(self.window)  # Also draw touches
            self.pygame.display.update()  # Update all drawn objects


    def black_screen(self, run):
        """ Non-Blocking black screen. Will read events and work accordindly, but wont stick you into a while loop"""

        if run:
            # Check for quit
            for event in self.pygame.event.get():
                # Check if the user clicks the X
                if event.type == self.pygame.QUIT:
                    return False
                # On mouse release play animation to show where cursor is
                elif (event.type == self.inputUp):
                    # Tell system to play animation when drawing
                    self.animation_manager.StartTouchAnimation(self.pygame.mouse.get_pos())

            # Draw background and objects
            self.renderer.DrawBackground((0 ,0 ,0))
            self.animation_manager.DrawTouchAnimation(self.window)  # Also draw touches
            self.pygame.display.update()  # Update all drawn objects

            return True

    def fade_to_black_screen(self, run, message, background_colour, fade_scalar ):
        """ Non-Blocking fade then stay on black screen.
        Will read events and work accordingly, but wont stick you into a while loop"""

        if run:
            # Convert tuple to list so to make it mutable
            background_colour = list(background_colour)

            # Not ideal that we create a text object each time but...
            text_object = TextObject(self.window, self.window_center, message, cen_x=True, cen_y=True,
                                     font_size=70, font_colour=(255, 255, 255))

            # Check for quit
            for event in self.pygame.event.get():
                # Check if the user clicks the X
                if event.type == self.pygame.QUIT:
                    return False
                # On mouse release play animation to show where cursor is
                elif (event.type == self.inputUp):
                    # Tell system to play animation when drawing
                    self.animation_manager.StartTouchAnimation(self.pygame.mouse.get_pos())

            # Scale background and text brightness
            for chan_idx in range(len(background_colour)):
                background_colour[chan_idx] = background_colour[chan_idx] * fade_scalar
            new_text_col = int(255 * fade_scalar)
            font_col = (new_text_col, new_text_col, new_text_col)
            text_object.set_colour(font_col)

            # Draw background and objects
            self.renderer.DrawBackground(background_colour)
            text_object.render(self.window)
            self.animation_manager.DrawTouchAnimation(self.window)  # Also draw touches
            self.pygame.display.update()  # Update all drawn objects

            return True


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Text objects~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class TextObject():

    def __init__(self ,window, window_center, text, wrap_text = False, location=None, cen_x = False, cen_y=False, font_size=30, font_colour=(255 ,255 ,255), inputMode = 1):
        """
        Create object that we can manipulate, ie move it's position and change it's parameters
        Also allows text wrapping to screen. This will cause text to be a list instead of a single object internally
        """
        self.window = window
        self.window_center = window_center
        self.font_size = font_size
        self.font = pygame.font.Font('freesansbold.ttf', self.font_size)
        self.font_colour = font_colour
        self.text = text
        self.type = "TextObject"
        if location != None and type(location) != list:
            self.location = list(location)
        else:
            self.location = location
        self.cen_x = cen_x
        self.cen_y = cen_y
        self.wrap_text = wrap_text
        if self.wrap_text:
            self.text_obj = self.create_wrapped_text()
        else:
            self.text_obj = self.font.render(self.text, False, self.font_colour)
            self.textRect = self.text_obj.get_rect()
            if self.location != None:
                self.set_pos(self.location)
            if cen_x:
                self.textRect[0] = window_center[0] - (self.textRect[2 ] /2)
            if cen_y:
                self.textRect[1] = window_center[1] - (self.textRect[3 ] /2)

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

    def render(self, window, grey=False, re_render = False):
        """handle drawing text"""
        if self.wrap_text:
            for obj_data in self.text_obj:
                obj = obj_data[0]
                obj_rect = obj_data[1]
                if re_render:
                    obj_data[0] = self.font.render(self.text, False, self.font_colour)
                    obj = obj_data[0]
                if self.window is None:
                    window.blit(obj, obj_rect)
                else:
                    self.window.blit(obj, obj_rect)
        else:
            if re_render:
                self.text_obj = self.font.render(self.text, False, self.font_colour)
            if self.window is None:
                window.blit(self.text_obj, self.textRect)
            else:
                self.window.blit(self.text_obj, self.textRect)

    def set_colour(self, font_col):
        """Set new colour and re render obj"""
        if self.wrap_text:
            for obj_data in self.text_obj:
                obj_data[0] = self.font.render(self.text, False, font_col)
        else:
            self.text_obj = self.font.render(self.text, False, font_col)

    def set_pos(self, pos):
        self.textRect[0] = pos[0]
        self.textRect[1] = pos[1]
        if self.wrap_text:
            print("THIS FEATURE IS NOT MADE YET")

    def set_x(self, x):
        self.textRect[0] = x
        if self.wrap_text:
            print("THIS FEATURE IS NOT MADE YET")

    def set_y(self, y):
        self.textRect[1] = y
        if self.wrap_text:
            print("THIS FEATURE IS NOT MADE YET")

    def set_text(self, text):
        self.text = text
        if self.wrap_text:
            self.text_obj = self.create_wrapped_text()
        else:
            self.text_obj = self.font.render(self.text, False, self.font_colour)

    def create_wrapped_text(self):
        """
        Method wraps text at given font and returns a text object of the block
        """

        # Get screen width
        res = pygame.display.Info()
        screen_width = res.current_w - 150 # HARD CODED = BAD!!
        font = pygame.font.Font('freesansbold.ttf', self.font_size)

        # Loop through woods and wrap them
        lines = []
        words = self.text.split() # split string into words
        while len(words) > 0:
            # Get as many words as will fit within allowed_width
            line_words = []
            while len(words) > 0:
                line_words.append(words.pop(0))
                fw, fh = font.size(' '.join(line_words + words[:1]))
                if fw > screen_width:
                    break

            # Add a line consisting of those words
            line = ' '.join(line_words)
            lines.append(line)

        text_objs = []
        x, y, w, h = 0, 0, 0, 0
        # Make an object per line of text
        for line in lines:
            obj = self.font.render(line, False, self.font_colour)
            text_objs.append( [obj, [x, y, w, h]] )
            h += obj.get_height()
            if obj.get_width() > w:
                w = obj.get_width()
        # Rect of all lines combined
        self.textRect = [x, y, w, h]

        # Get h and w of top line
        line_height = text_objs[0][0].get_height()
        line_width = text_objs[0][0].get_width()

        # Get the x and y pos of starting line, we will use it to add to for each next line
        if self.location == None:
            self.location = [0, 0]
        if self.cen_x:
            self.location[0] = self.textRect[0] = self.window_center[0] - (w / 2)
        if self.cen_y:
            self.location[1] = self.window_center[1] - (h / 2)

        # Set pos of each line
        for line_idx in range(0, len(text_objs)):
            x = self.location[0] + (line_height * line_idx)
            y = self.location[1] + (line_height * line_idx)
            text_objs[line_idx][1] = [x, y, line_width, line_height]

        return text_objs

#####################################################Renderer##################################################################

class Renderer():
    """Class to render common things, such as background """

    def __init__(self, window, window_center, inputMode = 1):
        self.window = window
        self.window_center = window_center

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


    def DrawBackground(self, colour):
        """takes window and colour to fill in the background of the window """
        self.window.fill(colour)  # Fill background black

    def DrawText(self, message, location, font_size=30, font_colour=(255, 255, 255)):
        """handle drawing text"""
        font = pygame.font.Font('freesansbold.ttf', font_size)
        text = font.render(message, False, font_colour)
        textRect = text.get_rect()
        textRect.center = location
        self.window.blit(text, textRect)
        return text, textRect

    def DrawTextCentered(self, message, font_size=30, font_colour=(255, 255, 255), x=-1, y=-1):
        """Draws text that's centered in X and Y"""
        font = pygame.font.Font('freesansbold.ttf', font_size)
        text = font.render(message, False, font_colour)
        textRect = text.get_rect()
        center = self.window_center
        if x < 0 and y < 0:  # if no X and Y given center both
            textRect.center = center
        elif x > -1 and y < 0:  # center Y but use the given X
            textRect.center = [x, center[1]]
        elif x < 0 and y > -1:  # center x and use given Y
            textRect.center = [center[0], y]
        else:  # if they gave both X and Y instead of using the other draw text func (for some reason?) use both x and y
            textRect.center = [x, y]

        self.window.blit(text, textRect)
        return text, textRect

    def HighlightRect(self, target_rect, pygame):
        """
        Using the rect of the obj, draw a box around it and an arrow pointing towards it
        Returns the rect of the produced arrow
        """

        # Variables
        this_path = os.path.dirname(__file__)
        relative_arrow = "/game_assets/graphics/arrow_pointer.png"
        path_to_arrow = this_path + relative_arrow
        arrow_img = pygame.image.load(path_to_arrow).convert_alpha()
        arrow_size = (arrow_img.get_width(), arrow_img.get_height())
        bounding_scalar = 50  # how much bigger the bounding should be than the original rect
        time_scalar = 50  # the scaler to move our arrow in a sine wave
        time = rospy.get_time()

        # Create bounding box
        bound_w = target_rect[2] + bounding_scalar
        bound_h = target_rect[3] + bounding_scalar
        bound_x = target_rect[0] + ((target_rect[2] - bound_w) / 2)
        bound_y = target_rect[1] + ((target_rect[3] - bound_h) / 2)
        bounding_rect = (bound_x, bound_y, bound_w, bound_h)

        # Create Arrow
        bounding_rect_center = (bound_x + (bound_w / 2), bound_y + (bound_h / 2))
        arrow_x = bound_x + (bound_w / 2) - (
                arrow_size[0] / 2)  # use our Y not cen y. 150 is half the width of the arrow
        if bounding_rect_center[1] > 600:  # if our arrow would hit the top of the screen draw it under the box
            arrow_y = bound_y - arrow_size[1] - 50 - (math.sin(time) * time_scalar)
            image = pygame.transform.scale(arrow_img, arrow_size)
        else:
            arrow_y = bound_y + bound_h + 50 + (math.sin(time) * time_scalar)
            image = pygame.transform.scale(arrow_img, arrow_size)
            image = pygame.transform.rotozoom(image, 180, 1)
        arrow_rect = (arrow_x, arrow_y, arrow_size[0], arrow_size[1])

        # Draw
        bound_box = pygame.draw.rect(self.window, (255, 0, 0), bounding_rect, width=20,
                                     border_radius=5)  # draw bounding box
        self.window.blit(image, arrow_rect)  # draw arrow

        return arrow_rect


#####################################################SoundManager##################################################################

class SoundManager():
    """Class to manage communication with sound_player service"""

    def __init__(self, music_filepath):
        self.music_filepath = music_filepath
        rospy.wait_for_service('/sound_player_service')
        self.sound_player = rospy.ServiceProxy('/sound_player_service', sound_player_srv, persistent=True)

    def load_track(self, track_title, track_time=0.0):
        """gives the sound player the song data, has it load it up to return information about the song, this is essentially "start_track" but betteer """
        track_path = os.path.join(self.music_filepath, track_title)
        # print(track_path)
        # Start track
        operation = "load_track"
        song_data = self.call_sound_player(operation, track_path, track_time)
        rospy.sleep(0.15)  # requires this to function consistently
        return song_data

    def call_sound_player(self, operation, data_1="", data_2=0.0):
        """makes it easier to call sound_player"""
        # rospy.wait_for_service('/sound_player_service') #dont wait since our connection is persistent.
        song_data = self.sound_player(operation, data_1, data_2)
        return song_data

    def start_track(self, track_title, track_time=0.0):
        """Starts a track and also saves the information returned as previous song, so we can replay songs without sending a new request"""
        # store default path to music

        track_path = os.path.join(self.music_filepath, track_title)
        # print(track_path)
        # Start track
        operation = "start_track"
        callback_data = self.call_sound_player(operation, track_path, track_time)
        rospy.sleep(0.1)  # requires this to function consistently
        song_data = self.request_song_data()
        return song_data

    def stop_track(self):
        """Stop track, dont hold any data in memory"""
        operation = "stop_track"
        data = self.call_sound_player(
            operation)  # we only need operation, the other variables can default, theyre ignored anyways
        rospy.sleep(0.1)  # requires this to function consistently
        return data

    def pause_unpause(self):
        """Pause track and resume from the same point later"""
        operation = "pause_resume"
        data = self.call_sound_player(
            operation)  # we only need operation, the other variables can default, theyre ignored anyways
        rospy.sleep(0.1)  # requires this to function consistently
        return data

    def pause(self):
        """Pause track if it's playing"""
        operation = "pause"
        data = self.call_sound_player(
            operation)  # we only need operation, the other variables can default, theyre ignored anyways
        rospy.sleep(0.1)  # requires this to function consistently
        return data

    def unpause(self):
        """play track if it's paused"""
        operation = "resume"
        data = self.call_sound_player(
            operation)  # we only need operation, the other variables can default, theyre ignored anyways
        rospy.sleep(0.1)  # requires this to function consistently
        return data

    def volume_change(self, volume_percentage):
        """change volume, 1.0 = 100%, 0.5 = 50% etc"""
        operation = "volume"
        volume = volume_percentage * 100  # other methods use decimal percentages, so for consistency this method does too, but then converts it to numerical percentage
        status = self.call_sound_player(operation,
                                        data_2=volume).status  # we only need operation and data_2 the other variable can default, it's ignored anyways
        rospy.sleep(0.1)  # requires this to function consistently
        return status

    def request_song_data(self):
        """ask ros topic for data TODO method needs reworking"""
        # time = rospy.get_time()
        data = self.call_sound_player("request_data")
        # data = rospy.wait_for_message("/song_data_publisher", SongData) #through the node
        # print("time taken waiting for node ", rospy.get_time() - time)
        return data

    def return_wav_lenth(self, song_path):
        if not os.path.exists(song_path):
            this_path = os.path.dirname(__file__)
            music_filepath = r"/game_assets/music/"
            song_path = this_path + music_filepath + song_path
        with contextlib.closing(wave.open(song_path, 'r')) as f:
            frames = f.getnframes()
            rate = f.getframerate()
            duration = frames / float(rate)
            return (duration)

    def slice_song(self, num_segments, target_song_to_split, distracting_songs=None):
        """
        Method that chops up a song and creates segments from it, this uses the file name, so you can use 2 differenty files and append the returned variabels together to jumble multiple songs
        target_song should be the path to a single song
        distracting songs should be a list of paths to songs, even if only 1 song is used
        """

        # Define variables
        this_path = os.path.dirname(__file__)
        music_filepath = r"/game_assets/music/"
        song_path = this_path + music_filepath + target_song_to_split
        self.path_to_save = this_path + music_filepath + "temp/"
        total_wav_len = (self.return_wav_lenth(song_path)) * 1000  # convert to millisecond
        slice_size = total_wav_len / num_segments
        prev_slice = 0
        correct_segs = []
        distract_segments = []

        # Slice and save songs
        for i in range(0, num_segments):
            audio_segment = AudioSegment.from_wav(song_path)
            audio_segment = audio_segment[prev_slice: prev_slice + slice_size]
            prev_slice += slice_size
            song_name = "c" + str(i) + target_song_to_split.split("/")[-1]
            # print(song_name) #TODO REMOVE ME
            song_path_save = self.path_to_save + song_name  # save song with new name, add C to show it's the correct song
            audio_segment.export(song_path_save, format="wav")  # Exports to a wav file in the current path.
            correct_segs.append("temp/" + song_name)  # list of all songs made
            rospy.loginfo("Temp file saved")

        # handle distracting song
        if type(distracting_songs) != list and distracting_songs != None:
            distracting_songs = [distracting_songs]
        if distracting_songs != None:
            for song in distracting_songs:  # so we can use multiple distracting songs
                song_path = this_path + music_filepath + song
                total_wav_len = (self.return_wav_lenth(song_path)) * 1000  # convert to millisecond
                slice_size = total_wav_len / num_segments
                prev_slice = 0

                # Slice distracting songs and add to same list
                for i in range(0, num_segments):
                    audio_segment = AudioSegment.from_wav(song_path)
                    audio_segment = audio_segment[prev_slice: prev_slice + slice_size]
                    prev_slice += slice_size
                    song_name = "d" + str(i) + song.split("/")[-1]
                    song_path_save = self.path_to_save + song_name  # cut off the problematic parts TODO change this to look for the "/" and cut after the "/"
                    audio_segment.export(song_path_save,
                                         format="wav")  # Exports to a wav file in the current path.
                    distract_segments.append("temp/" + song_name)  # list of all songs made
                    # rospy.loginfo("Temp file saved")

        return correct_segs, distract_segments


#####################################################QTManager/CommandManager##################################################################


class QTManager():
    """Handles sending communication to robot """

    def __init__(self, levels = None, debug = False):
        """
        You can pass in levels which is the levels_loader class. this is a requirement if you want to use
        qt_say_blocking with a blank screen
        """
        self.robo_timer = TimeFunctions()
        self.time_per_char = 0.11
        self.right_arm_pos_pub = rospy.Publisher('/qt_robot/right_arm_position/command', Float64MultiArray,
                                                 queue_size=10)
        self.left_arm_pos_pub = rospy.Publisher('/qt_robot/left_arm_position/command', Float64MultiArray,
                                                queue_size=10)
        self.level_loader = levels
        self.talking_anim_time = 4 # How long the talking animation plays for (seconds)
        self.debug = debug


    def init_robot(self, arm_vel):
        """Method to init robot parameters"""
        # Set control mode, incase they were changed beforehand
        rospy.wait_for_service('/qt_robot/motors/setControlMode')
        self.set_mode = rospy.ServiceProxy('/qt_robot/motors/setControlMode', set_control_mode)
        mode_changed = self.set_mode(["right_arm", "left_arm"], 1)
        if mode_changed:
            rospy.loginfo("Motors successfully set control mode 1")
        else:
            rospy.loginfo("Motor control mode could not be changed")
            self.run = False
        # Set velocity of arms incase they were set differently
        rospy.wait_for_service('/qt_robot/motors/setVelocity')
        set_vel = rospy.ServiceProxy('/qt_robot/motors/setVelocity', set_velocity)
        speed_changed = set_vel(["right_arm", "left_arm"], arm_vel)
        if speed_changed:
            rospy.loginfo("Motors successfully set to default speed ({})".format(arm_vel))
        else:
            rospy.loginfo("Motor speed could not be changed")
            self.run = False

    def set_arm_vel(self, arm_vel ,command_blocking = False):
        "Set velocity of arm motors"
        # TODO change this to take in list of what motors to change speed of
        rospy.wait_for_service('/qt_command_service')
        command_controller = rospy.ServiceProxy('/qt_command_service', qt_command)
        command_complete = command_controller("velocity", str(arm_vel), command_blocking)
        return command_complete

    def send_qt_command(self, speech=None, gesture=None, emote=None, command_blocking=False):
        """Neatens and simplifies sending commands to QT
        if we want to use multiple functions of QT at once and dont care about tracking time taken,
        we should use this method instead of the others
        """
        #If any command fails, send false
        command_status = []
        if speech != None:  # do qt_speak
            # print("sending speech req")
            rospy.wait_for_service('/qt_command_service')
            command_controller = rospy.ServiceProxy('/qt_command_service', qt_command)
            command_complete = command_controller("tts", speech, command_blocking)
            command_status.append(command_complete)
        if gesture != None:  # do qt_speak
            rospy.wait_for_service('/qt_command_service')
            command_controller = rospy.ServiceProxy('/qt_command_service', qt_command)
            command_complete = command_controller("gesture", gesture, command_blocking)
            command_status.append(command_complete)
        if emote != None:  # do qt_speak
            rospy.wait_for_service('/qt_command_service')
            command_controller = rospy.ServiceProxy('/qt_command_service', qt_command)
            command_complete = command_controller("emote", emote, command_blocking)
            command_status.append(command_complete)

        success = True
        if False in command_status:
            success = False
        return success

    def qt_say_blocking(self, text, black_screen = False):
        """Makes QT say something, then makes you wait until the speaking is done"""
        if self.debug:
            print("QT Say: {}".format(text))
        timer_len = len(text) * self.time_per_char
        self.qt_emote("talking") # Show talking face
        # Set timers
        self.robo_timer.CreateTimer("QT_SAY_BLOCKING", timer_len) # Timer for done speaking
        self.robo_timer.CreateTimer("EMOTE FINISHED", self.talking_anim_time) # Timer for emote finished
        self.send_qt_command(speech=text)
        talking = True
        while talking and not rospy.is_shutdown():
            if black_screen:
                self.level_loader.black_screen(True)
            if self.robo_timer.CheckTimer("EMOTE FINISHED"):
                # Renew talking emote and start timer again
                self.qt_emote("talking")
                self.robo_timer.CreateTimer("EMOTE FINISHED", self.talking_anim_time)  # Timer for emote finished
            if self.robo_timer.CheckTimer("QT_SAY_BLOCKING"):  # if our timer is done
                talking = False

    def qt_say(self, text):
        """Makes QT say something, then makes starts a timer until the speaking is done"""
        # TODO add threading here, to keep repeating the emote, until the speach timer ends.
        if self.debug:
            print("QT Say: {}".format(text))
        timer_len = len(text) * self.time_per_char
        timer_id = "QT_SAY"
        self.robo_timer.CreateTimer(timer_id, timer_len)  # Creates timer with ID QT_SAY
        self.send_qt_command(speech=text)  # Have QT say the text
        return timer_id

    def qt_gesture(self, req_gesture):
        """Make QT do gesture, non-blocking """
        if self.debug:
            print("QT gesture: {}".format(req_gesture))
        self.send_qt_command(gesture=req_gesture)

    def qt_emote(self, req_emote):
        """Make QT emote, non-blocking"""
        if self.debug:
            print("QT emote: {}".format(req_emote))
        self.send_qt_command(emote=req_emote)

    def qt_actuate(self, motors_pos, command_blocking = False):
        """
        Make QT move its arms and head
        requires this data structure
        [ [motor_list], [motor_pos] ]
        e.g:
        [ ["left_arm", "right_arm", "head"], [[0,0,0], [0,0,0], [0,0]] ]
        """
        if self.debug:
            print("Moving QT's {} joints to {}".format(motors_pos[0], motors_pos[1]))
        rospy.wait_for_service('/qt_command_service')
        command_controller = rospy.ServiceProxy('/qt_command_service', qt_command)
        command_complete = command_controller("actuation", motors_pos, command_blocking)
        return command_complete

    def move_right_arm(self, joint_angles, command_blocking = False):
        """ Move just right arm """
        if self.debug:
            print("Moving QT's right arm joint to {}".format(joint_angles))
        rospy.wait_for_service('/qt_command_service')
        command_controller = rospy.ServiceProxy('/qt_command_service', qt_command)
        motor_pos = str( [["right_arm"], [joint_angles] ] )
        command_complete = command_controller("actuation", motor_pos, command_blocking)
        return command_complete

    def move_left_arm(self, joint_angles, command_blocking = False):
        """ Move left arm, but using coordinates for right arm, and flip them automatically"""
        if self.debug:
            print("Moving QT's left arm joint to {}".format(joint_angles))
        rospy.wait_for_service('/qt_command_service')
        command_controller = rospy.ServiceProxy('/qt_command_service', qt_command)
        joint_angles[0] = -joint_angles[0]
        motor_pos = str([["left_arm"], [joint_angles]])
        command_complete = command_controller("actuation", motor_pos, command_blocking)
        return command_complete

    def set_voice_vol(self, volume, int_percent = True):
        """ Set volume using float between 0.0 - 1.0 or int between 0 and 100
            Volume = either an integer between 0 and 100, or float between 0.0 and 1.0
            int_percent = specifies to use 0-100 range or not. set true to use integers between 0-100
        """
        if int_percent and (0 <= volume <= 100):
            newVolume = int(volume)
        elif not int_percent and (0 <= volume <= 1):
            newVolume = int(volume * 100) # convert to 0-100 range for service
        else:
            print("Invalid volume given when setting robot voice: {}".format(volume))
            return

        service = "/qt_robot/setting/setVolume"
        rospy.wait_for_service(service)
        set_voice_proxy = rospy.ServiceProxy(service, setting_setVolume)
        set_voice_proxy(newVolume)
        if self.debug:
            print("Robot volume changed to {}".format(newVolume))


#####################################################AnimationManager##################################################################

class AnimationManager():
    """Class for misselaneous functions"""

    def __init__(self, pygame):
        self.pygame = pygame
        self.play_touch_animation = False

    def StartTouchAnimation(self, mouse_pos):
        """Start animation """
        # animation vars
        self.colour = (230, 230, 230)  # white
        self.circle_radius = 0  # will be changed dynamically
        self.border_width = 10  # 0 = filled circle
        self.start_animation_time = rospy.get_time()
        self.animation_location = mouse_pos
        self.play_touch_animation = True

    def DrawTouchAnimation(self, window):
        """Handle / play animation """
        if self.play_touch_animation:
            time_since_start = (
                                       rospy.get_time() - self.start_animation_time) * 1000  # seconds elapsed since start convert to milliseconds
            if time_since_start < 300:  # animation time = under half a sec
                scalar = time_since_start / 4  # low numbers = faster growth time
                max_size = 45
                if scalar < max_size:
                    self.circle_radius = scalar
                else:
                    self.circle_radius = max_size

                self.pygame.draw.circle(window, self.colour, self.animation_location, self.circle_radius,
                                        self.border_width)
            else:
                self.play_touch_animation = False


#####################################################TimeFunctions##################################################################

class TimeFunctions():
    """Class to handle general functions such as time keeping"""

    def __init__(self):
        self.timers = {}

    def CreateTimer(self, timer_id, time_to_wait, verbose=True):
        """Add's time goal to timers list or replaces old timer """
        # Let it happen, but if a timer overwrites an old one, print warning
        if timer_id in self.timers.keys() and verbose:
            print("You have overwritten an older timer, make sure you're not stuck in a loop")
        self.timers[timer_id] = rospy.get_time() + time_to_wait
        return (timer_id)

    def CheckTimer(self, timer_id):
        if timer_id in self.timers.keys():
            if self.timers[timer_id] <= rospy.get_time():
                self.timers.pop(timer_id)
                return True  # timer is done
            else:
                return False  # Timer is not done yet
        else:  # timer doesnt exist
            print("Timer referenced does not exist, check timer ID given")
            return None

    def get_timers(self):
        return self.timers  # return the timer list


#####################################################Button##################################################################

class Button():
    """
    class used for the generation and management of buttons
    """

    def __init__(self, image_path, x_y_locations, pygame, return_info={}, scale=1.0, unique_id="", on_click=object,
                 on_release=object, text="", should_grey=True, inputMode = 1):
        if not os.path.exists(image_path):
            print("File does not exist. Path = ", image_path)
        #else:
        #     print("File located at",image_path)
        self.pygame = pygame
        raw_image = self.pygame.image.load(image_path).convert_alpha()
        grey_scaled_raw_image = General().convert_to_grey(raw_image)
        img_x = x_y_locations[0]
        img_y = x_y_locations[1]
        img_w = int(raw_image.get_width() * scale)
        img_h = int(raw_image.get_height() * scale)
        scaled_size = (img_w, img_h)
        self.image = self.pygame.transform.scale(raw_image, scaled_size)
        self.image_greyscale = self.pygame.transform.scale(grey_scaled_raw_image, scaled_size)
        self.rect = self.pygame.Rect(img_x, img_y, img_w, img_h)
        if unique_id == "":
            self.id = rospy.get_time()  # unique ID for each button based on time when made
        else:
            self.id = unique_id
        self.return_info = return_info
        self.type = "Button"
        self.should_grey = should_grey
        self.on_click = on_click # TODO, THIS TRIGERS ON RELEASE INSTEAD OF ON CLICK...
        self.on_release = on_release

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

        self.text = text
        self.init_text()

    def init_text(self):
        font_scale = 80
        too_large = True
        # Keep scaling down text until it fits in the button
        while too_large or font_scale <= 2:
            text, textRect = self.create_text(font_scale)
            if textRect[2] < (self.rect[2] * 0.85): # *0.9 gives a buffer on either side of the text
                too_large = False
            font_scale -= 2
        self.text = text
        self.textRect = textRect
        self.textRect.center = self.rect.center # Center text in center button

    def create_text(self, font_percent=70):
        self.font_size = int(self.rect[3] * (font_percent / 100))  # 70% of button height by default
        font = self.pygame.font.Font('freesansbold.ttf', self.font_size)
        text = font.render(self.text, False, (0, 0, 0))
        textRect = text.get_rect()
        return text, textRect

    def render(self, screen, grey=False):
        """Draw button onto given screen, either as greyscale or coloured"""
        if grey and self.should_grey:  # if we get a request to pause show greyscaled version
            screen.blit(self.image_greyscale, self.rect)
        else:
            screen.blit(self.image, self.rect)
        if self.text != "":
            screen.blit(self.text, self.textRect)

    def get_event(self, event, mouse_pos):
        """returns if button was pressed"""
        # If the mouse clicked this button
        if event.type == self.inputUp and event.button == 1:
            if self.rect.collidepoint(mouse_pos):
                if self.on_click is not None:
                    self.on_click()
                return True
            else:
                return False
        else:
            return False

    def set_pos(self, newpos):
        """Takes a list of (x,y) and sets rect"""
        self.rect[0] = newpos[0]
        self.rect[1] = newpos[1]

    def get_pos(self):
        """Return X and Y coordinates"""
        return (self.rect[0], self.rect[1])

    def set_info(self, info):
        self.return_info = info

    def get_info(self):
        return self.return_info

    def get_rect(self):
        """returns object rect"""
        return self.rect


######################################################ToggleButton#################################################################

class ToggleButton():
    """Class to load images that serve as buttons """

    def __init__(self, default_image_path, toggled_image_path, x_y_locations,
                 pygame, scale=1, unique_id="", return_info="", when_toggle_on=object, when_toggle_off=object,
                 should_grey = True, inputMode = 1):
        # Set vars
        self.pygame = pygame
        self.highlighted = False
        self.return_info = return_info
        self.toggle_state = False
        self.when_toggle_on = when_toggle_on
        self.when_toggle_off = when_toggle_off

        # load images and make grey versions
        raw_image = self.pygame.image.load(default_image_path).convert_alpha()
        raw_img_grey = General().convert_to_grey(raw_image)
        toggled_raw_image = self.pygame.image.load(toggled_image_path).convert_alpha()
        toggled_raw_grey = General().convert_to_grey(toggled_raw_image)

        # Scale and set pos of imgs
        img_x = x_y_locations[0]
        img_y = x_y_locations[1]
        img_w = int(raw_image.get_width() * scale)
        img_h = int(raw_image.get_height() * scale)
        scaled_size = (img_w, img_h)
        self.image = self.pygame.transform.scale(raw_image, scaled_size)
        self.image_grey = self.pygame.transform.scale(raw_img_grey, scaled_size)
        self.toggled_image = self.pygame.transform.scale(toggled_raw_image, scaled_size)
        self.toggled_image_grey = self.pygame.transform.scale(toggled_raw_grey, scaled_size)
        self.rect = self.pygame.Rect(img_x, img_y, img_w, img_h)

        if unique_id == "":
            self.id = rospy.get_time()  # unique ID for each button based on time when made
        else:
            self.id = unique_id
        self.type = "ToggleButton"
        self.should_grey = should_grey

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


    def render(self, screen, grey=False):
        """Draw image onto screen"""
        if self.toggle_state:
            if grey and self.should_grey:
                screen.blit(self.toggled_image_grey,
                            self.rect)
            else:
                screen.blit(self.toggled_image, self.rect)
        else:
            if grey and self.should_grey:
                screen.blit(self.image_grey, self.rect)
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

    def toggle_off(self):
        """ Toggle to off from either state.
        If already toggled off, do nothing otherwise perform on toggle off func"""
        if self.toggle_state:
            self.when_toggle_off()
            self.toggle_state = False

    def toggle_on(self):
        """ Toggle to on from either state.
        If already toggled on, do nothing otherwise perform on toggle on func"""
        if not self.toggle_state:
            self.when_toggle_on()
            self.toggle_state = True

    def toggle_img(self):
        """Toggle but only the img to render """
        self.toggle_state = not self.toggle_state
        return (self.toggle_state)

    # OLD GET EVENT
    def old_get_event(self, event, mouse_pos):
        """Button event handle, if mouse release, then toggle"""
        mouse_on_button = self.rect.collidepoint(mouse_pos)
        if mouse_on_button:
            if event.type == self.inputUp:
                return self.toggle_toggle()
        return self.toggle_state

    # Is a rework of get_event
    def get_event(self, event, mouse_pos):
        """ Toggle the button, and return true if it was clicked this loop"""
        mouse_on_button = self.rect.collidepoint(mouse_pos)
        if mouse_on_button:
            if event.type == self.inputUp:
                self.toggle_toggle()
                return True
        return False

    def get_info(self):
        return self.return_info
    
    def get_rect(self):
        return self.rect

    def get_pos(self):
        """Takes a list of (x,y) and sets rect"""
        return (self.rect[0], self.rect[1])

######################################################PausePlayButton#################################################################

class PausePlayButton():
    """All functionality of toggle button, but with an option to replace with a 3rd image"""

    def __init__(self, pause_path, play_path, rewind_path,
                 x_y_locations, pygame, scale=1, unique_id="", on_pause=object, on_play=object,
                 should_grey = True, inputMode = 1):
        # Set vars
        self.pygame = pygame
        self.highlighted = False
        self.playing = True  # starts paused
        self.on_pause = on_pause
        self.on_play = on_play
        self.rewind_toggle = False  # show rewind or not
        self.should_grey = should_grey

        # load images
        raw_play = self.pygame.image.load(play_path).convert_alpha()
        raw_play_grey = General().convert_to_grey(raw_play)
        raw_pause = self.pygame.image.load(pause_path).convert_alpha()
        raw_pause_grey = General().convert_to_grey(raw_pause)
        raw_rewind = self.pygame.image.load(rewind_path).convert_alpha()
        raw_rewind_grey = General().convert_to_grey(raw_rewind)

        # Scale and set pos of imgs
        img_x = x_y_locations[0]
        img_y = x_y_locations[1]
        img_w = int(raw_play.get_width() * scale)
        img_h = int(raw_play.get_height() * scale)
        scaled_size = (img_w, img_h)  # scale all 3 images by same scalars
        self.play = self.pygame.transform.scale(raw_play, scaled_size)
        self.play_grey = self.pygame.transform.scale(raw_play_grey, scaled_size)
        self.pause = self.pygame.transform.scale(raw_pause, scaled_size)
        self.pause_grey = self.pygame.transform.scale(raw_pause_grey, scaled_size)
        self.rewind = self.pygame.transform.scale(raw_rewind, scaled_size)
        self.rewind_grey = self.pygame.transform.scale(raw_rewind_grey, scaled_size)
        self.rect = self.pygame.Rect(img_x, img_y, img_w, img_h)
        if unique_id == "":
            self.id = rospy.get_time()  # unique ID for each button based on time when made
        else:
            self.id = unique_id
        self.type = "PausePlayButton"

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

    def render(self, screen, grey=False):
        """Draw image onto screen"""
        if self.rewind_toggle:
            if grey and self.should_grey:
                screen.blit(self.rewind_grey, self.rect)  # TODO replace this with the greyscaled version of this image
            else:
                screen.blit(self.rewind, self.rect)
        else:
            if self.playing:
                if grey and self.should_grey:
                    screen.blit(self.pause_grey,
                                self.rect)  # TODO replace this with the greyscaled version of this image
                else:
                    screen.blit(self.pause, self.rect)
            else:
                if grey and self.should_grey:
                    screen.blit(self.play_grey,
                                self.rect)  # TODO replace this with the greyscaled version of this image
                else:
                    screen.blit(self.play, self.rect)
        return screen  # redundant

    def its_rewind_time(self):  # an easter egg
        """toggles rewind to true  ONLY TO BE USED EXTERNALLY"""
        self.rewind_toggle = True
        self.playing = False  #
        return self.rewind_toggle

    def rewind_off(self):
        """toggles rewind to false  ONLY TO BE USED EXTERNALLY"""
        self.rewind_toggle = False
        self.playing = True
        return self.rewind_toggle

    def toggle_pause(self):
        """Toggles the function 'self.toggle' """
        # act acording to the state
        if self.playing:  # if we're playing and we click the icon = Pause
            self.on_pause()  # pause
        else:
            self.on_play()  # unpause
            self.rewind_toggle = False  # if this was true, set it to false now
        # flip the state
        self.playing = not self.playing
        return (self.playing)

    def toggle_img(self):
        """Toggle but only the img to render """
        self.playing = not self.playing
        return (self.playing)

    def get_event(self, event, mouse_pos):
        """Button event handle, if mouse release, then toggle"""
        mouse_on_button = self.rect.collidepoint(mouse_pos)
        if mouse_on_button:
            if event.type == self.inputUp:
                return self.toggle_pause()  # flip paused and play
        return self.playing  # return the state we were in

    def get_rect(self):
        return self.rect

    def get_pos(self):
        """Takes a list of (x,y) and sets rect"""
        return (self.rect[0], self.rect[1])


######################################################DragableButton#################################################################

class DraggableButton():
    """Class to load images that serve as buttons that can be dragged and dropped """

    def __init__(self, image_path, toggled_image_path, x_y_locations, pygame,
                 scale=1, return_info={}, when_toggle_on=object, when_toggle_off=object, unique_id="",
                 should_grey = True, inputMode = 1):

        self.pygame = pygame

        # Load images
        raw_image = self.pygame.image.load(image_path).convert_alpha()
        raw_image_grey = General().convert_to_grey(raw_image)
        toggled_raw_image = self.pygame.image.load(toggled_image_path).convert_alpha()
        toggled_raw_image_grey = General().convert_to_grey(toggled_raw_image)

        # Set pos
        img_x = x_y_locations[0]
        img_y = x_y_locations[1]
        self.img_w = int(raw_image.get_width() * scale)
        self.img_h = int(raw_image.get_height() * scale)

        # Scale
        scaled_size = (self.img_w, self.img_h)
        self.image = self.pygame.transform.scale(raw_image, scaled_size)
        self.toggled_image = self.pygame.transform.scale(toggled_raw_image, scaled_size)
        self.image_grey = self.pygame.transform.scale(raw_image_grey, scaled_size)
        self.toggled_image_grey = self.pygame.transform.scale(toggled_raw_image_grey, scaled_size)
        self.rect = self.pygame.Rect(img_x, img_y, self.img_w, self.img_h)
        self.seg_init_pos = (img_x, img_y)
        self.initial_mouse_pos = self.seg_init_pos

        # Set Vars
        self.highlighted = False
        self.block = False
        self.return_info = return_info
        self.toggle = False
        self.when_toggle_on = when_toggle_on
        self.when_toggle_off = when_toggle_off
        self.mouse_is_held = False
        self.disable_drag = False
        self.should_grey = should_grey

        self.type = "DraggableButton"
        if unique_id == "":
            self.id = rospy.get_time()  # unique ID for each button based on time when made
        else:
            self.id = unique_id

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


    def render(self, screen, grey=False):
        """Draw image onto screen"""
        if self.toggle:
            if grey and self.should_grey:
                screen.blit(self.toggled_image_grey,
                            self.rect)
            else:
                screen.blit(self.toggled_image, self.rect)
        else:
            if grey and self.should_grey:
                screen.blit(self.image_grey, self.rect)
            else:
                screen.blit(self.image, self.rect)
        return screen

    def set_info(self, info):
        """Stores info into correct attribute"""
        self.return_info = info

    def toggle_toggle(self):
        """Toggles the function 'self.toggle' """
        self.toggle = not self.toggle
        if self.toggle:
            self.when_toggle_on()
        else:
            self.when_toggle_off()

        return (self.toggle)

    def set_pos(self, newpos):
        """Takes a list of (x,y) and sets rect"""
        self.rect[0] = newpos[0]
        self.rect[1] = newpos[1]

    def get_pos(self):
        """Takes a list of (x,y) and sets rect"""
        return (self.rect[0], self.rect[1])


    def get_event(self, event, mouse_pos, snap_back = False):
        """Button event handle, if mouse release, then toggle"""
        mouse_on_button = self.rect.collidepoint(mouse_pos)
        if mouse_on_button:
            # At start of drag/ long press, save some vars
            if event.type == self.inputDown and not self.mouse_is_held:
                self.mouse_is_held = True  # Tells us that the mouse was clicked during this event handling
                self.initial_mouse_pos = mouse_pos
                self.seg_init_pos = (self.rect.x, self.rect.y) # for snapping back
            # During drag / long press
            if self.mouse_is_held:
                if event.type == self.inputUp and mouse_pos == self.initial_mouse_pos:
                    self.mouse_is_held = False
                    self.set_pos(self.seg_init_pos)
                    return self.toggle_toggle(), self.rect
                elif event.type == self.inputUp and mouse_pos != self.initial_mouse_pos:
                    self.mouse_is_held = False  # mouse was released somewhere else
                    if snap_back:
                        self.set_pos(self.seg_init_pos)
                else:
                    if not self.disable_drag:
                        x = mouse_pos[0] - self.img_w / 2
                        y = mouse_pos[1] - self.img_h / 2
                        self.set_pos((x ,y))

        return self.toggle, self.rect

######################################################InputBox#################################################################
class InputBox():
    """ rect that you can click on and write text in. Text can be read by method get_text"""

    def __init__(self, x, y, w, h, default_text='', allowed_chars="", fontsize = -1, max_chars = -1,
                 force_lowercase = True, force_uppercase= False, inputMode = 1):
        """
        Creates input box, with some grey text in it, that disappears when typing
        x = horizontal placement of text box (bottom left origin)
        y = vertical placement of text box (bottom left origin)
        w = width of text box
        h = height of text box
        default_text = the text that shows when text box is empty
        allowed_chars = if a char doesn't match any in the string given, then dont add it to text.
        By default, this is all upper and lowercase letters and numbers
        fontsize = the size of the font, -1 or 0 results to font size being equal to box height
        """
        self.COLOR_INACTIVE = pygame.Color('lightskyblue3')
        self.COLOR_ACTIVE = pygame.Color('dodgerblue2')
        if fontsize > 0:
            self.FONT = pygame.font.Font(None, fontsize)
        else:
            self.FONT = pygame.font.Font(None, h)
        self.max_chars = max_chars
        self.rect = pygame.Rect(x, y, w, h)
        self.color = self.COLOR_INACTIVE
        self.default_text = default_text
        self.text = self.default_text
        self.txt_surface = self.FONT.render(self.text, True, self.color)
        self.active = False
        self.returnInput = None
        self.force_lowercase = force_lowercase
        self.force_uppercase = force_uppercase
        # Prefer lowercase to upper case
        if self.force_lowercase and self.force_uppercase:
            self.force_uppercase = False
        self.uppercase = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        self.lowercase = "abcdefghijklmnopqrstuvwxyz"
        self.integers = "0123456789"
        # other_chars = ".-"
        self.allowed_chars = allowed_chars
        if allowed_chars == "" or type(allowed_chars) != str:
            self.allowed_chars = self.uppercase + self.lowercase + self.integers

        # Set inputs to either touch or mouse
        self.input_mode = inputMode  # input mode 1 for touch, 2 for mouse
        if self.input_mode == 2:
            self.inputUp = pygame.MOUSEBUTTONUP
            self.inputDown = pygame.MOUSEBUTTONDOWN
            self.inputMotion = pygame.MOUSEMOTION
        else:
            self.inputUp = pygame.FINGERUP
            self.inputDown = pygame.FINGERDOWN
            self.inputMotion = pygame.FINGERMOTION


    def handle_event(self, event):
        event_triggered = None
        if event.type == self.inputDown:
            # If the user clicked on the input_box rect.
            if self.rect.collidepoint(event.pos):
                event_triggered = "MOUSEBUTTONDOWN"
                # Set to active, if deault text in there, remove it
                if self.text == self.default_text:
                    self.text = ""
                self.active = True
            else:
                if self.text == "" or self.text == None:
                    self.text = self.default_text
                self.active = False
            # Change the current color of the input box.
            self.color = self.COLOR_ACTIVE if self.active else self.COLOR_INACTIVE
        if event.type == pygame.KEYDOWN:
            if self.active:
                event_triggered = "KEYDOWN"
                # Let user delete chars
                if event.key == pygame.K_BACKSPACE:
                    self.text = self.text[:-1]
                else:
                    char = event.unicode
                    # Skip keys such as shift etc
                    if char == "" or char == " ":
                        return
                    # convert upper case chars into lower case and continue
                    if self.force_lowercase:
                        if char in self.uppercase:
                            char_index = self.uppercase.index(char)
                            char = self.lowercase[char_index]
                    elif self.force_uppercase:
                        if char in self.lowercase:
                            char_index = self.lowercase.index(char)
                            char = self.uppercase[char_index]
                    # Check if char is allowed
                    if char in self.allowed_chars:
                        # impose char limit
                        if self.max_chars > 0:
                            if len(self.text) < self.max_chars:
                                self.text += char
                        else:
                            self.text += char

        # Re-render the text only on certain events, to save performance
        if event.type == self.inputDown or event.type == pygame.KEYDOWN or event.type == self.inputUp:
            self.txt_surface = self.FONT.render(self.text, True, self.color)
        return event_triggered

    def get_text(self):
        return self.text

    def render(self, screen):
        screen.blit(self.txt_surface, (self.rect.x+5, self.rect.y+5))
        pygame.draw.rect(screen, self.color, self.rect, 3)

######################################################HorizontalSlider#################################################################

class HorizontalSlider():
    """Class to handle all functions of the horizontal sliders"""

    def __init__(self, image_path_slider, image_path_cursor, x_y_locations, scale=1, on_click=object,
                 on_release=object, music_filepath="/game_assets/music/", inputMode = 1):
        # init slider
        raw_slider_image = pygame.image.load(image_path_slider).convert_alpha()
        self.img_x = x_y_locations[0]
        self.img_y = x_y_locations[1]
        self.scale = scale
        slider_img_w = int(raw_slider_image.get_width() * self.scale)
        slider_img_h = int(raw_slider_image.get_height() * self.scale)
        scaled_size = (slider_img_w, slider_img_h)
        self.slider_image = pygame.transform.scale(raw_slider_image, scaled_size)  # scale up the slider
        self.slider_rect = pygame.Rect(self.img_x, self.img_y, slider_img_w,
                                       slider_img_h)  # make collision box around it according to it's size

        # init cursor
        raw_cursor_image = pygame.image.load(image_path_cursor).convert_alpha()
        cursor_img_w = int(raw_cursor_image.get_width() * self.scale)
        cursor_img_h = int(raw_cursor_image.get_height() * self.scale)
        self.half_cursor_height = cursor_img_h / 2
        scaled_size = (cursor_img_w, cursor_img_h)
        self.cursor_image = pygame.transform.scale(raw_cursor_image, scaled_size)
        self.cursor_rect = pygame.Rect(self.img_x, self.img_y, cursor_img_w, cursor_img_h)

        # init variables
        self.slider_being_held = False
        self.bar_overwrite = 0.0  # percentage of total len, used to know how far along bar is being held
        self.slider_len = 1080 * self.scale  # total pixel length that the bar should extend to at the end of the track
        self.lower_bound_pix = 37 * self.scale  # starting point of red bar (pixel_x)
        slider_min = self.img_x + self.lower_bound_pix  # starting point of red bar (pixel_x)
        slider_max = slider_min + self.slider_len  # ending point of max red bar pixel X location of slider end point
        self.slider_range = (slider_min, slider_max)  # min and max including space to left of bar
        self.on_click = on_click
        self.on_release = on_release
        self.type = "HorizontalSlider"

        # Set inputs to either touch or mouse
        self.input_mode = inputMode  # input mode 1 for touch, 2 for mouse
        if self.input_mode == 2:
            self.inputUp = pygame.MOUSEBUTTONUP
            self.inputDown = pygame.MOUSEBUTTONDOWN
            self.inputMotion = pygame.MOUSEMOTION
        else:
            self.inputUp = pygame.FINGERUP
            self.inputDown = pygame.FINGERDOWN
            self.inputMotion = pygame.FINGERMOTION

    def render(self, screen, progress, grey=False):
        """Draw slider, cursor and progress bar onto screen """
        if grey:  # use grey graphics
            screen.blit(self.slider_image, self.slider_rect)  # draw bar
            self.draw_progress_bar(screen, progress, grey)  # draw the red progress bar
            self.draw_cursor(screen)  # draw the cursor
        else:  # render as normal
            screen.blit(self.slider_image, self.slider_rect)  # draw bar
            self.draw_progress_bar(screen, progress)  # draw the red progress bar
            self.draw_cursor(screen)  # draw the cursor

    def render_bar(self, screen, progress, grey=False):
        """Draw slider, cursor and progress bar onto screen """
        if grey:  # use grey graphics
            screen.blit(self.slider_image, self.slider_rect)  # draw bar
            self.draw_progress_bar(screen, progress, grey)  # draw the red progress bar
        else:  # render as normal
            screen.blit(self.slider_image, self.slider_rect)  # draw bar
            self.draw_progress_bar(screen, progress)  # draw the red progress bar

    def render_cursor(self, screen, progress, grey=False):
        """Draw slider, cursor and progress bar onto screen """
        if grey:  # use grey graphics
            screen.blit(self.slider_image, self.slider_rect)  # draw bar
            self.draw_cursor(screen)  # draw the cursor
        else:  # render as normal
            screen.blit(self.slider_image, self.slider_rect)  # draw bar
            self.draw_cursor(screen)  # draw the cursor

    def draw_progress_bar(self, screen, progress, grey=False):
        """Uses a percentage to colour the completed relevant of the slider in red"""
        complete_bar_width = self.slider_len
        bar_height_pix = 57 * self.scale
        bar_y_offset = 11 * self.scale  # shifts progress bar to center of main bar
        bar_x = self.slider_range[0]  # starting x of progress bar
        bar_y = self.img_y + bar_y_offset  # starting y of bar - thickness of bar + given Y location
        if self.slider_being_held:
            bar_width = complete_bar_width * self.bar_overwrite  # If the user is moving the slider, display their new slider
        else:
            bar_width = complete_bar_width * progress
        if grey:
            bar_colour = (200, 200, 200)
        else:
            bar_colour = (255, 0, 0)

        self.cursor_x = bar_width + bar_x  # starting X of bar + size of progress bar
        self.red_bar = pygame.draw.rect(screen, bar_colour, pygame.Rect((bar_x, bar_y), (bar_width, bar_height_pix)))

    def get_slider_rect(self):
        return self.slider_rect

    def get_cursor_rect(self):
        return self.cursor_rect

    def draw_cursor(self, screen):
        """uses progress to move cursor to where it should be. THIS SHOULD ALWAYS BE AFTER 'draw_progress_bar()' """
        cursor_y = self.img_y + int(self.half_cursor_height)  # spawn cursor at half it's length to allign it
        self.cursor_rect.center = (self.cursor_x, cursor_y)
        screen.blit(self.cursor_image, self.cursor_rect)

    def get_event(self, event, mouse_pos, track_info=["", 0.0, 999]):  # track info = title, elapsed_time, total_time
        """handle events """
        mouse_on_cursor = self.cursor_rect.collidepoint(mouse_pos)
        mouse_x = mouse_pos[0]

        # Once slider is grabbed
        if event.type == self.inputDown and mouse_on_cursor:
            self.on_click()
            if self.slider_range[0] < mouse_x < self.slider_range[1]:
                self.slider_being_held = True

        # Once slider is released
        if event.type == self.inputUp and self.slider_being_held:
            # expand track_info
            track_title = track_info[0]
            track_time = track_info[1]
            track_total_time = track_info[2]

            time_to_start = track_total_time * self.bar_overwrite  # get overwrite in secs

            self.on_release(track_title, time_to_start)
            self.slider_being_held = False
            self.bar_overwrite = 2.0  # reset slider overwrite. Use a weird number, as this should only occur if i've made a coding error.

        # While slider is being dragged
        if self.slider_being_held:  # If the user is dragging the cursor
            if self.slider_range[0] < mouse_x < self.slider_range[1]:  # If mouse x within slider x range
                self.bar_overwrite = (mouse_x - self.slider_range[
                    0]) / self.slider_len  # Get percentage of where mouse is compared to len of bar
            elif mouse_x > self.slider_range[1]:
                self.bar_overwrite = 1.0  # Don't let cursor move past slider bar
            elif mouse_x < self.slider_range[0]:
                self.bar_overwrite = 0.0  # Don't let cursor move past slider bar

        return self.slider_being_held

class VolumeSlider():
    """ Class that generates a slider object using pygame."""

    def __init__(self, pygame, location, default_vol = 0,  scale = 1, min_val = 0, max_val=100, on_release=None,
                 inputMode = 1):
        """ Create the slider object at given location with given scale.
            pygame = pygame, pass in initialised pygame, so we don't reinitialise
            location = list or tuple, x and y pos
            default_vol = float or int, the volume the slider should start at in percentage. default = 0, range 0 - 1
            scale = float or int, default is 1
            min_max = list / tuple, values for min and max percentage. so that you can have custom ranges
                (e.g (70, 90) would have the max return of the slider be 90, and the minimum be 70)
                default = (0,100)
            on_release = func, this should be a function / method that runs on release of the mouse
        """

        self.pygame = pygame

        # Load images
        path_to_graphics = os.path.join(os.path.dirname(__file__), "game_assets/graphics/")

        # Slider box
        slider_box_path = path_to_graphics + "volume_slider_box.png"
        slider_box_image = self.pygame.image.load(slider_box_path).convert_alpha()
        slider_box_grey = General().convert_to_grey(slider_box_image)

        # Scale and get rect of slider box
        slider_box_x = location[0]
        slider_box_y = location[1]
        self.slider_box_w = int(slider_box_image.get_width() * scale)
        self.slider_box_h = int(slider_box_image.get_height() * scale)

        scaled_size = (self.slider_box_w, self.slider_box_h)
        self.slider_box = self.pygame.transform.scale(slider_box_image, scaled_size)
        self.slider_box_grey = self.pygame.transform.scale(slider_box_grey, scaled_size)
        self.slider_box_rect = self.pygame.Rect(slider_box_x, slider_box_y, self.slider_box_w, self.slider_box_h)

        # Load and scale slider
        slider_path = path_to_graphics + "volume_slider_slider.png"
        slider_image = self.pygame.image.load(slider_path).convert_alpha()
        slider_grey = General().convert_to_grey(slider_image)

        self.slider_x, self.slider_y = self.CalculateSliderPos(default_vol)
        self.slider_w = int(slider_image.get_width() * scale)
        self.slider_h = int(slider_image.get_height() * scale)

        scaled_size = (self.slider_w, self.slider_h)
        self.slider = self.pygame.transform.scale(slider_image, scaled_size)
        self.slider_grey = self.pygame.transform.scale(slider_grey, scaled_size)
        self.slider_rect = self.pygame.Rect(self.slider_x, self.slider_y - (self.slider_h /2), self.slider_w, self.slider_h)

        # Init vars
        self.scale = scale
        self.drag = False
        self.on_release = on_release
        self.type = "VolumeSlider"
        self.min_value = min_val
        self.max_value = max_val
        self.current_vol = 0

        # Set inputs to either touch or mouse
        self.input_mode = inputMode  # input mode 1 for touch, 2 for mouse
        if self.input_mode == 2:
            self.inputUp = pygame.MOUSEBUTTONUP
            self.inputDown = pygame.MOUSEBUTTONDOWN
            self.inputMotion = pygame.MOUSEMOTION
        else:
            self.inputUp = pygame.FINGERUP
            self.inputDown = pygame.FINGERDOWN
            self.inputMotion = pygame.FINGERMOTION

        # initialise text
        self.init_text()

    def CalculateSliderPos(self, percentage):
        """ Move the slider along the box according to the percentage passed in
        Use the percentage passed in, as well as slider box rect to set the slider
        to where it should be along the box"""
        # TODO this is inaccurate as it doesnt places via top left corner, while rest of code uses center
        x,y,w,h = self.slider_box_rect
        slider_x = x
        slider_y = y + (h - (h * percentage/100))# such that 0% = bottom, 100% = top
        return slider_x, slider_y

    def CalculateVolumePercent(self):
        """ Use pos of slider box and slider to get equivalent percentage.
         percentage from bottom to top the slider is at. i.e slider at top == 1"""
        slider_min = self.slider_box_rect.top
        my_slider_y = self.slider_rect.center[1] - slider_min # slider pos - min to get slider in respect of slider box
        slider_percent =  1 - (my_slider_y / self.slider_box_rect.height) # flip percent, so sliding upwards increases %
        volume_percent = int( self.min_value + round(slider_percent * (self.max_value - self.min_value), 0)) # get percentage of min and max bounds

        return volume_percent

    def handle_event(self, event, mouse_pos):
        # Handle on click
        if event.type == self.inputDown:
            if self.slider_rect.collidepoint(mouse_pos):
                self.drag = True
        # Handle mouse release
        elif event.type == self.inputUp:
            # Skip if never clicked on vol slider
            if not self.drag:
                return
            # If they're dragging, let them trigger on release
            if self.on_release != None:
                self.current_vol = self.CalculateVolumePercent()
                self.on_release(self.current_vol)
                self.current_vol_obj.set_text(str(self.current_vol))
            self.drag = False
        elif event.type == self.inputMotion:
            # Skip if never clicked on vol slider
            if not self.drag:
                return
            # Move slider within it's bounds if mouse was held on it
            slider_max = (self.slider_box_rect.top + self.slider_box_rect.height)
            slider_min = self.slider_box_rect.top
            if slider_min < mouse_pos[1] < slider_max:
                self.slider_rect.y = mouse_pos[1] # Place center at mouse
            elif slider_max < mouse_pos[1]:
                self.slider_rect.y = slider_max - (self.slider_h/2)
            elif mouse_pos[1] < slider_min:
                self.slider_rect.y = slider_min - (self.slider_h/2)
            # update slider text to match
            self.current_vol_obj.set_y(self.slider_rect.y + (self.current_vol_obj.textRect.h / 2))
            self.current_vol_obj.set_text(" ")

    def render(self, screen, grey = False):
        if grey:
            # Draw grey graphics onto screen
            screen.blit(self.slider_box_grey, self.slider_box_rect)
            screen.blit(self.slider_grey, self.slider_rect)
            self.draw_text(screen, grey)
        else:
            # Draw onto screen
            screen.blit(self.slider_box, self.slider_box_rect)
            screen.blit(self.slider, self.slider_rect)
            self.draw_text(screen, grey)

    def init_text(self):
        font_size = int(24 * self.scale)
        # Create objects
        self.max_vol_obj = TextObject(None, None, str(self.max_value), location=(0, 0), font_size=font_size)
        self.min_vol_obj = TextObject(None, None, str(self.min_value), location=(0, 0), font_size=font_size)
        self.current_vol_obj = TextObject(None, None, str(self.current_vol), location=(0, 0), font_size=font_size)
        #Move objects to start positions
        self.max_vol_obj.set_pos(
            (self.slider_box_rect[0],
             self.slider_box_rect[1] - (self.max_vol_obj.textRect.h)))
        self.min_vol_obj.set_pos(
            (self.slider_box_rect[0] + (self.min_vol_obj.textRect.w / 2),
             self.slider_box_rect[1] + self.slider_box_rect.height))
        self.current_vol_obj.set_pos(
            (self.slider_box_rect[0] + self.slider_box_rect[2]  + self.current_vol_obj.textRect.w + 10,
             self.slider_rect.center[1]))

    def draw_text(self, screen, window, re_render = False):
        self.max_vol_obj.render(screen, window, re_render)
        self.min_vol_obj.render(screen, window, re_render)
        self.current_vol_obj.render(screen, window, re_render)

class LineObject():
    """Create object orientated lines that we can store and draw later, for screen order """

    def __init__(self, start_pos, end_pos, colour = (255,100,100), width = 3):

        self.start_pos = start_pos
        self.end_pos = end_pos
        self.colour = colour
        self.width = width

    def render(self, screen):
        pygame.draw.line(screen, self.colour, self.start_pos, self.end_pos, self.width)


###########################################################END OF LIBRARY############################################################