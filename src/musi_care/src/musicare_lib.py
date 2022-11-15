#!/usr/bi   n/env python
#
# Library of useful classes that the other games use
#
#
import rospy
import pygame
import pygame.freetype  # some reason fixed events repeating themselves
import os
import math
import random
import wave
import contextlib
from pydub import AudioSegment
from musi_care.msg import SongData
from musi_care.srv import sound_player_srv
from musi_care.srv import qt_command


# TODO REPLACE ALL GREY SCALE VERSIONS TO QUICK SCRIPT OF CV2.convert to greyscale

# rospy.init_node('musicare_lib', anonymous=False)
# rospy.loginfo("musicare_lib launched successfully")

######################################################Uncategorised#################################################################
class General():

    def __init__(self):
        pass

    def save_data(self, user_id, game_name, level):
        """Save the user's level data to file
        How save system works:
        Data is all in a dictionary of dictionaires,        
        Example data:
        {
            {"guess_the_mood":
                {1:
                    {"time_taken":123
                    "accuracy":100
                    "hints_given":2
                    },
                2:
                    {"time_taken":123
                    "accuracy":100
                    "hints_given":2
                    }
                }
            },
            
            {"fix_the_song":
                {1:
                    {"time_taken":123
                    "accuracy":100
                    "hints_given":2
                    }
                }
            },
        }
        
        
        """
        save_location = r"./user_saves/"
        save_name = save_location  # + user_id
        print(os.path.exists(save_location))
        # /user_saves


######################################################DraggableButton#################################################################

class Behaviours():
    """Class to store and return saved / repeatable behaviours other than builtin with QT"""

    def __init__(self, pygame, path_to_music):
        self.timeout_started = False
        self.timer = TimeFunctions()
        self.sound_manager = SoundManager(path_to_music)
        self.command_manager = QTManager()
        self.pygame = pygame
        self.paused = False
        self.speaking_timer_id = ""

    def qt_reminder(self, events, music_playing=True, timeout_message="If you are stuck. please click the clue button"):
        """If x seconds of no inputs pass, qt should pause music and say something"""
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
                if event.type == self.pygame.MOUSEBUTTONDOWN:
                    print("starting timer")
                    self.timer.CreateTimer(timer_id, timeout_time, verbose=False)  # restart timer
                    self.talking = False

        # check if we're still paused and if QT is still speaking
        if self.talking and music_playing:
            if self.command_manager.robo_timer.CheckTimer(self.speaking_timer_id):
                self.talking = False
                self.sound_manager.unpause()
                self.timeout_started = False  # so that we start a new timer

    def get_agreements(self):  # List of all sayings when QT agrees EG yes, correct
        sayings = ["Yes!", "Correct!", "That's right!", "That's correct!", "Great, that's right!",
                   "Good job, That is the right answer!"]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        return saying

    def get_disagreements(self):  # List of all sayings when QT disagree EG no, sorry.
        sayings = ["No, Please try again...", "Sorry, no, please try again...", "No, that's not correct...",
                   "I'm sorry that is not right, Please try again...", "Sorry, that is not the right answer..."]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        return saying

    def get_praise(self):
        sayings = ["Amazing, that is the right answer!", "Great job!", "Alright, that's correct!", "Wow, well done!",
                   "Nice! Well done!", "You did really well on that one!", "You are a natural",
                   "You made that one look easy!"]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        return saying

    def get_incorrect(self):
        sayings = ["Okay, next question", "On to the next question", ]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        return saying

    def get_next_level(self):
        sayings = ["Okay, next level", "On to the next level", "Lets play the next level!", "Now onto the next one!",
                   "Lets play another one!"]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        return saying

    def get_help(self):
        sayings = ["Okay, next level", "On to the next level", "Lets play the next level!", "Now onto the next one!",
                   "Lets play another one!"]
        ind = random.randint(0, len(sayings) - 1)
        saying = sayings[ind]
        return saying


#####################################################General Levels##################################################################

class StandardLevels():
    """Class to draw basic screens multiple games use, such as yes or no screen """

    def __init__(self, window, window_center, pygame, path_to_music):
        self.window = window
        self.window_center = window_center
        self.pygame = pygame
        self.renderer = Renderer(window, window_center)
        self.animation_manager = AnimationManager(pygame)
        self.command_manager = QTManager()
        self.sound_manager = SoundManager(path_to_music)
        self.timer = TimeFunctions()

    def yes_or_no_screen(self, text, run, background_colour):
        """Screen for Yes or No questions"""
        # Variables
        this_file_path = os.path.dirname(__file__)
        path_to_imgs = 'game_assets/graphics'
        yes_img_path = os.path.join(this_file_path, path_to_imgs, "Yes_button.png")
        no_img_path = os.path.join(this_file_path, path_to_imgs, "No_button.png")

        # Create buttons
        yes = Button(yes_img_path, yes_img_path, (225, 500), self.pygame, scale=2.2)
        no = Button(no_img_path, no_img_path, (1625, 500), self.pygame, scale=2.2)

        # Have QT act
        self.command_manager.qt_emote("talking")
        self.command_manager.qt_say("Should i explain the rules of the game called, 'Guess, the  mood'?")

        while not rospy.is_shutdown() and run:

            # Event handling
            for event in self.pygame.event.get():  # Check if the user clicks the X
                if event.type == self.pygame.QUIT:
                    return "QUIT"
                elif (
                        event.type == self.pygame.MOUSEBUTTONUP):  # on mouse release play animation to show where cursor is
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
            self.renderer.DrawTextCentered(text, font_size=100, y=200)  # top text
            self.animation_manager.DrawTouchAnimation(self.window)  # also draw touches
            self.pygame.display.update()  # Update all drawn objects

    def QTSpeakingScreen(self, qt_say, run, background_colour, should_gesture=True, gesture="explain_right", ):
        """Method displays background and text in centre"""
        text_display = "Please listen to QT robot"

        if run:  # Dont start this screen if the previous screen wanted to close out the game

            self.command_manager.qt_emote("talking")  # show mouth moving
            speaking_timer_id = self.command_manager.qt_say(
                qt_say)  # says text we give it, and starts an internal timer that we can check on

            qt_speaking = True  # used to tell us when to stop blocking
            if should_gesture:
                self.command_manager.qt_gesture(gesture)

            while qt_speaking and not rospy.is_shutdown() and run:
                # check for quit
                for event in self.pygame.event.get():
                    # Check if the user clicks the X
                    if event.type == self.pygame.QUIT:
                        return False
                    elif (
                            event.type == self.pygame.MOUSEBUTTONUP):  # on mouse release play animation to show where cursor is
                        mouse_pos = self.pygame.mouse.get_pos()
                        self.animation_manager.StartTouchAnimation(
                            mouse_pos)  # tell system to play animation when drawing

                # Draw background and objects
                self.renderer.DrawBackground(background_colour)
                self.renderer.DrawTextCentered("Please listen to QT robot", font_size=70)
                self.animation_manager.DrawTouchAnimation(self.window)  # also draw touches
                self.pygame.display.update()  # Update all drawn objects

                if self.command_manager.robo_timer.CheckTimer(speaking_timer_id):  # If our timer is done
                    qt_speaking = False
                    return True

    def QTSpeakingPopupScreen(self, qt_say, graphics, run, background_colour, should_gesture=True,
                              gesture="explain_right"):
        """Method displays all our gui with a popup infront with text in centre
           graphics = a dict of functions that are saved with the parameters needed to render them
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
            popup = Button(popup_path, popup_path_grey, (700, 550), self.pygame, scale=1.5)
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
                            event.type == self.pygame.MOUSEBUTTONUP):  # on mouse release play animation to show where cursor is
                        mouse_pos = self.pygame.mouse.get_pos()
                        self.animation_manager.StartTouchAnimation(
                            mouse_pos)  # tell system to play animation when drawing

                # Draw background and objects
                self.renderer.DrawBackground(background_colour)
                for key in graphics:  # run each partial function
                    graphics[key]()  # run as func
                popup.render(self.window)
                self.renderer.DrawTextCentered("Please listen to QT robot", font_size=70)
                self.animation_manager.DrawTouchAnimation(self.window)  # also draw touches
                self.pygame.display.update()  # Update all drawn objects

                if self.command_manager.robo_timer.CheckTimer(
                        speaking_timer_id):  # If our timer's internal timer is done
                    qt_speaking = False  # unessecary but might as well
                    return True

    def tap_to_continue(self, run, background_colour,
                        text_display="Please tap the screen when you are ready to start the level.", qt_say=None,
                        should_gesture=True, gesture="explain_right"):
        """Screen that waits until tap, non blocking even if QT speaking"""

        if run:  # Dont start this screen if the previous screen wanted to close out the game

            # Handle speaking
            if qt_say != None:
                self.command_manager.qt_emote("talking")  # show mouth moving
                speaking_timer_id = self.command_manager.qt_say(
                    qt_say)  # says text we give it, and starts an internal timer that we can check on

            # Handle gesturing
            if should_gesture:
                self.command_manager.qt_gesture(gesture)

            clicked = False
            while not clicked and not rospy.is_shutdown() and run:

                # check for quit
                for event in self.pygame.event.get():
                    # Check if the user clicks the X
                    if event.type == self.pygame.QUIT:
                        return False
                    elif (
                            event.type == self.pygame.MOUSEBUTTONUP):  # on mouse release play animation to show where cursor is
                        clicked = True
                        self.animation_manager.StartTouchAnimation(
                            self.pygame.mouse.get_pos())  # tell system to play animation when drawing
                # Draw background and objects
                self.renderer.DrawBackground(background_colour)
                speed_coefficient = 1  # half sine freq
                dot_decider = math.sin(speed_coefficient * rospy.get_time())
                if dot_decider < -1 / 3:
                    self.renderer.DrawTextCentered(text_display, font_size=70)
                elif dot_decider < 1 / 3:
                    self.renderer.DrawTextCentered(text_display + ".", font_size=70)
                else:
                    self.renderer.DrawTextCentered(text_display + "..", font_size=70)
                self.animation_manager.DrawTouchAnimation(self.window)  # also draw touches
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
                                event.type == self.pygame.MOUSEBUTTONUP):  # on mouse release play animation to show where cursor is
                            self.animation_manager.StartTouchAnimation(
                                self.pygame.mouse.get_pos())  # tell system to play animation when drawing
                    # Draw background and objects
                    self.renderer.DrawBackground(background_colour)
                    if prelim_msg != None and second == seconds:
                        self.renderer.DrawTextCentered(qt_speech, font_size=100)
                    else:
                        self.renderer.DrawTextCentered(qt_speech, font_size=500, font_colour=(150, 250, 150))

                    self.animation_manager.DrawTouchAnimation(self.window)  # also draw touches
                    self.pygame.display.update()  # Update all drawn objects

                    if prelim_msg != None and second == seconds:  # if 1st loop, check a different timer
                        second_passed = self.command_manager.robo_timer.CheckTimer(timer_id)
                    else:
                        second_passed = self.timer.CheckTimer(timer_id)

            return True


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Text objects~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class TextObject():

    def __init__(self,window, window_center, text, location=None, cen_x = False, cen_y=False, font_size=30, font_colour=(255,255,255)):
        self.window = window
        self.font = pygame.font.Font('freesansbold.ttf', font_size)
        self.text = self.font.render(text, False, font_colour)
        self.textRect = self.text.get_rect()
        if location != None:
            self.set_pos(location)
        if cen_x:
            self.textRect[0] = window_center[0] - (self.textRect[2]/2)
        if cen_y:
            self.textRect[1] = window_center[1] - (self.textRect[3]/2)
        self.type = "TextObject"

    def render(self, window):
        """handle drawing text"""
        self.window.blit(self.text, self.textRect)

    def set_pos(self, pos):
        self.textRect[0] = pos[0]
        self.textRect[1] = pos[1]

#####################################################Renderer##################################################################

class Renderer():
    """Class to render common things, such as background """

    def __init__(self, window, window_center):
        self.window = window
        self.window_center = window_center

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
        with contextlib.closing(wave.open(song_path, 'r')) as f:
            frames = f.getnframes()
            rate = f.getframerate()
            duration = frames / float(rate)
            return (duration)

    def slice_song(self, num_segments, target_song_to_split, distracting_songs=None):
        """Method that chops up a song and creates segments from it, this uses the file name, so you can use 2 differenty files and append the returned variabels together to jumble multiple songs
        target_song should be the path to a single song
        distracting songs should be a list of paths to songs, even if only 1 song is used
        """

        # Define variables
        this_path = os.path.dirname(__file__)
        music_filepath = r"/game_assets/music/"
        correct_segs = []
        distract_segments = []
        song_path = this_path + music_filepath + target_song_to_split
        self.path_to_save = this_path + music_filepath + "temp/"
        total_wav_len = (self.return_wav_lenth(song_path)) * 1000  # convert to millisecond
        slice_size = total_wav_len / num_segments
        prev_slice = 0

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

    def __init__(self):
        self.robo_timer = TimeFunctions()

    def init_robot(self, arm_vel):
        """Method to init robot parameters"""
        # Set control mode, incase they were changed before hand
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

    def send_qt_command(self, speech=None, gesture=None, emote=None, command_blocking=False):
        """Neatens and simplifies sending commands to QT 
        if we want to use multiple functions of QT at once and dont care about tracking time taken, we should use this method instead of the others
        """
        if speech != None:  # do qt_speak
            # print("sending speech req")
            rospy.wait_for_service('/qt_command_service')
            command_controller = rospy.ServiceProxy('/qt_command_service', qt_command)
            command_complete = command_controller("tts", speech, command_blocking)
            return command_complete
        if gesture != None:  # do qt_speak
            rospy.wait_for_service('/qt_command_service')
            command_controller = rospy.ServiceProxy('/qt_command_service', qt_command)
            command_complete = command_controller("gesture", gesture, command_blocking)
            return command_complete
        if emote != None:  # do qt_speak
            rospy.wait_for_service('/qt_command_service')
            command_controller = rospy.ServiceProxy('/qt_command_service', qt_command)
            command_complete = command_controller("emote", emote, command_blocking)
            return command_complete

    def qt_say_blocking(self, text):
        """Makes QT say something, then makes you wait until the speaking is done"""
        timer_len = len(text) * 0.08  # 0.2s per letter 4 letter word is given 0.8s to be said
        timer_id = "QT_SAY_BLOCKING"
        self.robo_timer.CreateTimer(timer_id, timer_len)  # creates timer with ID 1 for 8s
        self.send_qt_command(speech=text)
        talking = True
        while talking and not rospy.is_shutdown():
            if self.robo_timer.CheckTimer("QT_SAY_BLOCKING"):  # if our timer is done
                talking = False

    def qt_say(self, text):
        """Makes QT say something, then makes starts a timer until the speaking is done"""
        timer_len = len(text) * 0.09  # 0.09s per letter. A 4 letter word is given 0.36 secs to be said
        timer_id = "QT_SAY"
        self.robo_timer.CreateTimer(timer_id, timer_len)  # Creates timer with ID 1 for 8s
        self.send_qt_command(speech=text)  # Have qt say the text
        return timer_id

    def qt_gesture(self, req_gesture):
        """Make QT do gesture, non-blocking """
        self.send_qt_command(gesture=req_gesture)

    def qt_emote(self, req_emote):
        """Make QT emote, non-blocking"""
        self.send_qt_command(emote=req_emote)


#####################################################AnimationManager##################################################################

class AnimationManager():
    """Class for misselaneous functions"""

    def __init__(self, pygame):
        self.pygame = pygame
        self.play_touch_animation = False

    def StartTouchAnimation(self, mouse_pos):
        """Start animation """
        # animation vars
        self.colour = (180, 180, 180)  # light grey
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
            if time_since_start < 400:  # animation time = under half a sec
                scalar = time_since_start / 4  # low numbers = faster growth time
                max_size = 50
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

    def __init__(self, image_path, image_greyscale_path, x_y_locations, pygame, return_info ={}, scale=1, unique_id="", on_click=object,
                 on_release=object):

        if not os.path.exists(image_path):
            print("File does not exist path = ", image_path)
        else:
            pass
            # print("File located at",image_path)
        self.pygame = pygame
        raw_image = self.pygame.image.load(image_path).convert_alpha()
        grey_scaled_raw_image = self.pygame.image.load(image_greyscale_path).convert_alpha()
        img_x = x_y_locations[0]
        img_y = x_y_locations[1]
        img_w = int(raw_image.get_width() * scale)
        img_h = int(raw_image.get_height() * scale)
        scaled_size = (img_w, img_h)
        self.image = self.pygame.transform.scale(raw_image, scaled_size)
        self.image_greyscale_path = self.pygame.transform.scale(grey_scaled_raw_image, scaled_size)
        self.rect = self.pygame.Rect(img_x, img_y, img_w, img_h)
        if unique_id == "":
            self.id = rospy.get_time()  # unique ID for each button based on time when made
        else:
            self.id = unique_id
        self.return_info = return_info
        self.type = "Button"

    def render(self, screen, grey=False):
        """Draw button onto given screen, either as greyscale or coloured"""
        if grey:  # if we get a request to pause show greyscaled version
            screen.blit(self.image_greyscale_path, self.rect)
        else:
            screen.blit(self.image, self.rect)

    def get_event(self, event, mouse_pos):
        """returns if button was pressed"""
        # If the mouse clicked this button
        if event.type == self.pygame.MOUSEBUTTONUP and event.button == 1:
            if self.rect.collidepoint(mouse_pos):
                return True
            else:
                return False
        else:
            return False

    def set_pos(self, newpos):
        """Takes a list of (x,y) and sets rect"""
        self.rect[0] = newpos[0]
        self.rect[1] = newpos[1]

    def set_info(self, info):
        self.return_info = info

    def get_rect(self):
        """returns object rect"""
        return self.rect


######################################################ToggleButton#################################################################

class ToggleButton():
    """Class to load images that serve as buttons """

    def __init__(self, default_image_path, toggled_image_path, default_image_grey, toggled_image_grey, x_y_locations,
                 pygame, scale=1, unique_id="", return_info="", when_toggle_on=object, when_toggle_off=object):
        # Set vars
        self.pygame = pygame
        self.highlighted = False
        self.return_info = return_info
        self.toggle_state = False
        self.when_toggle_on = when_toggle_on
        self.when_toggle_off = when_toggle_off

        # load imges
        raw_image = self.pygame.image.load(default_image_path).convert_alpha()
        raw_img_grey = self.pygame.image.load(default_image_grey).convert_alpha()
        toggled_raw_image = self.pygame.image.load(toggled_image_path).convert_alpha()
        toggled_raw_grey = self.pygame.image.load(toggled_image_grey).convert_alpha()

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

    def render(self, screen, grey=False):
        """Draw image onto screen"""
        if self.toggle_state:
            if grey:
                screen.blit(self.toggled_image_grey,
                            self.rect)  # TODO replace this with the greyscaled version of this image
            else:
                screen.blit(self.toggled_image, self.rect)
        else:
            if grey:
                screen.blit(self.image_grey, self.rect)  # TODO replace this with the greyscaled version of this image
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

    def get_rect(self):
        return self.rect


######################################################ToggleButton#################################################################

class PausePlayButton():
    """All functionality of toggle button, but with an option to replace with a 3rd image"""

    def __init__(self, pause_path, pause_path_grey, play_path, play_path_grey, rewind_path, rewind_path_grey,
                 x_y_locations, pygame, scale=1, unique_id="", on_pause=object, on_play=object):

        # Set vars
        self.pygame = pygame
        self.highlighted = False
        self.playing = True  # starts paused
        self.on_pause = on_pause
        self.on_play = on_play
        self.rewind_toggle = False  # show rewind or not

        # load imges
        raw_play = self.pygame.image.load(play_path).convert_alpha()
        raw_play_grey = self.pygame.image.load(play_path_grey).convert_alpha()
        raw_pause = self.pygame.image.load(pause_path).convert_alpha()
        raw_pause_grey = self.pygame.image.load(pause_path_grey).convert_alpha()
        raw_rewind = self.pygame.image.load(rewind_path).convert_alpha()
        raw_rewind_grey = self.pygame.image.load(rewind_path_grey).convert_alpha()

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

    def render(self, screen, grey=False):
        """Draw image onto screen"""
        if self.rewind_toggle:
            if grey:
                screen.blit(self.rewind_grey, self.rect)  # TODO replace this with the greyscaled version of this image
            else:
                screen.blit(self.rewind, self.rect)
        else:
            if self.playing:
                if grey:
                    screen.blit(self.pause_grey,
                                self.rect)  # TODO replace this with the greyscaled version of this image
                else:
                    screen.blit(self.pause, self.rect)
            else:
                if grey:
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
            if event.type == self.pygame.MOUSEBUTTONUP:
                return self.toggle_pause()  # flip paused and play
        return self.playing  # return the state we were in

    def get_rect(self):
        return self.rect


######################################################DragableButton#################################################################

class DraggableButton():
    """Class to load images that serve as buttons that can be dragged and dropped """

    def __init__(self, image_path, toggled_image_path, default_image_grey, toggled_image_grey, x_y_locations, pygame,
                 scale=1, return_info={}, when_toggle_on=object, when_toggle_off=object):
        self.pygame = pygame

        # load images
        raw_image = self.pygame.image.load(image_path).convert_alpha()
        toggled_raw_image = self.pygame.image.load(toggled_image_path).convert_alpha()
        raw_image_grey = self.pygame.image.load(default_image_grey).convert_alpha()
        toggled_raw_image_grey = self.pygame.image.load(toggled_image_grey).convert_alpha()

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
        self.highlighted = False
        self.block = False
        self.return_info = return_info
        self.toggle = False
        self.when_toggle_on = when_toggle_on
        self.when_toggle_off = when_toggle_off
        self.mouse_is_held = False
        self.type = "DraggableButton"

    def render(self, screen, grey=False):
        """Draw image onto screen"""
        if self.toggle:
            if grey:
                screen.blit(self.toggled_image_grey,
                            self.rect)  # TODO replace this with the greyscaled version of this image
            else:
                screen.blit(self.toggled_image, self.rect)
        else:
            if grey:
                screen.blit(self.image_grey, self.rect)  # TODO replace this with the greyscaled version of this image
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

    def get_event(self, event, mouse_pos):
        """Button event handle, if mouse release, then toggle"""
        mouse_on_button = self.rect.collidepoint(mouse_pos)
        if mouse_on_button:
            if event.type == self.pygame.MOUSEBUTTONDOWN and not self.mouse_is_held:
                self.mouse_is_held = True  # Tells us that the mouse was clicked during this event handling
                self.initial_pos = mouse_pos
            if self.mouse_is_held:
                if event.type == self.pygame.MOUSEBUTTONUP and mouse_pos == self.initial_pos:
                    self.mouse_is_held = False
                    return self.toggle_toggle(), self.rect
                elif event.type == self.pygame.MOUSEBUTTONUP and mouse_pos != self.initial_pos:
                    self.mouse_is_held = False  # mouse was released somewhere else
                else:
                    self.rect.x = mouse_pos[0] - self.img_w / 2
                    self.rect.y = mouse_pos[1] - self.img_h / 2

        return self.toggle, self.rect


######################################################HorizontalSlider#################################################################

class HorizontalSlider():
    """Class to handle all functions of the horizontal sliders"""

    def __init__(self, image_path_slider, image_path_cursor, x_y_locations, scale=1, on_click=object, on_release=object,
                 music_filepath="/game_assets/music/"):
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
                                       slider_img_h)  # make collision box around it according it it's size

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
        if event.type == pygame.MOUSEBUTTONDOWN and mouse_on_cursor:
            self.on_click()
            if self.slider_range[0] < mouse_x < self.slider_range[1]:
                self.slider_being_held = True

        # Once slider is released
        if event.type == pygame.MOUSEBUTTONUP and self.slider_being_held:
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

###########################################################END OF LIBRARY############################################################
