#!/usr/bin/env python

#libraries
from guess_mood import Guess_The_Mood_Game
from fix_song import Fix_The_Song_Game
from clap_beat import Clap_To_Beat_Game
from simon_says_clap import Simon_Says_Clap_Game
from musicare_lib import QTManager
from musicare_lib import StandardLevels
from musicare_lib import Behaviours
from std_msgs.msg import Float64MultiArray
import rospy
import time

#init node
rospy.init_node('Pilot', anonymous=False)
rospy.loginfo("Pilot node launched successfully")

#create arm msgs to move qt arms
arm_up_right = [20, -59.599998474121094, -40.70000076293945] #motor pos for right arm to be in air
arm_up_left = [-20, -59.599998474121094, -40.70000076293945] #motor pos for right arm to be in air
arms_up = '[["right_arm", "left_arm"], [{}, {}]]'.format(arm_up_right,arm_up_left)
arm_right_msg = Float64MultiArray()
arm_left_msg = Float64MultiArray()
arm_right_msg.data = arm_up_right
arm_left_msg.data = arm_up_left

#init games
user_id = "Jwaad"
game = Guess_The_Mood_Game(user_id)

levels = StandardLevels(game.window, game.window_center, game.pygame, game.music_filepath)
qt_manager = QTManager(levels = levels)
behave = Behaviours(game.pygame, game.music_filepath)

background = (100,100,100)
run = True

games_to_play = [1, 2, 3, 4]

#Pause until they're ready
levels.tap_to_continue(run,background, text_display="Tap when you want to start.")

# Guess the mood
if 1 in games_to_play:
    qt_manager.send_qt_command(gesture="wave", emote="grin")
    qt_manager.qt_say_blocking("Hi!,,, My name is Q-T!..", black_screen = True)#" #I am going to be playing 3 games with you today, Each game will be music themed and will only last for 2 minutes.")
    qt_manager.qt_say_blocking("Hope you are well!...", black_screen = True)
    qt_manager.qt_say_blocking("Would you like to play some games with me?...")
    run, consent = levels.yes_or_no_screen("Would you like to play some games with me?", run, background, silent=True)
    while not consent:
        # If they said no, say "please?"
        run, consent = levels.yes_or_no_screen("Can you please play with me?", run, background)
        levels.black_screen(run)
    qt_manager.send_qt_command(gesture="happy", emote="grin")
    qt_manager.qt_say_blocking("Great!... Then, I am going to start the first game now!...", black_screen = True)
    game.Main()
    for level in range(2,4):
        qt_manager.qt_say_blocking(behave.get_next_level(), black_screen = True)
        game.play_level(run, "easy", level)

# Fix the song
if 2 in games_to_play:
    game = Fix_The_Song_Game(user_id, reduce_screen = False)
    qt_manager.send_qt_command(gesture="happy", emote="grin")
    qt_manager.qt_say_blocking("Great job on that last game... Lets play the next one now...!", black_screen = True)
    game.Main("easy", 1)
    qt_manager.qt_say_blocking(behave.get_next_level(), black_screen=True)
    for level in range(2, 4):
        if level == 3:
            qt_manager.qt_say_blocking("For this next level, I will add in a fake song to make it harder!", black_screen = True)
        game.level_loader.tap_to_continue(run, background)
        game.play_music_blocking("easy", level)
        game.level_loader.countdown(3, run, background, prelim_msg="Lets Fix The Song!")
        game.play_level(difficulty= "easy", level_num=level)

# Clap to the beat
if 3 in games_to_play:
    game = Clap_To_Beat_Game(user_id, reduce_screen=False)
    qt_manager.send_qt_command(gesture="happy", emote="grin")
    time.sleep(1)
    qt_manager.qt_say_blocking("I hope you enjoyed that game! Lets move on to the next one now...", black_screen = True)
    time.sleep(2.5)
    levels.yes_or_no_screen("But first. Will you help me get my drum?", run, background)
    ready = False
    while not ready:
        # Move arms up
        qt_manager.qt_actuate(arms_up)
        run, ready = levels.yes_or_no_screen("Is it set up now?", run, background)
        levels.black_screen(run)
    # Play level 1
    game.Main(level=4)
    time.sleep(2)
    qt_manager.send_qt_command(emote="grin")
    qt_manager.qt_say_blocking("I had fun! I hope you did too!", black_screen=True)


# Simon says
if 4 in games_to_play:
    game = Simon_Says_Clap_Game(user_id, reduce_screen=False)
    qt_manager.send_qt_command(gesture="happy", emote="grin")
    qt_manager.qt_say_blocking("Great!.. One last game now!", black_screen = True)
    # Play level 1
    game.Main(level = 4)
    time.sleep(2)
    # Get rid of drum so we can emote
    qt_manager.send_qt_command(emote="grin")
    levels.yes_or_no_screen("That was so fun! That's the last game. Can you please remove my drum?", run, background)
    ready = False
    while not ready:
        # Move arms up
        qt_manager.qt_actuate(arms_up)
        run, ready = levels.yes_or_no_screen("Is the drum out of the way?", run, background)
        levels.black_screen(run)

qt_manager.send_qt_command(gesture="wave", emote="grin")
qt_manager.qt_say_blocking("Thank you for playing with me!", black_screen = True)
qt_manager.qt_say_blocking("I am grateful for you spending time with me!", black_screen = True)
print("Shutting game down")



