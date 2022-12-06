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

#init node
rospy.init_node('Pilot', anonymous=False)
rospy.loginfo("Pilot node launched successfully")
right_arm_pos_pub = rospy.Publisher('/qt_robot/right_arm_position/command', Float64MultiArray, queue_size=10)
left_arm_pos_pub = rospy.Publisher('/qt_robot/left_arm_position/command', Float64MultiArray, queue_size=10)

#create arm msgs to move qt arms
arm_up_right = [-5, -59.599998474121094, -40.70000076293945] #motor pos for right arm to be in air
arm_up_left = [5, -59.599998474121094, -40.70000076293945] #motor pos for right arm to be in air
arm_right_msg = Float64MultiArray()
arm_left_msg = Float64MultiArray()
arm_left_msg.data = arm_up_right
arm_left_msg.data = arm_up_left

#init games
user_id = "Jwaad"
game = Guess_The_Mood_Game(user_id)

levels = StandardLevels(game.window, game.window_center, game.pygame, game.music_filepath)
qt_manager = QTManager(levels = levels)
behave = Behaviours(game.pygame, game.music_filepath)

background = (100,100,100)
run = True

games_to_play = [1, 2, 3, 4 ] # [ 1, 2, 3, 4 ]
"""
try:
    levels.tap_to_continue(run,background)
    # Guess the mood
    if 1 in games_to_play:
        qt_manager.send_qt_command(gesture="wave", emote="grin")
        qt_manager.qt_say_blocking("Hi!,,, My name is Q-T!", black_screen = True)#" #I am going to be playing 3 games with you today, Each game will be music themed and will only last for 2 minutes.")
        qt_manager.send_qt_command(emote="talking")
        qt_manager.qt_say_blocking("Hope you are well!...", black_screen = True)
        qt_manager.send_qt_command(emote="talking")
        #qt_manager.qt_say_blocking("Would you like to play some games with me?...")
        run, consent = levels.yes_or_no_screen("Would you like to play some games with me?", run, background, silent=True)
        while not consent:
            # If they said no, say "please?"
            run, consent = levels.yes_or_no_screen("Can you please play with me?", run, background)
            levels.black_screen(run)
        run = levels.QTSpeakingScreen("", run, background)
        qt_manager.send_qt_command(gesture="happy", emote="grin")
        qt_manager.qt_say_blocking("Great!... Then, I am going to start the first game now!", black_screen = True)
        game.Main()
        for level in range(2,4):
            run = levels.QTSpeakingScreen("", run, background)
            qt_manager.qt_say_blocking(behave.get_next_level(), black_screen = True)
            game.play_level(run, "easy", level)
    # Fix the song
    if 2 in games_to_play:
        game = Fix_The_Song_Game(user_id, reduce_screen = False)
        qt_manager.send_qt_command(gesture="happy", emote="grin")
        run = levels.QTSpeakingScreen("", run, background)
        qt_manager.qt_say_blocking("Great job on that last game... Lets play the next one now!", black_screen = True)
        game.Main()
        for level in range(2, 4):
            run = levels.QTSpeakingScreen("", run, background)
            qt_manager.qt_say_blocking(behave.get_next_level(), black_screen = True)
            game.level_loader.tap_to_continue(run, background)
            game.play_music_blocking("easy", level)
            game.level_loader.countdown(3, run, background, prelim_msg="Get ready to hear the song!")
            game.play_level(difficulty= "easy", level_num=level)
    # Clap to the beat
    if 3 in games_to_play:
        game = Clap_To_Beat_Game(user_id, reduce_screen=False)
        qt_manager.send_qt_command(gesture="happy", emote="grin")
        run = levels.QTSpeakingScreen("", run, background)
        qt_manager.qt_say_blocking("I hope you enjoyed that game! Lets move on to the next one now.", black_screen = True)
        levels.yes_or_no_screen("But first. Will you help me get my drum?!", run, background)
        ready = False
        while not ready:
            #Move arms up
            qt_manager.move_right_arm(arm_up_right)
            qt_manager.move_left_arm(arm_up_right)  # not a mistake, this method flips the pos of the 1st joint
            run, ready = levels.yes_or_no_screen("Is it set up now?", run, background)
            levels.black_screen(run)
        game.Main()
    # Simon says
    if 4 in games_to_play:
        game = Simon_Says_Clap_Game(user_id, reduce_screen=False)
        qt_manager.send_qt_command(gesture="happy", emote="grin")
        run = levels.QTSpeakingScreen("", run, background)
        qt_manager.qt_say_blocking("Great!.. One last game, now!", black_screen = True)
        game.Main()
except():
    #game.pygame.quit
    game.sound_manager.stop_track()
    rospy.loginfo("Audio may not be stopped due to interrupt")
finally:
    run = levels.QTSpeakingScreen("", run, background)
    qt_manager.send_qt_command(gesture="wave", emote="grin")
    qt_manager.qt_say_blocking("That was the last one!", black_screen = True)
    qt_manager.send_qt_command(emote="talking")
    qt_manager.qt_say_blocking("Thank you for playing with me!", black_screen = True)
    qt_manager.send_qt_command(emote="talking")
    qt_manager.qt_say_blocking("I am grateful for you spending time with me!", black_screen = True)
    print("Shutting game down")
"""
motor_msg = [["right_arm", "left_arm"], [arm_up_right, arm_up_left]]

qt_manager.qt_actuate(motor_msg)
