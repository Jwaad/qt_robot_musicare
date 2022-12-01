#!/usr/bin/env python

#libraries
from guess_mood import Guess_The_Mood_Game
from fix_song import Fix_The_Song_Game
from clap_beat import Clap_To_Beat_Game
from simon_says_clap import Simon_Says_Clap_Game
from musicare_lib import QTManager
from musicare_lib import StandardLevels
from std_msgs.msg import Float64MultiArray
import rospy



#init node
rospy.init_node('Pilot', anonymous=False)
rospy.loginfo("Node launched successfully")
right_arm_pos_pub = rospy.Publisher('/qt_robot/right_arm_position/command', Float64MultiArray, queue_size=10)
left_arm_pos_pub = rospy.Publisher('/qt_robot/left_arm_position/command', Float64MultiArray, queue_size=10)

#create arm msgs to move qt arms
arm_up_right = [-5, -59.599998474121094, -40.70000076293945] #motor pos for right arm to be in air
arm_up_left = [10.699999809265137, -59.599998474121094, -40.70000076293945] #motor pos for right arm to be in air
arm_right_msg = Float64MultiArray()
arm_left_msg = Float64MultiArray()
arm_left_msg.data = arm_up_right
arm_left_msg.data = arm_up_left

#init games
fix_song = Fix_The_Song_Game()
clap_game = Clap_To_Beat_Game()
guess_game = Guess_The_Mood_Game()
simon_says = Simon_Says_Clap_Game()


background = (100,100,100)
run = True

#Run fix the song
try:
    QTManager.send_qt_command(gesture="wave", emote="grin")
    QTManager.qt_say_blocking("Hi!,,, My name is Q-T!")#" #I am going to be playing 3 games with you today, Each game will be music themed and will only last for 2 minutes.")
    QTManager.send_qt_command(emote="talking")
    QTManager.qt_say_blocking("Hope you are well!...")
    QTManager.send_qt_command(emote="talking")
    QTManager.qt_say_blocking("Would you like to play some games with me?...")
    consent = StandardLevels.yes_or_no_screen('Would you like to play ?', run, background)
    QTManager.send_qt_command(gesture="happy", emote="grin")
    QTManager.qt_say_blocking("Great!... Then, I am going to start the first game now!")
    guess_game.Main()
except():
    guess_game.pygame.quit
    guess_game.sound_manager.stop_track()
    rospy.loginfo("Audio may not be stopped due to interrupt")
finally:
    print("Shutting game down")
"""

if consent:
    #Run guess the mood
    try:
        guess_game.transition_screen_blocking("Please listen to QT", "Alright, you have finished the game called fix the song... Next, we will play ay Guessing game!")
        guess_game.transition_screen_blocking("Please listen to QT", "I am going to start the next game now!", should_gesture = False)
        guess_game.Main()
    except():
        guess_game.pygame.quit
        rospy.loginfo("Audio may not be stopped due to interrupt")
    finally:
        print("Shutting game down")   


    #Run clap to beat
    try:
        #clap_game.transition_screen_blocking("Please listen to QT", "Great! you have finished guess the song, now we will play a clapping game!")
        #clap_game.qt_say("But first we need to get my drum ready!")
        #right_arm_pos_pub.publish(arm_right_msg)
        #left_arm_pos_pub.publish(arm_left_msg)
        #rospy.sleep(1)
        arm_up = [20.699999809265137, -59.599998474121094, -40.70000076293945] #motor pos for right arm to be in air
        clap_game.move_right_arm(arm_up)
        clap_game.move_left_arm(arm_up)
        rospy.sleep(3)
        #Delay and do it again, to increase the likihood it's done
        clap_game.move_right_arm(arm_up)
        clap_game.move_left_arm(arm_up)

        clap_game.transition_screen("Wait for the researcher to setup the drum.")
        clap_game.transition_screen_blocking("Please listen to QT", "I am going to start the last game now!")
        clap_game.Main()
        
        clap_game.transition_screen_blocking("All done!", "")
        clap_game.transition_screen_blocking("Thank you for playing with me !", "Thank you so much for playing with me... We are done for today... Thanks for helping us with this experiment... Hope you have a good day!.. ")
        clap_game.qt_emote("grin")
        clap_game.qt_gesture("wave")
        clap_game.qt_say("Bye byee!")
    except(): 
        clap_game.shutdown()
        clap_game.pygame.quit
        rospy.loginfo("Audio may not be stopped due to interrupt")
    finally:
        print("Shutting game down")

clap_game.qt_gesture("arms_up") #To reset arms

"""
    

