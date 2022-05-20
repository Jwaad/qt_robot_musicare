#! /usr/bin/env python

from __future__ import print_function

import rospy
import actionlib # Brings in the SimpleActionClient
import sys
from std_msgs.msg import Int16MultiArray

# Brings in the messages used by the "face lock on" action, including the goal message and the result message.
import jwaad_test.msg

#Data gotten from camera feed
image_cols = 640
image_rows = 480

def Main():
    while not rospy.is_shutdown():
        offsets = rospy.wait_for_message("/face_track_offsets",Int16MultiArray)
        x_offset, y_offset = offsets.data
        x, y = OffsetsToXandY(x_offset, y_offset) #Converts offsets to X and Y locations in picture
        if ((image_cols)/4 >= x) or (x >= (image_cols*3)/4) or ((image_rows)/4 >= y) or (y >= (image_rows*3)/4):  #If the persons face nears 1/3 of the edges
            result = face_lock_on_client()
            print(result)

def OffsetsToXandY(x_offset,y_offset):
    #Convert X
    if x_offset >= 0 :
        x = x_offset + (image_cols/2) #Halfway mark + offset from center
    else:
        x = (image_cols/2) + x_offset #Halfway mark - offset from center
        
    #Convert Y
    if y_offset >= 0 :
        y = y_offset + (image_rows/2) #Halfway mark + offset from center
    else:
        y = (image_rows/2) + y_offset #Halfway mark - offset from center
    
    return (x,y)

def face_lock_on_client():
    # Creates the SimpleActionClient, passing the type of the action to the constructor
    client = actionlib.SimpleActionClient('face_lock_on_action_server', jwaad_test.msg.FaceLockOnAction)

    # Waits until the action server has started up and started
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = jwaad_test.msg.FaceLockOnGoal(y_offset=0, x_offset=0)

    # Sends the goal to the action server.
    client.send_goal(goal)
    rospy.loginfo("Request sent to center face")

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        rospy.init_node('face_lock_on_action_client')
        Main()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
