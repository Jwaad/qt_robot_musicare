#! /usr/bin/env python
import rospy
import actionlib
import tty
import sys
import termios
import jwaad_test.msg
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

#Yaw is X movement
#Pitch is Y movement

class LockOnToFace(object):
    # create messages that are used to publish feedback/result
    feedback = jwaad_test.msg.FaceLockOnFeedback()
    result = jwaad_test.msg.FaceLockOnResult()
 
    def __init__(self, name):
        self._action_name = name
        self.head_position_pub = rospy.Publisher('/qt_robot/head_position/command', Float64MultiArray, queue_size=10)                                    #Init publisher
        self._as = actionlib.SimpleActionServer(self._action_name, jwaad_test.msg.FaceLockOnAction, execute_cb=self.ExecuteCallback, auto_start = False) #Init action server 
        self._as.start()            #Start action server
        self.GetJointStates()       #I shouldn't need to do this, but moving head wont work without it for some reason.
        self.MoveHeadToDefault()    #Move head to default pos (0,0)
    
    #Get the current pose of the QT head
    def GetJointStates(self):  #Get and return head joint states
        data = rospy.wait_for_message("/qt_robot/joints/state", JointState)
        return data

    #Move head to watch person head on
    def MoveToFace(self, x_pixel_offset, y_pixel_offset):

        #Create message
        head_ref = Float64MultiArray()
        
        #Convert pixel offsets to angles - 10 pixels is 1 degree
        yaw_offset = (x_pixel_offset) /10
        pitch_offset = -(y_pixel_offset) /10 #FLIP POLARITY OF Y
        
        #get current joint pose, and add in offsets
        joint_pose = self.GetJointStates()
        yaw = joint_pose.position[1] + yaw_offset
        pitch = joint_pose.position[0] + pitch_offset

        #Publish and move head        
        head_ref.data = [yaw, pitch]
        #print(head_ref)
        self.head_position_pub.publish(head_ref)
    
    #Move head to 0, 0
    def MoveHeadToDefault(self):
        head_ref = Float64MultiArray()
        head_ref.data = [0, 0]
        self.head_position_pub.publish(head_ref)
    
    def WaitForQTHeadToFinishMove(self):
        movement_ongoing = True
        while movement_ongoing:
            move_status = rospy.wait_for_message("/qt_head_status", Bool)
            qt_head_free = move_status.data #This should eventually return false
            if qt_head_free:
                movement_ongoing = False

            #TODO, ADD A TIMEOUT HERE
    
    ###MAIN CODE###

    #Action call             print("im_running")back
    def ExecuteCallback(self, goal):
        self.success = True
        #print("Action Name: ", self._action_name)
        #print("Goal : Y =", goal.y_offset, "X =", goal.x_offset)
        
        rospy.loginfo('Now centering face')
        rospy.loginfo('\033[33mWARNING: This will not work with multiple faces in view, currently that will cause strange behaviour.\033[39m') #Print yellow warning message
        
        #Default goals
        remaining_x = 0
        remaining_y = 0
        face_centered = False
        
        while not face_centered: 
            #check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                print("Action server was pre-empted ending execution")
                self.success = False
                break
                
            #Get offsets from topic, but take an average. Only move once the target is mostly stationary.
            target_stationary = False
            while not target_stationary:
                x_offsets = []
                y_offsets = []
                my_range = 5
                sample_start = rospy.get_time() #Time when sampling started
                for i in range(my_range):
                    offsets = rospy.wait_for_message("/face_track_offsets",Int16MultiArray)
                    x, y = offsets.data
                    x_offsets.append(x)
                    y_offsets.append(y)
                sampling_time = rospy.get_time() - sample_start #Time taken to take all samples
                x_distance = x_offsets[my_range-1] - x_offsets[0]
                x_speed = x_distance / sampling_time            #Pixel's moved per second in X
                y_distance = y_offsets[my_range-1] - y_offsets[0]
                y_speed = y_distance / sampling_time            #Pixel's moved per second in Y
                stationary_cutoff = 15                          #The pixel/s speed that we consider to be slow enough to consider stationary
                if (-stationary_cutoff <= x_speed <= stationary_cutoff) and (-stationary_cutoff <= y_speed <= stationary_cutoff):
                    target_stationary = True
                    #print("TARGET IS STATIONARY")
                else:
                    #print("TARGET IS TRYING TO ESCAPE")
                    pass
                
            #Average X and Y from the samplings
            remaining_x = sum(x_offsets)/my_range
            remaining_y = sum(y_offsets)/my_range
            
            #set goals as range of sent goal
            x_max = goal.x_offset + 30
            x_min = -x_max
            y_max = goal.y_offset + 30
            y_min = -y_max
                     
            #Send feedback
            self.feedback.remaining_x_offset = int(remaining_x) #Inted
            self.feedback.remaining_y_offset = int(remaining_y)
            self._as.publish_feedback(self.feedback)
            
            #If goal reached then end, else continue
            if ( x_min <= remaining_x <= x_max ) and ( y_min <= remaining_y <= y_max ):
                face_centered = True
            else:
                #Give method X and Y in pixel distance
                self.MoveToFace(remaining_x, remaining_y ) #Move qt's head to viewers face
                self.WaitForQTHeadToFinishMove() #Stay here til the QT head is free
        if self.success:
            self.result.completed = True
            rospy.loginfo('Face has been centered')
            self._as.set_succeeded(self.result)

if __name__ == '__main__':
    rospy.init_node('face_lock_on_action_server')
    server = LockOnToFace(rospy.get_name())
    rospy.spin()

