#!/usr/bin/env python3
 
import rospy
import smach
import smach_ros
from mavros_msgs.srv import CommandBool, SetMode

# Importing all the state classes
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__)))
from take_off import TakeOff
from search import Search
from follow import Follow
from intercept import Intercept
#from terminate import Terminate

class VisualServoFSM:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("visual_servo_fsm")

        # Set up service proxies
        rospy.wait_for_service("/uav0/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("/uav0/mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/uav0/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("/uav0/mavros/set_mode", SetMode)

        # Initialize SMACH state machine
        self.sm = smach.StateMachine(outcomes=["TERMINATED", "SHUTDOWN"])
        
        # Adding state transitions
        with self.sm:
            smach.StateMachine.add(
                "TAKE_OFF", 
                TakeOff(self.arming_client, self.set_mode_client), 
                transitions={"HOVER_HEIGHT_REACHED": "SEARCH"}
            )

            smach.StateMachine.add(
                "SEARCH", 
                Search(), 
                transitions={"TARGET_ACQUIRED": "FOLLOW", "CONTINUE_SEARCH": "SEARCH"}
            )

            smach.StateMachine.add(
                "FOLLOW", 
                Follow(), 
                transitions={"READY_TO_INTERCEPT": "INTERCEPT", "TARGET_LOST": "SEARCH"}
            )

            smach.StateMachine.add(
                "INTERCEPT", 
                Intercept(), 
                transitions={"TARGET_HIT": "SEARCH", "TARGET_LOST": "SEARCH"}
            )

    def execute(self):
        # Execute the state machine
        outcome = self.sm.execute()
        rospy.loginfo(f"State machine finished with outcome: {outcome}")

if __name__ == "__main__":
    try:
        fsm = VisualServoFSM()
        fsm.execute()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutdown requested.")


        '''



            smach.StateMachine.add(
                "TERMINATE", 
                Terminate(), 
                transitions={"TERMINATED": "TERMINATED"}
            )
'''