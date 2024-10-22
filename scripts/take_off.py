import rospy
import smach
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBoolRequest, SetModeRequest
from geometry_msgs.msg import PoseStamped
import numpy as np

class TakeOff(smach.State):
    def __init__(self, arming_client, set_mode_client, hover_height=8.0):
        smach.State.__init__(self, outcomes=["HOVER_HEIGHT_REACHED"])
        self.arming_client = arming_client
        self.set_mode_client = set_mode_client
        self.hover_height = hover_height

        # Current pose of UAV1
        self.current_pose_uav1 = PoseStamped()

        # Subscribers
        self.local_pos_sub_uav1 = rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, self.pose_cb_uav1)

        # Publishers
        self.local_pos_pub = rospy.Publisher("/uav0/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        self.rate = rospy.Rate(10)  # 10 Hz

    def pose_cb_uav1(self, msg):
        self.current_pose_uav1 = msg

    def execute(self, userdata):
        # Set OFFBOARD mode
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        # Arm the drone
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()

        while not rospy.is_shutdown():
            # Switch to OFFBOARD mode if not already in it
            if self.current_pose_uav1.header.stamp == rospy.Time(0):
                rospy.loginfo("Waiting for UAV pose data...")
                self.rate.sleep()
                continue

            if self.current_pose_uav1.pose.position.z >= self.hover_height - 0.2:
                rospy.loginfo("Hover height reached. Proceeding to SEARCH state.")
                return "HOVER_HEIGHT_REACHED"

            if rospy.Time.now() - last_req > rospy.Duration(1.0):
                if self.set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD mode enabled")
                if self.arming_client.call(arm_cmd).success:
                    rospy.loginfo("Vehicle armed")
                last_req = rospy.Time.now()

            # Set position target to hover height at current x and y
            waypoint = PoseStamped()
            waypoint.pose.position.x = self.current_pose_uav1.pose.position.x
            waypoint.pose.position.y = self.current_pose_uav1.pose.position.y
            waypoint.pose.position.z = self.hover_height

            self.local_pos_pub.publish(waypoint)

            self.rate.sleep()
