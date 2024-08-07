#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import numpy as np
from std_msgs.msg import Int32, Float32

current_state = State()
current_pose = PoseStamped()

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global current_pose
    current_pose = msg
    #rospy.loginfo("Current Position: x: {}, y: {}, z: {}".format(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z))

if __name__ == "__main__":
    rospy.init_node("target_node")

    namespace = "/uav1/"
    rospy.loginfo(f"Namespace: {namespace}")

    state_sub = rospy.Subscriber(f"{namespace}mavros/state", State, callback=state_cb)
    local_pos_sub = rospy.Subscriber(f"{namespace}mavros/local_position/pose", PoseStamped, callback=pose_cb)

    local_pos_pub = rospy.Publisher(f"{namespace}mavros/setpoint_position/local", PoseStamped, queue_size=10)
    speed_pub = rospy.Publisher("/target_speed", Float32, queue_size=10)

    rospy.wait_for_service(f"{namespace}mavros/cmd/arming")
    arming_client = rospy.ServiceProxy(f"{namespace}mavros/cmd/arming", CommandBool)

    rospy.wait_for_service(f"{namespace}mavros/set_mode")
    set_mode_client = rospy.ServiceProxy(f"{namespace}mavros/set_mode", SetMode)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    pose = PoseStamped()
    pose.pose.position.x = 8
    pose.pose.position.y = 5
    pose.pose.position.z = 2

    for i in range(100):
        if rospy.is_shutdown():
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    A = 10
    B = 5
    C = 1
    z_constant = 2
    t = 0
    v = 1  # Initial speed in m/s
    cycle_count = 0

    rospy.sleep(5)

    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD enabled")
            last_req = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if arming_client.call(arm_cmd).success:
                    rospy.loginfo("Vehicle armed")
                last_req = rospy.Time.now()

        x = A * np.sin(t)
        y = B * np.sin(2 * t)
        z = 2 + C * np.sin(t)

        vx = A * np.cos(t)
        vy = 2 * B * np.cos(2 * t)

        speed = np.sqrt(vx**2 + vy**2)
        dt = 0.05 * (v / speed)

        t += dt

        # Check if a full cycle (8 shape) is completed
        if t >= 2 * np.pi:
            t = 0
            cycle_count += 1
            rospy.loginfo(f"Cycle {cycle_count} completed at speed {v} m/s")
            v += 0.5
            rospy.loginfo(f"Increasing speed to {v} m/s")
                
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z_constant

        local_pos_pub.publish(pose)

        # Publish the speed
        speed_pub.publish(v)
        #rospy.loginfo(f"Speed published: {v} m/s")

        rate.sleep()
