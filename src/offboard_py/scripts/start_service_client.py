#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty

def start_nodes():
    rospy.wait_for_service('start_pursuit')
    try:
        start_pursuit = rospy.ServiceProxy('start_pursuit', Empty)
        start_pursuit()
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node('start_service_client')
    start_nodes()
