#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import numpy as np
from std_msgs.msg import Float32
from dynamic_reconfigure.server import Server                 # Import Dynamic Reconfigure Server
from offboard_py.cfg import TargetConfigConfig


# Global variables to store current state and pose
current_state = State()
current_pose = PoseStamped()


def state_cb(msg):
    global current_state
    current_state = msg


def pose_cb(msg):
    global current_pose
    current_pose = msg
    # rospy.loginfo("Current Position: x: {}, y: {}, z: {}".format(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z))


class TargetController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("target_node")
        
        # Retrieve parameters from the parameter server
        self.trajectory_type = rospy.get_param("target/motion/trajectory_type", "circle")
        self.radius = rospy.get_param("target/motion/radius", 40)
        self.vertical_amplitude = rospy.get_param("target/motion/vertical_amplitude", 2)
        self.v = rospy.get_param("target/motion/v", 4)

        # Initialize Dynamic Reconfigure Server
        self.srv = Server(TargetConfigConfig, self.dynamic_reconfigure_callback)

        self.set_initial_reconfigure_values()

        rospy.loginfo("Dynamic Reconfigure Server for TargetController Initialized.")

        # Validate parameters
        self.validate_parameters()

        # Initialize variables based on trajectory type
        self.initialize_trajectory()

        # Subscribers
        namespace = "/uav1/"
        rospy.loginfo(f"Namespace: {namespace}")
        self.state_sub = rospy.Subscriber(f"{namespace}mavros/state", State, callback=state_cb)
        self.local_pos_sub = rospy.Subscriber(f"{namespace}mavros/local_position/pose", PoseStamped, callback=pose_cb)

        # Publishers
        self.local_pos_pub = rospy.Publisher(f"{namespace}mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.speed_pub = rospy.Publisher("/target_speed", Float32, queue_size=10)

        # Service Proxies
        rospy.wait_for_service(f"{namespace}mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy(f"{namespace}mavros/cmd/arming", CommandBool)
        
        rospy.wait_for_service(f"{namespace}mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy(f"{namespace}mavros/set_mode", SetMode)

        # Rate
        self.rate = rospy.Rate(20)  # 20 Hz

        # Initialize pose
        self.pose = PoseStamped()
        self.pose.pose.position.x = 8
        self.pose.pose.position.y = 5
        self.pose.pose.position.z = 2

    def validate_parameters(self):
        required_params = [
            "target/motion/trajectory_type",
            "target/motion/radius",
            "target/motion/vertical_amplitude",
            "target/motion/v"
        ]

        missing_params = [param for param in required_params if not rospy.has_param(param)]
        if missing_params:
            rospy.logerr(f"Missing required parameters: {missing_params}")
            rospy.signal_shutdown("Required parameters missing.")
        else:
            rospy.loginfo("All required parameters for target_node loaded successfully.")
            rospy.loginfo(f"Trajectory Type: {self.trajectory_type}")
            rospy.loginfo(f"Radius: {self.radius}")
            rospy.loginfo(f"Vertical Amplitude: {self.vertical_amplitude}")
            rospy.loginfo(f"Initial Speed (v): {self.v} m/s")

    def initialize_trajectory(self):
        if self.trajectory_type.lower() == "circle":
            self.circle = True
            self.A = None
            self.B = None
            self.C = None
        else:
            self.circle = False  # Extend to other trajectory types as needed
            # For simplicity, using radius for A and B, and vertical_amplitude for C
            self.A = self.radius
            self.B = self.radius
            self.C = self.vertical_amplitude

    def set_initial_reconfigure_values(self):
        # Pull the parameters from the parameter server (populated by the YAML file)
        initial_config = {
            'trajectory_type': rospy.get_param("target/motion/trajectory_type", "circle"),
            'radius': rospy.get_param("target/motion/radius", 40.0),
            'vertical_amplitude': rospy.get_param("target/motion/vertical_amplitude", 1.0),
            'v': rospy.get_param("target/motion/v", 8.0)
        }

        # Update the dynamic reconfigure server with these values from the parameter server
        self.srv.update_configuration(initial_config)



    def dynamic_reconfigure_callback(self, config, level):
        """
        Callback function for dynamic reconfigure.
        Updates trajectory parameters based on user input.
        """
        rospy.loginfo(f"Reconfigure Request: {config}")

        # Update trajectory parameters
        self.trajectory_type = config.trajectory_type
        self.radius = config.radius
        self.vertical_amplitude = config.vertical_amplitude
        self.v = config.v

        # Reinitialize trajectory based on new parameters
        self.initialize_trajectory()
        rospy.loginfo(f"Updated Trajectory Parameters: type={self.trajectory_type}, radius={self.radius}, vertical_amplitude={self.vertical_amplitude}, v={self.v} m/s")

        return config

    def run(self):
        # Send a few setpoints before starting
        for i in range(100):
            if rospy.is_shutdown():
                break
            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()

        # Set OFFBOARD mode
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        # Arm the drone
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()

        # Initialize trajectory variables
        t = 0
        cycle_count = 0

        rospy.sleep(5)  # Wait for connections

        while not rospy.is_shutdown():
            # Switch to OFFBOARD mode
            if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(1.0):
                if self.set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD mode enabled")
                last_req = rospy.Time.now()
            else:
                if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(1.0):
                    if self.arming_client.call(arm_cmd).success:
                        rospy.loginfo("Vehicle armed")
                    last_req = rospy.Time.now()

            # Determine position based on trajectory type
            if self.circle:
                x = self.radius * np.sin(t)
                y = self.radius * np.cos(t)
                z = 8 + self.vertical_amplitude * np.sin(t)
                # Update angular velocity and increment angle
                omega = self.v / self.radius
                t += omega * 0.05  # Increment t by angular velocity times the time step
            else:
                # Example: Figure Eight trajectory
                x = self.A * np.sin(t)
                y = self.B * np.sin(2 * t)
                z = 8 + self.C * np.sin(t)

                vx = self.A * np.cos(t)
                vy = 2 * self.B * np.cos(2 * t)
                vz = self.C * np.cos(t)

                speed = np.sqrt(vx**2 + vy**2 + vz**2)
                dt = 0.05 * (self.v / speed)

                t += dt

            # Check if a full cycle is completed
            if t >= 2 * np.pi:
                t = 0
                cycle_count += 1
                rospy.loginfo(f"Cycle {cycle_count} completed at speed {self.v} m/s")
                # If you want to change speed after cycles, modify 'v' here
                # Example: self.v += 0
                rospy.loginfo(f"Maintaining speed at {self.v} m/s")

            # Update pose
            self.pose.pose.position.x = x
            self.pose.pose.position.y = y
            self.pose.pose.position.z = z
            # rospy.loginfo(f"z = {z} m")
            self.local_pos_pub.publish(self.pose)

            # Publish the speed
            self.speed_pub.publish(self.v)
            # rospy.loginfo(f"Speed published: {self.v} m/s")

            self.rate.sleep()


if __name__ == "__main__":
    try:
        controller = TargetController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutdown requested.")
