#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
import matplotlib.pyplot as plt
import numpy as np
import signal
from mpl_toolkits.mplot3d import Axes3D  # For 3D plotting
from tabulate import tabulate             # For table formatting
import scipy.signal                      # For peak detection
import os                                 # For directory operations
from scipy.spatial.transform import Rotation as R  # For quaternion to Euler conversion

class DataLogger:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("analysis_node")
        rospy.loginfo("Analysis Node Initialized.")

        # Load all parameters from the parameter server
        self.load_parameters()

        # Initialize data storage
        self.initialize_data_storage()

        # Initialize extra information
        self.start_time = rospy.Time.now()
        self.end_time = None
        self.trajectory_cycles = 0
        self.target_lost = False

        # Define the save folder from parameters, default to "/home/cle/experiment_results"
        self.save_folder = self.general_params.get("save_folder", "/home/cle/experiment_results")

        # Ensure the save folder exists
        if not os.path.exists(self.save_folder):
            try:
                os.makedirs(self.save_folder)
                rospy.loginfo(f"Created save folder at: {self.save_folder}")
            except Exception as e:
                rospy.logerr(f"Failed to create save folder at {self.save_folder}: {e}")
                rospy.signal_shutdown("Unable to create save folder.")

        # Subscribe to necessary topics
        self.subscribe_topics()

        # Initialize pose data
        self.uav0_pose = None
        self.uav1_pose = None

        # Initialize pitch angle and vertical position difference
        self.pitch_angles = []
        self.vertical_position_diff = []

        # Initialize time logging using a timer
        self.time_log = []
        self.time_timer = rospy.Timer(rospy.Duration(0.1), self.time_log_callback)  # 10 Hz

        # Register signal handler for shutdown
        signal.signal(signal.SIGINT, self.shutdown_handler)

    def load_parameters(self):
        # Retrieve pursuer parameters
        self.pursuer_params = rospy.get_param("pursuer", {})
        # Retrieve target parameters
        self.target_params = rospy.get_param("target", {})
        # Retrieve general parameters
        self.general_params = rospy.get_param("general", {})

        rospy.loginfo("Loaded Experiment Parameters:")
        self.print_parameters()

    def print_parameters(self):
        table = []

        # Pursuer Parameters
        if 'vision' in self.pursuer_params:
            for key, value in self.pursuer_params['vision'].items():
                table.append([f"Pursuer Vision: {key}", value])

        if 'control' in self.pursuer_params:
            for key, subdict in self.pursuer_params['control'].items():
                if isinstance(subdict, dict):
                    for subkey, subvalue in subdict.items():
                        # Wrap long parameter names by inserting newlines
                        param_name = f"Pursuer Control:\n{key} - {subkey}"
                        table.append([param_name, self.value_to_string(subvalue)])
                else:
                    param_name = f"Pursuer Control:\n{key}"
                    table.append([param_name, self.value_to_string(subdict)])

        # Target Parameters
        if 'motion' in self.target_params:
            for key, value in self.target_params['motion'].items():
                table.append([f"Target Motion: {key}", value])

        # General Parameters
        for key, value in self.general_params.items():
            if key != "save_folder":  # Exclude save_folder from the table
                table.append([f"General: {key}", value])

        # Store the table for later use
        self.parameter_table = table  # Store for later use

        # **Print the table**
        if table:
            print(tabulate(table, headers=["Parameter", "Value"], tablefmt="grid"))
        else:
            rospy.logwarn("No parameters found on the parameter server under 'pursuer', 'target', or 'general'.")

    def initialize_data_storage(self):
        # For pursuer's vertical data
        self.vertical_error_projector = []
        self.pitch_angles = []
        self.vertical_position_diff = []

        # For horizontal and distance error logging
        self.error_log = []
        self.distance_error_log = []

        # For velocity tracking
        self.velocities = []
        self.max_velocity = 0  # To store the max velocity for color scaling

        # For yaw rate, delta h, delta forward
        self.yaw_rate_log = []
        self.delta_h_log = []
        self.delta_forward_log = []

        # For 3D trajectory storage
        self.uav0_positions = []
        self.uav1_positions = []

    def subscribe_topics(self):
        # Subscriptions for pursuer data
        rospy.Subscriber("/uav0/vertical_error_projector", Float32, self.vertical_error_projector_cb)

        # Removed subscriptions:
        # rospy.Subscriber("/uav0/pitch_angle", Float32, self.pitch_angle_cb)
        # rospy.Subscriber("/uav0/vertical_position_diff", Float32, self.vertical_position_diff_cb)
        # rospy.Subscriber("/uav0/time_log", Float32, self.time_log_cb)

        # Subscriptions for horizontal and distance error logging
        rospy.Subscriber("/uav0/horizontal_error", Float32, self.error_callback)
        rospy.Subscriber("/uav0/distance_error", Float32, self.distance_error_callback)

        # Subscriptions for yaw rate, delta h, delta forward
        rospy.Subscriber("/uav0/yaw_rate", Float32, self.yaw_rate_callback)
        rospy.Subscriber("/uav0/delta_h", Float32, self.delta_h_callback)
        rospy.Subscriber("/uav0/delta_forward", Float32, self.delta_forward_callback)

        # Subscriptions for velocity
        rospy.Subscriber("/uav0/mavros/local_position/velocity_local", TwistStamped, self.velocity_callback)

        # Subscriptions for drone positions (3D trajectory plotting)
        self.namespace_uav0 = "/uav0/"
        self.namespace_uav1 = "/uav1/"
        rospy.Subscriber(f"{self.namespace_uav0}mavros/local_position/pose", PoseStamped, self.pose_cb_uav0)
        rospy.Subscriber(f"{self.namespace_uav1}mavros/local_position/pose", PoseStamped, self.pose_cb_uav1)

        # Optional: Subscribe to target node's cycle count and target lost status if available
        # For example, if target.py publishes 'cycle_count' and 'target_lost' on specific topics
        # rospy.Subscriber("/target/cycle_count", Int32, self.cycle_count_callback)
        # rospy.Subscriber("/target/target_lost", Bool, self.target_lost_callback)

    # Callbacks for pursuer data
    def vertical_error_projector_cb(self, msg):
        self.vertical_error_projector.append(msg.data)
        rospy.logdebug(f"Received Vertical Error Projector: {msg.data}")

    # Callbacks for horizontal and distance error logging
    def error_callback(self, msg):
        self.error_log.append(msg.data)
        rospy.logdebug(f"Received Horizontal Error: {msg.data}")

    def distance_error_callback(self, msg):
        self.distance_error_log.append(msg.data)
        rospy.logdebug(f"Received Distance Error: {msg.data}")

    # Callbacks for yaw rate, delta h, delta forward
    def yaw_rate_callback(self, msg):
        self.yaw_rate_log.append(msg.data)
        rospy.logdebug(f"Received Yaw Rate: {msg.data}")

    def delta_h_callback(self, msg):
        self.delta_h_log.append(msg.data)
        rospy.logdebug(f"Received Delta H: {msg.data}")

    def delta_forward_callback(self, msg):
        self.delta_forward_log.append(msg.data)
        rospy.logdebug(f"Received Delta Forward: {msg.data}")

    # Callback for velocity
    def velocity_callback(self, msg):
        linear_velocity = msg.twist.linear
        velocity_magnitude = np.sqrt(linear_velocity.x**2 + linear_velocity.y**2 + linear_velocity.z**2)
        self.velocities.append(velocity_magnitude)
        rospy.logdebug(f"Received Velocity: {velocity_magnitude} m/s")

        # Update the max velocity
        if velocity_magnitude > self.max_velocity:
            self.max_velocity = velocity_magnitude

    # Callback for uav0 pose
    def pose_cb_uav0(self, msg):
        self.uav0_pose = msg
        position = msg.pose.position
        orientation = msg.pose.orientation

        # Append uav0 position for 3D trajectory
        self.uav0_positions.append([position.x, position.y, position.z])

        # Compute pitch angle from orientation
        quaternion = [
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ]
        rotation = R.from_quat(quaternion)
        euler = rotation.as_euler('xyz', degrees=False)
        pitch = euler[1]  # Pitch angle
        self.pitch_angles.append(pitch)
        rospy.logdebug(f"Computed Pitch Angle: {pitch} radians")

        # If uav1_pose is available, compute vertical_position_diff
        if self.uav1_pose is not None:
            uav1_z = self.uav1_pose.pose.position.z
            vertical_diff = position.z - uav1_z
            self.vertical_position_diff.append(vertical_diff)
            rospy.logdebug(f"Computed Vertical Position Difference: {vertical_diff} m")

    # Callback for uav1 pose
    def pose_cb_uav1(self, msg):
        self.uav1_pose = msg
        position = msg.pose.position

        # Append uav1 position for 3D trajectory
        self.uav1_positions.append([position.x, position.y, position.z])

        # If uav0_pose is available, compute vertical_position_diff
        if self.uav0_pose is not None:
            uav0_z = self.uav0_pose.pose.position.z
            vertical_diff = uav0_z - position.z
            self.vertical_position_diff.append(vertical_diff)
            rospy.logdebug(f"Computed Vertical Position Difference: {vertical_diff} m")

    # Timer callback for time logging
    def time_log_callback(self, event):
        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.start_time).to_sec()
        self.time_log.append(elapsed_time)
        rospy.logdebug(f"Logged Time: {elapsed_time} s")

    # Plot the 3D trajectories of both UAVs
    def plot_3d_trajectories(self, ax):
        uav0_positions = np.array(self.uav0_positions)
        uav1_positions = np.array(self.uav1_positions)

        if uav0_positions.size == 0 or uav1_positions.size == 0:
            rospy.logwarn("No data to plot for UAV trajectories.")
            return

        # Plot UAV0 trajectory
        ax.plot(uav0_positions[:, 0], uav0_positions[:, 1], uav0_positions[:, 2], label="Pursuer (UAV0)", color="blue")

        # Plot UAV1 trajectory
        ax.plot(uav1_positions[:, 0], uav1_positions[:, 1], uav1_positions[:, 2], label="Target (UAV1)", color="red")

        # Set axis labels
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_zlabel('Z Position (m)')
        ax.set_title('3D Trajectories of Pursuer and Target UAVs')
        ax.legend()

    def plot_error_with_velocity(self, ax):
        if len(self.error_log) == 0 or len(self.vertical_error_projector) == 0 or len(self.velocities) == 0:
            rospy.logwarn("Not enough data to plot error with velocity.")
            return

        min_length = min(len(self.error_log), len(self.vertical_error_projector), len(self.velocities))
        errors = np.array(self.error_log[:min_length])
        vertical_errors = np.array(self.vertical_error_projector[:min_length])
        velocities = np.array(self.velocities[:min_length])

        scatter = ax.scatter(errors, vertical_errors, c=velocities, cmap='coolwarm', vmin=0, vmax=self.max_velocity)
        ax.figure.colorbar(scatter, ax=ax, label='Speed (m/s)')

        ax.set_xlim([-400, 400])
        ax.set_ylim([-400, 400])

        ax.set_xlabel('Horizontal Error (px)')
        ax.set_ylabel('Vertical Error (px)')
        ax.set_title('Horizontal and Vertical Errors Colored by Velocity')

    def plot_results(self):
        if len(self.time_log) == 0:
            rospy.logwarn("Time log is empty. Cannot plot.")
            return

        # Determine min_length based on specific pairs needed for each plot
        min_length_errors = min(len(self.time_log), len(self.error_log), len(self.vertical_error_projector))
        min_length_vertical = min(len(self.time_log), len(self.vertical_position_diff), len(self.distance_error_log))
        min_length_yaw = min(len(self.time_log), len(self.yaw_rate_log))
        min_length_delta_h = min(len(self.time_log), len(self.delta_h_log))
        min_length_delta_forward = min(len(self.time_log), len(self.delta_forward_log))
        min_length_velocity = min(len(self.time_log), len(self.velocities))

        # Slice data accordingly for each plot
        time_errors = self.time_log[:min_length_errors]
        error_log = self.error_log[:min_length_errors]
        vertical_error_projector = self.vertical_error_projector[:min_length_errors]

        time_vertical = self.time_log[:min_length_vertical]
        vertical_position_diff = self.vertical_position_diff[:min_length_vertical]
        distance_error_log = self.distance_error_log[:min_length_vertical]

        time_yaw = self.time_log[:min_length_yaw]
        yaw_rate_log = self.yaw_rate_log[:min_length_yaw]

        time_delta_h = self.time_log[:min_length_delta_h]
        delta_h_log = self.delta_h_log[:min_length_delta_h]

        time_delta_forward = self.time_log[:min_length_delta_forward]
        delta_forward_log = self.delta_forward_log[:min_length_delta_forward]

        time_velocity = self.time_log[:min_length_velocity]
        velocities = self.velocities[:min_length_velocity]

        # First figure: Horizontal and Vertical Errors over Time
        fig, axs = plt.subplots(2, figsize=(12, 12))
        axs[0].plot(time_errors, error_log, label='Horizontal Error', color='tab:blue')
        axs[0].plot(time_errors, vertical_error_projector, label='Vertical Error', color='tab:red')
        axs[0].set_xlabel('Time (s)')
        axs[0].set_ylabel('Error (px)')
        axs[0].set_ylim([-400, 400])
        axs[0].legend()
        axs[0].set_title('Horizontal and Vertical Errors over Time')

        # Vertical position difference and lateral distance with separate y-axes
        ax1 = axs[1]
        ax1.plot(time_vertical, vertical_position_diff, label='Vertical Position Difference', color='tab:green')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Vertical Position Difference (m)')

        ax2 = ax1.twinx()  # Create a twin y-axis
        ax2.plot(time_vertical, distance_error_log, label='Lateral Distance', color='tab:purple')
        ax2.set_ylabel('Lateral Distance (m)')

        ax1.legend(loc="upper left")
        ax2.legend(loc="upper right")
        ax1.set_title('Vertical Position Difference and Lateral Distance over Time')

        plt.tight_layout()
        # Save the plot
        errors_over_time_path = os.path.join(self.save_folder, "errors_over_time.png")
        plt.savefig(errors_over_time_path)
        rospy.loginfo(f"Saved plot: errors_over_time.png to {self.save_folder}")
        plt.show(block=False)

        # Second figure: 3D trajectory
        fig_3d = plt.figure(figsize=(8, 6))
        ax_3d = fig_3d.add_subplot(111, projection='3d')
        self.plot_3d_trajectories(ax_3d)
        plt.tight_layout()
        # Save the plot
        traj_3d_path = os.path.join(self.save_folder, "3d_trajectories.png")
        plt.savefig(traj_3d_path)
        rospy.loginfo(f"Saved plot: 3d_trajectories.png to {self.save_folder}")
        plt.show(block=False)

        # Third figure: error vs velocity plot
        fig_vel = plt.figure(figsize=(8, 6))
        ax_vel = fig_vel.add_subplot(111)
        if len(time_velocity) > 0 and len(velocities) > 0 and len(error_log[:min_length_velocity]) == len(vertical_error_projector[:min_length_velocity]):
            scatter = ax_vel.scatter(
                error_log[:min_length_velocity],
                vertical_error_projector[:min_length_velocity],
                c=velocities, cmap='coolwarm', vmin=0, vmax=self.max_velocity
            )
            ax_vel.figure.colorbar(scatter, ax=ax_vel, label='Speed (m/s)')

            ax_vel.set_xlim([-400, 400])
            ax_vel.set_ylim([-400, 400])

            ax_vel.set_xlabel('Horizontal Error (px)')
            ax_vel.set_ylabel('Vertical Error (px)')
            ax_vel.set_title('Horizontal and Vertical Errors Colored by Velocity')
        else:
            rospy.logwarn("Not enough data to plot error vs velocity.")

        plt.tight_layout()
        # Save the plot
        error_vel_path = os.path.join(self.save_folder, "error_vs_velocity.png")
        plt.savefig(error_vel_path)
        rospy.loginfo(f"Saved plot: error_vs_velocity.png to {self.save_folder}")
        plt.show(block=False)

        # Fourth figure: new subplots for yaw rate, delta h, delta forward
        fig4, axs4 = plt.subplots(3, figsize=(12, 18))

        # Horizontal error and yaw rate with separate scales
        ax1 = axs4[0]
        ax1.plot(time_errors, error_log, label='Horizontal Error', color='tab:blue')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Horizontal Error (px)')

        ax2 = ax1.twinx()
        ax2.plot(time_yaw, yaw_rate_log, label='Yaw Rate', color='tab:orange')
        ax2.set_ylabel('Yaw Rate (rad/s)')

        ax1.legend(loc="upper left")
        ax2.legend(loc="upper right")
        ax1.set_title('Horizontal Error and Yaw Rate over Time')

        # Vertical error and delta h with separate scales
        ax1 = axs4[1]
        ax1.plot(time_errors, vertical_error_projector, label='Vertical Error', color='tab:red')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Vertical Error (px)')

        ax2 = ax1.twinx()
        ax2.plot(time_delta_h, delta_h_log, label='Delta H', color='tab:green')
        ax2.set_ylabel('Delta H (m/s²)')

        ax1.legend(loc="upper left")
        ax2.legend(loc="upper right")
        ax1.set_title('Vertical Error and Delta H over Time')

        # Distance error and delta forward with separate scales
        ax1 = axs4[2]
        ax1.plot(time_vertical, distance_error_log, label='Distance Error', color='tab:purple')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Distance Error (m)')

        ax2 = ax1.twinx()
        ax2.plot(time_delta_forward, delta_forward_log, label='Delta Forward', color='tab:brown')
        ax2.set_ylabel('Delta Forward (m/s²)')

        ax1.legend(loc="upper left")
        ax2.legend(loc="upper right")
        ax1.set_title('Distance Error and Delta Forward over Time')

        plt.tight_layout()
        # Save the plot
        additional_errors_path = os.path.join(self.save_folder, "additional_errors_over_time.png")
        plt.savefig(additional_errors_path)
        rospy.loginfo(f"Saved plot: additional_errors_over_time.png to {self.save_folder}")
        plt.show(block=False)

        # Keep the figures alive and responsive until manually closed by the user
        while plt.get_fignums():
            plt.pause(0.1)
                
    def display_summary_table(self, simulation_time):
        # Create a new table with improved formatting to avoid repetition
        formatted_table = []

        # Pursuer Vision parameters
        if 'vision' in self.pursuer_params:
            formatted_table.append(["Pursuer Vision: virtual_plane", self.pursuer_params['vision'].get('virtual_plane', 'N/A')])
            formatted_table.append(["Pursuer Vision: IOU", self.pursuer_params['vision'].get('IOU', 'N/A')])
            formatted_table.append(["Pursuer Vision: accuracy", self.pursuer_params['vision'].get('accuracy', 'N/A')])
            formatted_table.append(["Pursuer Vision: vertical_offset", self.pursuer_params['vision'].get('vertical_offset', 'N/A')])

        # Pursuer Control parameters: Indented format for clarity
        if 'control' in self.pursuer_params:
            formatted_table.append(["Pursuer Control:", ""])
            formatted_table.append(["  follow_distance", self.pursuer_params['control'].get('follow_distance', 'N/A')])
            if 'pid_control_horizontal' in self.pursuer_params['control']:
                formatted_table.append(["  pid_control_horizontal:", ""])
                formatted_table.append(["    Kp", self.pursuer_params['control']['pid_control_horizontal'].get('Kp', 'N/A')])
                formatted_table.append(["    Ki", self.pursuer_params['control']['pid_control_horizontal'].get('Ki', 'N/A')])
                formatted_table.append(["    Kd", self.pursuer_params['control']['pid_control_horizontal'].get('Kd', 'N/A')])
            if 'pid_control_vertical' in self.pursuer_params['control']:
                formatted_table.append(["  pid_control_vertical:", ""])
                formatted_table.append(["    Kp", self.pursuer_params['control']['pid_control_vertical'].get('Kp', 'N/A')])
                formatted_table.append(["    Ki", self.pursuer_params['control']['pid_control_vertical'].get('Ki', 'N/A')])
                formatted_table.append(["    Kd", self.pursuer_params['control']['pid_control_vertical'].get('Kd', 'N/A')])
            if 'pid_control_distance' in self.pursuer_params['control']:
                formatted_table.append(["  pid_control_distance:", ""])
                formatted_table.append(["    Kp", self.pursuer_params['control']['pid_control_distance'].get('Kp', 'N/A')])
                formatted_table.append(["    Ki", self.pursuer_params['control']['pid_control_distance'].get('Ki', 'N/A')])
                formatted_table.append(["    Kd", self.pursuer_params['control']['pid_control_distance'].get('Kd', 'N/A')])

        # Target Motion parameters
        if 'motion' in self.target_params:
            formatted_table.append(["Target Motion: radius", self.target_params['motion'].get('radius', 'N/A')])
            formatted_table.append(["Target Motion: vertical_amplitude", self.target_params['motion'].get('vertical_amplitude', 'N/A')])
            formatted_table.append(["Target Motion: v", self.target_params['motion'].get('v', 'N/A')])
            formatted_table.append(["Target Motion: trajectory_type", self.target_params['motion'].get('trajectory_type', 'N/A')])

        # General parameters
        for key, value in self.general_params.items():
            if key != "save_folder":  # Exclude save_folder from the table
                formatted_table.append([f"General: {key}", value])

        # Additional information (Simulation details)
        formatted_table.append(["Simulation Time (s)", f"{simulation_time:.2f}"])
        formatted_table.append(["Trajectory Cycles", self.trajectory_cycles])
        formatted_table.append(["Target Lost", "Yes" if self.target_lost else "No"])

        # Create a figure for the table with increased width
        fig, ax = plt.subplots(figsize=(12, len(formatted_table) * 0.5))  # Adjust the height based on rows
        ax.axis('off')  # Hide axes

        # Define column widths: parameter names get more space
        col_widths = [0.7, 0.3]  # Give more space to parameter names and less to values

        # Create the table in the figure
        tbl = ax.table(cellText=formatted_table, colLabels=["Parameter", "Value"], colWidths=col_widths, loc='center', cellLoc='left')

        # Adjust font size and scaling for readability
        tbl.auto_set_font_size(False)
        tbl.set_fontsize(10)
        tbl.scale(1, 1.5)

        # Ensure the title is rendered above the table, not inside it
        plt.title("Experiment Parameters and Summary", fontsize=16, pad=20)  # Use `pad` to create space

        # Define the path to save the summary table as a PNG
        save_path = os.path.join(self.save_folder, "experiment_summary.png")

        # Save the table as a PNG file
        plt.savefig(save_path, bbox_inches='tight')
        plt.close(fig)

        rospy.loginfo(f"Experiment summary table saved to {save_path}")

    def run(self):
        rospy.spin()

    def shutdown_handler(self, signum, frame):
        rospy.loginfo("Shutdown signal received. Compiling final report...")
        self.end_time = rospy.Time.now()
        simulation_time = (self.end_time - self.start_time).to_sec()

        # Determine if target was lost
        # For simplicity, define target lost as any instance where distance_error > threshold
        distance_threshold = 5  # Define a suitable threshold based on your experiment
        if any(np.abs(np.array(self.distance_error_log)) > distance_threshold):
            self.target_lost = True
        else:
            self.target_lost = False

        # Count trajectory cycles
        # Assuming each peak in vertical_error_projector corresponds to a cycle
        # This is a simplistic approach and may need refinement based on actual data
        peaks, _ = scipy.signal.find_peaks(self.vertical_error_projector, height=0)
        self.trajectory_cycles = len(peaks)

        # Display the table as PNG
        self.display_summary_table(simulation_time)

        # Plot the results and save plots
        self.plot_results()

        rospy.signal_shutdown("Shutdown signal received.")

    def value_to_string(self, value):
        # Convert values to string and handle types
        if isinstance(value, float):
            return f"{value:.2f}"
        else:
            return str(value)

if __name__ == "__main__":
    try:
        logger = DataLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass
