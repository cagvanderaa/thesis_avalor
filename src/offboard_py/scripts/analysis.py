#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int32
import matplotlib.pyplot as plt
import numpy as np
import signal
import os
import math

class ErrorLogger:
    def __init__(self):
        self.time_log = []
        self.error_log = []
        self.distance_error_log = []
        self.current_speed = 0
        self.start_time = rospy.Time.now()
        self.tracking_started = False
        self.speed_logs = {}
        self.save_folder = "/home/cle"

        self.error_sub = rospy.Subscriber("/uav0/horizontal_error", Float32, self.error_callback)
        self.distance_error_sub = rospy.Subscriber("/uav0/distance_error", Float32, self.distance_error_callback)

        # Subscribe to the target speed signal
        self.speed_sub = rospy.Subscriber("/target_speed", Float32, self.speed_callback)

        # Register signal handler for shutdown
        signal.signal(signal.SIGINT, self.shutdown_handler)

    def speed_callback(self, msg):
        new_speed = msg.data
        if new_speed != self.current_speed:
            if self.tracking_started:
                self.plot_and_reset_logs(self.current_speed)
            self.current_speed = new_speed
            self.start_time = rospy.Time.now()
            self.tracking_started = True
            #rospy.loginfo(f"Tracking started at speed {new_speed} m/s")

    def error_callback(self, msg):
        if not self.tracking_started:
            return

        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.start_time).to_sec()

        self.time_log.append(elapsed_time)
        self.error_log.append(msg.data)
        if len(self.distance_error_log) < len(self.error_log):
            self.distance_error_log.append(float('nan'))

        #rospy.loginfo(f"Logged horizontal error at time {elapsed_time}")

    def distance_error_callback(self, msg):
        if not self.tracking_started:
            return

        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.start_time).to_sec()

        if len(self.time_log) == len(self.distance_error_log):
            self.time_log.append(elapsed_time)
        self.distance_error_log.append(msg.data)
        if len(self.error_log) < len(self.distance_error_log):
            self.error_log.append(float('nan'))

        #rospy.loginfo(f"Logged distance error at time {elapsed_time}")

    def interpolate_nan(self, x, y):
        """
        Interpolates NaN values in the given data.
        """
        if len(x) == 0 or len(y) == 0:
            return y  # If the arrays are empty, return as is
        nans, x = np.isnan(y), lambda z: z.nonzero()[0]
        if not np.any(~nans):
            return y  # If there are no non-NaN values, return as is
        y[nans] = np.interp(x(nans), x(~nans), y[~nans])
        return y

    def plot_and_reset_logs(self, speed):
        rospy.loginfo(f"Plotting and resetting logs for speed {speed}")

        if len(self.time_log) == 0 or len(self.error_log) == 0 or len(self.distance_error_log) == 0:
            rospy.loginfo("No data to plot for this speed. Skipping...")
            return

        time_log = np.array(self.time_log)
        error_log = np.array(self.error_log)
        distance_error_log = np.array(self.distance_error_log)

        rospy.loginfo(f"Time log: {time_log}")
        rospy.loginfo(f"Error log: {error_log}")
        rospy.loginfo(f"Distance error log: {distance_error_log}")

        error_log = self.interpolate_nan(time_log, error_log)
        distance_error_log = self.interpolate_nan(time_log, distance_error_log)

        if speed not in self.speed_logs:
            self.speed_logs[speed] = {'time': [], 'error': [], 'distance': []}

        self.speed_logs[speed]['time'].extend(time_log)
        self.speed_logs[speed]['error'].extend(error_log)
        self.speed_logs[speed]['distance'].extend(distance_error_log)

        # Reset logs for the next speed
        self.time_log = []
        self.error_log = []
        self.distance_error_log = []

        self.plot_all_logs(save=True)

    def plot_all_logs(self, save=False):
        speeds = sorted(self.speed_logs.keys())
        n_plots = len(speeds)
        
        fig, axes = plt.subplots(n_plots, 1, figsize=(15, 7 * n_plots))
        if n_plots == 1:
            axes = [axes]  # Ensure axes is iterable

        for ax, speed in zip(axes, speeds):
            time_log = self.speed_logs[speed]['time']
            error_log = self.speed_logs[speed]['error']
            distance_error_log = self.speed_logs[speed]['distance']

            # Calculate MSE and variance for the horizontal error
            rmse = math.sqrt(np.mean(np.square(error_log)))
            std = np.std(error_log)

            rospy.loginfo(f"RMSE for speed {speed}: {rmse}")
            rospy.loginfo(f"Standard Deviation for speed {speed}: {std}")

            ax1 = ax
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('Horizontal Error', color='tab:blue')
            ax1.plot(time_log, error_log, label=f'Horizontal Error (MSE: {rmse:.2f}, Var: {std:.2f})', color='tab:blue')
            ax1.tick_params(axis='y', labelcolor='tab:blue')

            ax2 = ax1.twinx()
            ax2.set_ylabel('Distance Error', color='tab:red')
            ax2.plot(time_log, distance_error_log, label='Distance Error', color='tab:red')
            ax2.tick_params(axis='y', labelcolor='tab:red')

            ax1.set_title(f'Horizontal and Distance Errors vs Time at Speed {speed} m/s')

        fig.tight_layout()
        if save:
            plt.savefig(os.path.join(self.save_folder, 'stability_plot.png'))
        plt.show()

    def shutdown_handler(self, signum, frame):
        rospy.loginfo("Shutdown signal received, plotting final graphs...")

        if len(self.time_log) > 0 or len(self.error_log) > 0 or len(self.distance_error_log) > 0:
            self.plot_and_reset_logs(self.current_speed)
        
        self.plot_all_logs(save=True)
        rospy.signal_shutdown("Shutdown signal received")

if __name__ == "__main__":
    rospy.init_node("error_logger_node")
    logger = ErrorLogger()
    rospy.spin()
