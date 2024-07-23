#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import numpy as np
import signal

class ErrorLogger:
    def __init__(self):
        self.time_log = []
        self.error_log = []
        self.distance_error_log = []
        self.start_time = rospy.Time.now()

        self.error_sub = rospy.Subscriber("/uav0/horizontal_error", Float32, self.error_callback)
        self.distance_error_sub = rospy.Subscriber("/uav0/distance_error", Float32, self.distance_error_callback)
        
        # Register signal handler for shutdown
        signal.signal(signal.SIGINT, self.shutdown_handler)

    def error_callback(self, msg):
        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.start_time).to_sec()

        self.time_log.append(elapsed_time)
        self.error_log.append(msg.data)
        if len(self.distance_error_log) < len(self.error_log):
            self.distance_error_log.append(float('nan'))

    def distance_error_callback(self, msg):
        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.start_time).to_sec()

        if len(self.time_log) == len(self.distance_error_log):
            self.time_log.append(elapsed_time)
        self.distance_error_log.append(msg.data)
        if len(self.error_log) < len(self.distance_error_log):
            self.error_log.append(float('nan'))

    def interpolate_nan(self, x, y):
        """
        Interpolates NaN values in the given data.
        """
        nans, x = np.isnan(y), lambda z: z.nonzero()[0]
        y[nans] = np.interp(x(nans), x(~nans), y[~nans])
        return y

    def shutdown_handler(self, signum, frame):
        rospy.loginfo("Shutdown signal received, plotting graph...")

        time_log = np.array(self.time_log)
        error_log = np.array(self.error_log)
        distance_error_log = np.array(self.distance_error_log)

        error_log = self.interpolate_nan(time_log, error_log)
        distance_error_log = self.interpolate_nan(time_log, distance_error_log)

        fig, ax1 = plt.subplots()

        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Horizontal Error', color='tab:blue')
        ax1.plot(time_log, error_log, label='Horizontal Error', color='tab:blue')
        ax1.tick_params(axis='y', labelcolor='tab:blue')

        ax2 = ax1.twinx()
        ax2.set_ylabel('Distance Error', color='tab:red')
        ax2.plot(time_log, distance_error_log, label='Distance Error', color='tab:red')
        ax2.tick_params(axis='y', labelcolor='tab:red')

        fig.tight_layout()
        plt.title('Horizontal and Distance Errors vs Time')
        plt.grid(True)
        plt.show()

        rospy.signal_shutdown("Shutdown signal received")

if __name__ == "__main__":
    rospy.init_node("error_logger_node")
    logger = ErrorLogger()
    rospy.spin()
