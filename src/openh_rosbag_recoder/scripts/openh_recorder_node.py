#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool
import subprocess
import signal
import os
from datetime import datetime

# Import the dynamic reconfigure server
from dynamic_reconfigure.server import Server
from openh_rosbag_recoder.cfg import RecorderParamsConfig

class RosbagRecorderNode:
    def __init__(self):
        """Node initialization."""
        rospy.loginfo("Initializing OpenH Rosbag Recorder Node...")

        # Internal state variables
        self.recording_process = None
        self.is_recording = False
        self.recording_switch = False  # Master switch controlled by dynamic reconfigure
        self.bag_save_path = None
        self.topics_str = ""

        # Set up the dynamic reconfigure server
        self.dyn_server = Server(RecorderParamsConfig, self.dynamic_reconfigure_callback)

        # Load static parameters from the parameter server
        self.load_ros_params()
        
        # Subscribe to the state topic that triggers recording
        rospy.Subscriber("/rosbag_replay_state", Bool, self.state_callback, queue_size=1)
        
        # Set up a cleanup function to be called on node shutdown
        rospy.on_shutdown(self.cleanup)

        rospy.loginfo("Recorder node is ready and waiting for triggers.")

    def load_ros_params(self):
        """Load static parameters from the ROS Parameter Server."""
        try:
            self.bag_save_path = rospy.get_param('~bag_save_path', '/tmp/rosbags')
            topics_list = rospy.get_param('~topics_to_record', [])
            
            if not topics_list:
                rospy.logwarn("No topics to record specified in the config file.")
            self.topics_str = " ".join(topics_list)

            rospy.loginfo("Bag save path: %s", self.bag_save_path)
            rospy.loginfo("Topics to record: %s", self.topics_str)
        except Exception as e:
            rospy.logerr("Failed to load parameters: %s", e)

    def dynamic_reconfigure_callback(self, config, level):
        """Dynamic reconfigure callback function."""
        self.recording_switch = config.recording_switch
        rospy.loginfo("Dynamic reconfigure: recording_switch set to ==> %s", self.recording_switch)
        
        # If the switch is turned off while a recording is in progress, stop it.
        if not self.recording_switch and self.is_recording:
            rospy.logwarn("Recording switch turned off during recording. Stopping now.")
            self.stop_recording()
            
        return config

    def state_callback(self, msg):
        """Callback for the /rosbag_replay_state topic subscriber."""
        is_start_signal = msg.data

        # Check if the master recording switch is enabled
        if not self.recording_switch:
            rospy.logdebug("Recording switch is OFF. Ignoring state signal.")
            return

        rospy.loginfo("Received state signal: %s", is_start_signal)

        # Decide whether to start or stop recording based on the signal and current state
        if is_start_signal and not self.is_recording:
            self.start_recording()
        elif not is_start_signal and self.is_recording:
            self.stop_recording()

    def start_recording(self):
        """Starts the rosbag recording subprocess."""
        if not os.path.exists(self.bag_save_path):
            try:
                os.makedirs(self.bag_save_path)
                rospy.loginfo("Created directory: %s", self.bag_save_path)
            except OSError as e:
                rospy.logerr("Failed to create directory %s: %s", self.bag_save_path, e)
                return

        # Generate a filename with a timestamp
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        bag_filename = os.path.join(self.bag_save_path, f"recording_{timestamp}.bag")
        
        command = f"rosbag record -O {bag_filename} {self.topics_str}"
        
        rospy.loginfo("Starting rosbag recording...")
        rospy.loginfo(f"COMMAND: {command}")

        # Use Popen to start the subprocess so the main process is not blocked
        self.recording_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
        self.is_recording = True

    def stop_recording(self):
        """Stops the rosbag recording subprocess."""
        if self.recording_process and self.is_recording:
            rospy.loginfo("Stopping rosbag recording...")
            # Send a SIGINT signal (equivalent to Ctrl+C) to allow rosbag record to shut down cleanly
            os.killpg(os.getpgid(self.recording_process.pid), signal.SIGINT)
            self.recording_process.wait()  # Wait for the subprocess to terminate completely
            self.recording_process = None
            self.is_recording = False
            rospy.loginfo("Recording stopped.")
        else:
            rospy.logwarn("Stop called, but no active recording process found.")

    def cleanup(self):
        """Cleanup operations to be performed on node shutdown."""
        rospy.loginfo("Shutting down recorder node.")
        self.stop_recording()

if __name__ == '__main__':
    rospy.init_node('openh_recorder_node', anonymous=False)
    try:
        node = RosbagRecorderNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass