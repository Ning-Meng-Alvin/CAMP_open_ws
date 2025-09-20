#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rosbag_recoder: a small Python wrapper around `rosbag record`.

Features:
- All configs come from private params (~), typically loaded via YAML in launch.
- Topic selection via `~topics` (list/string) or YAML; regex/exclude supported.
- SpaceMouse long press (button0 by default, >=2.0s) toggles START/STOP.
- Services: ~start, ~stop, ~reload.
- Supports splitting by size/duration, compression lz4/bz2, optional auto-stop.

Naming:
- Output base name is <bag_prefix>_<YYYY-MM-DD>_<HH-MM-SS>
  e.g. ~/.ros/bags/rec_2025-09-19_11-23-45.bag
  With splitting: ..._000000.bag, ..._000001.bag, ...
"""

import os
import signal
import subprocess
import time
from datetime import datetime

import rospy
import yaml
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger, TriggerResponse

GREEN = "\033[92m"
RED="\033[91m"
RESET  = "\033[0m"


def _to_list(val):
    """Normalize a parameter to list. Accepts YAML-like string, comma/space separated, or list."""
    if val is None:
        return []
    if isinstance(val, list):
        return val
    if isinstance(val, str):
        s = val.strip()
        if s.startswith('[') and s.endswith(']'):
            try:
                v = yaml.safe_load(s)
                return v or []
            except Exception:
                pass
        parts = [p for p in [x.strip() for x in s.replace(',', ' ').split()] if p]
        return parts
    return [val]


class RecorderNode:
    def __init__(self):
        rospy.init_node("rosbag_recoder")

        # ---- General recording params (loaded via rosparam/YAML in launch) ----
        self.out_dir        = os.path.expanduser(rospy.get_param("~out_dir", "~/.ros/bags"))
        self.bag_prefix     = rospy.get_param("~bag_prefix", "rec")
        self.topics_param   = rospy.get_param("~topics", [])               # list or string
        self.topics_yaml    = rospy.get_param("~topics_yaml", "")          # optional YAML {topics:[...], exclude:[...]}
        self.exclude_param  = rospy.get_param("~exclude", [])
        self.use_regex      = bool(rospy.get_param("~use_regex", False))   # -e treat topics as regex
        self.compression    = rospy.get_param("~compression", "lz4")       # lz4|bz2|none
        self.split_size_mb  = int(rospy.get_param("~split_size_mb", 0))    # >0 enable size-based split
        self.split_duration = int(rospy.get_param("~split_duration_s", 0)) # >0 enable time-based split
        self.max_splits     = int(rospy.get_param("~max_splits", 0))       # 0 = unlimited
        self.duration_stop  = int(rospy.get_param("~stop_after_s", 0))     # >0 auto stop after seconds

        # NOTE: Passed directly to rosbag CLI; unit follows rosbag's CLI (bytes in Noetic).
        self.buffsize       = int(rospy.get_param("~buffsize", 0))         # 0 -> not set

        self.all_if_empty   = bool(rospy.get_param("~record_all_if_empty", True))

        # ---- SpaceMouse / Joy trigger ----
        self.joy_topic          = rospy.get_param("~joy_topic", "/spacenav/joy")
        self.start_button_index = int(rospy.get_param("~start_button_index", 0))  # button0
        self.long_press_sec     = float(rospy.get_param("~long_press_sec", 2.0))
        self.toggle_mode        = bool(rospy.get_param("~toggle_mode", True))
        self.auto_start         = bool(rospy.get_param("~auto_start", False))

        # Internal states
        self.proc = None
        self.recording = False
        self.rec_start_walltime = 0.0
        self.btn_down = False
        self.btn_down_since = None
        self.press_fired = False  # <- Ensures one trigger per hold

        # Load topics once (can be reloaded)
        self._reload_topic_lists()

        # ROS I/O
        self.joy_sub = rospy.Subscriber(self.joy_topic, Joy, self._on_joy, queue_size=10)
        self.srv_start  = rospy.Service("~start",  Trigger, self._srv_start)
        self.srv_stop   = rospy.Service("~stop",   Trigger, self._srv_stop)
        self.srv_reload = rospy.Service("~reload", Trigger, self._srv_reload)

        if self.auto_start:
            self.start_recording()

    # ---------- Parameter helpers ----------

    def _reload_topic_lists(self):
        """Reload topic/exclude lists from params or YAML file."""
        topics_param = self.topics_param
        exclude_param = self.exclude_param
        if self.topics_yaml and os.path.exists(self.topics_yaml):
            try:
                with open(self.topics_yaml, 'r') as f:
                    data = yaml.safe_load(f) or {}
                if "topics" in data:
                    topics_param = data["topics"]
                if "exclude" in data:
                    exclude_param = data["exclude"]
            except Exception as e:
                rospy.logwarn("Failed to read YAML '%s': %s", self.topics_yaml, e)
        self.topics = _to_list(topics_param)
        self.exclude = _to_list(exclude_param)

    def _build_rosbag_cmd(self):
        """Build the rosbag record command line based on current params."""
        os.makedirs(self.out_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        bag_base = f"{self.bag_prefix}_{ts}"
        out_path = os.path.join(self.out_dir, bag_base)

        cmd = ["rosbag", "record", "-O", out_path]

        # Compression
        comp = str(self.compression).lower()
        if comp == "lz4":
            cmd += ["--lz4"]
        elif comp == "bz2":
            cmd += ["--bz2"]

        # Splitting
        if self.split_size_mb > 0 or self.split_duration > 0:
            cmd += ["--split"]
            if self.split_size_mb > 0:
                cmd += [f"--size={self.split_size_mb}"]
            if self.split_duration > 0:
                cmd += [f"--duration={self.split_duration}"]
            if self.max_splits > 0:
                cmd += [f"--max-splits={self.max_splits}"]

        # Buffer size
        if self.buffsize > 0:
            cmd += [f"--buffsize={self.buffsize}"]

        # Regex / exclude
        if self.use_regex:
            cmd += ["-e"]
        for ex in self.exclude:
            cmd += ["-x", ex]

        # Topic selection
        if self.topics:
            cmd += self.topics
        elif self.all_if_empty:
            cmd += ["-a"]
        else:
            raise RuntimeError("No topics given and record_all_if_empty=false")

        return cmd

    # ---------- Joy handling (long-press: one-shot per hold) ----------

    def _on_joy(self, msg: Joy):
        pressed = (len(msg.buttons) > self.start_button_index and
                   msg.buttons[self.start_button_index] == 1)
        now = rospy.Time.now().to_sec()

        if pressed:
            # First press -> start timer, mark as not fired
            if not self.btn_down:
                self.btn_down = True
                self.btn_down_since = now
                self.press_fired = False

            # While holding: if not fired and threshold reached -> trigger once
            if (not self.press_fired) and self.btn_down_since is not None \
               and (now - self.btn_down_since) >= self.long_press_sec:
                if not self.recording:
                    rospy.loginfo(f"{GREEN}*** Long press detected -> START recording. ***{RESET}")
                    self.start_recording()
                else:
                    if self.toggle_mode:
                        rospy.loginfo(f"{RED}*** Long press detected -> STOP recording.***{RESET}")
                        self.stop_recording()
                # Mark fired: no repeat until released
                self.press_fired = True

        else:
            # Released -> reset, allow next press to fire
            if self.btn_down:
                self.btn_down = False
                self.btn_down_since = None
                self.press_fired = False

    # ---------- Start / Stop logic ----------

    def start_recording(self):
        if self.recording:
            rospy.logwarn("Already recording.")
            return
        try:
            cmd = self._build_rosbag_cmd()
        except Exception as e:
            rospy.logerr("Cannot start rosbag: %s", e)
            return

        rospy.loginfo("Starting rosbag: %s", " ".join(cmd))
        try:
            self.proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
            self.recording = True
            self.rec_start_walltime = time.time()
        except Exception as e:
            rospy.logerr("Failed to start rosbag: %s", e)
            self.proc = None
            self.recording = False

    def stop_recording(self):
        if not self.recording or self.proc is None:
            rospy.logwarn("Not recording.")
            return
        try:
            os.killpg(self.proc.pid, signal.SIGINT)
            self.proc.wait(timeout=10)
        except Exception:
            try:
                os.killpg(self.proc.pid, signal.SIGTERM)
            except Exception:
                pass
        finally:
            self.proc = None
            self.recording = False
            rospy.loginfo("Recording stopped. Bags at: %s", self.out_dir)

    # ---------- Services ----------

    def _srv_start(self, _req):
        self.start_recording()
        return TriggerResponse(success=self.recording, message="start requested")

    def _srv_stop(self, _req):
        was = self.recording
        self.stop_recording()
        return TriggerResponse(success=was and not self.recording, message="stop requested")

    def _srv_reload(self, _req):
        was = self.recording
        if was:
            self.stop_recording()
        self._reload_topic_lists()
        if was:
            self.start_recording()
        return TriggerResponse(success=True, message="reloaded topics (and restarted if previously running)")

    # ---------- Spin loop ----------

    def spin(self):
        rate = rospy.Rate(20)
        try:
            while not rospy.is_shutdown():
                # Auto-stop after duration
                if self.recording and self.duration_stop > 0:
                    if (time.time() - self.rec_start_walltime) >= self.duration_stop:
                        rospy.loginfo("Stop-after reached -> stopping recording.")
                        self.stop_recording()

                # Detect unexpected rosbag exit
                if self.recording and self.proc is not None and (self.proc.poll() is not None):
                    rospy.logwarn("rosbag process exited unexpectedly (code %s).", self.proc.returncode)
                    self.proc = None
                    self.recording = False

                rate.sleep()
        finally:
            if self.recording:
                self.stop_recording()


def main():
    node = RecorderNode()
    node.spin()


if __name__ == "__main__":
    main()
