#!/home/camp/miniconda3/envs/franka_conda/bin/python
# -*- coding: utf-8 -*-

"""
franka_unified_controller.py

Unified controller with three mutually-exclusive modes:
  1) Cartesian velocity servo from /spacenav/twist
  2) One-shot JointMotion from /franky/target_joint_position
  3) Joint-state follower from /joint_state_follower (replay) with
     an internal ring buffer (default 2000 frames). If the buffer
     becomes empty and no new JSF message arrives for ~jsf_timeout
     seconds, it falls back to servo mode.

Key safety:
- All motion-type switches are serialized inside ONE control thread.
- Before switching to JointMotion, we send a single zero-velocity
  CartesianVelocityMotion and join(), to avoid "motion type change" errors.
- Publisher thread only READS robot state; control thread is the ONLY one
  sending robot.move()/join_motion().

Author: you :)
"""

import numpy as np
from dataclasses import dataclass
from typing import Union
import threading
from collections import deque

import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from geometry_msgs.msg import Twist as TwistMsg

# ===== Franky SDK =====
from franky import (
    Robot,
    Affine, ReferenceType, Twist, RelativeDynamicsFactor,
    CartesianImpedanceMotion, JointMotion, CartesianVelocityMotion
)

# -------------------- Data structures --------------------
@dataclass
class FrankaStates:
    joint_positions: Union[list, np.ndarray]
    joint_velocities: Union[list, np.ndarray]
    joint_efforts:   Union[list, np.ndarray]
    ee_translation:  np.ndarray
    ee_quaternion:   np.ndarray


# -------------------- ROS interface --------------------
class FrankaROS:
    def __init__(self):
        self.ros_init()
        self.franka_states = None
        self.FRANKA_JOINT_NAMES = [
            "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
            "panda_joint5", "panda_joint6", "panda_joint7"
        ]

    def ros_init(self):
        # params (configurable by launch)
        self.robot_ip = rospy.get_param('~robot_ip', '172.16.0.2')
        self.relative_dynamics_factor = rospy.get_param('~relative_dynamics_factor', 0.2)
        self.ctrl_rate = rospy.get_param('~control_rate', 100.0)
        self.pub_rate  = rospy.get_param('~publish_rate', 100.0)
        self.jstates_topic = rospy.get_param('~jstates_topic', '/joint_states')
        self.eepose_topic  = rospy.get_param('~eepose_topic' , '/end_effector_pose')

        # publishers
        self.jmsg = JointState()
        self.jmsg.header.seq = 0
        self.jmsg.header.frame_id = 'panda_link0'
        self.pub_jstates = rospy.Publisher(self.jstates_topic, JointState, queue_size=2)

        self.emsg = PoseStamped()
        self.emsg.header.seq = 0
        self.emsg.header.frame_id = 'panda_link0'
        self.pub_eepose  = rospy.Publisher(self.eepose_topic, PoseStamped, queue_size=2)

        self.broadcaster = tf.TransformBroadcaster()

    def thread_pub_franka_states(self, robot: Robot):
        """Publish arm joint states, EE pose, and TF at high rate (read-only)."""
        self.jmsg.name = self.FRANKA_JOINT_NAMES
        rate = rospy.Rate(self.pub_rate)
        while not rospy.is_shutdown():
            # READ robot state once per cycle (single snapshot)
            currpose = robot.current_pose.end_effector_pose
            self.franka_states = FrankaStates(
                robot.current_joint_positions,
                robot.current_joint_velocities,
                robot.state.tau_J,
                currpose.translation,
                currpose.quaternion
            )

            # joint states
            self.jmsg.header.seq += 1
            self.jmsg.position = self.franka_states.joint_positions
            self.jmsg.velocity = self.franka_states.joint_velocities
            self.jmsg.effort   = self.franka_states.joint_efforts
            self.jmsg.header.stamp = rospy.Time.now()
            self.pub_jstates.publish(self.jmsg)

            # ee pose
            self.emsg.header.seq += 1
            self.emsg.pose.position    = Point(*self.franka_states.ee_translation)
            self.emsg.pose.orientation = Quaternion(*self.franka_states.ee_quaternion)
            self.emsg.header.stamp = rospy.Time.now()
            self.pub_eepose.publish(self.emsg)

            # tf
            self.broadcaster.sendTransform(
                translation=self.franka_states.ee_translation,
                rotation=self.franka_states.ee_quaternion,
                time=rospy.Time.now(),
                parent="panda_link0",
                child="panda_EE",
            )
            rate.sleep()


# -------------------- Manager (control) --------------------
class FrankaManager():
    def __init__(self):
        # ROS
        rospy.init_node('franka_unified_controller', anonymous=False)
        self.ros = FrankaROS()

        # Franky
        self.robot = Robot(self.ros.robot_ip)
        # transl/rot/elbow factors (can be tuned)
        self.robot.relative_dynamics_factor = RelativeDynamicsFactor(0.2, 0.3, 0.3)
        self.control_rate = self.ros.ctrl_rate

        # ===== one-shot joint goal =====
        self._goal_lock = threading.Lock()
        self._pending_joint_goal = None  # list[float] or None

        # ===== velocity stream gate & rate-limit =====
        self._vel_enabled = True
        self._last_twist = np.zeros(6, dtype=float)
        self._twist_eps = 1e-4

        # ===== JSF (joint_state_follower) mode with buffer =====
        self._jsf_enabled = False
        self._jsf_request_start = False
        self._jsf_last_time = rospy.Time(0)
        self._jsf_timeout = rospy.get_param('~jsf_timeout', 0.2)
        self._jsf_lock = threading.Lock()
        self._jsf_buffer = deque(maxlen=int(rospy.get_param('~jsf_buffer_len', 2000)))

        # ===== inputs =====
        self.teleop_cmd = np.zeros(6, dtype=float)
        rospy.Subscriber("/spacenav/twist", TwistMsg, self.spacenav_callback, queue_size=20)
        # ensure first value (optional)
        try:
            rospy.wait_for_message("/spacenav/twist", TwistMsg, timeout=5.0)
        except rospy.ROSException:
            rospy.logwarn("No /spacenav/twist yet; starting anyway.")

        rospy.Subscriber("/franky/target_joint_position", JointState, self.on_joint_goal, queue_size=1)
        # use a larger subscriber queue to reduce upstream drops while control thread is joining motions
        rospy.Subscriber("/joint_state_follower", JointState, self.on_jsf,
                         queue_size=self._jsf_buffer.maxlen)

        rospy.loginfo("Unified controller ready. JSF buffer len = %d", self._jsf_buffer.maxlen)

    # -------- inputs --------
    def spacenav_callback(self, msg: TwistMsg):
        self.teleop_cmd = np.array([
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z
        ], dtype=float)

    def on_joint_goal(self, msg: JointState):
        """Receive a one-shot joint goal, reorder if names provided, then cache it."""
        try:
            if msg.name:
                name2pos = dict(zip(msg.name, msg.position))
                ordered = [name2pos[n] for n in self.ros.FRANKA_JOINT_NAMES]
            else:
                ordered = list(msg.position)
            with self._goal_lock:
                self._pending_joint_goal = ordered
            rospy.loginfo("[unified] received one-shot joint goal (%d)", len(ordered))
        except Exception as e:
            rospy.logerr("on_joint_goal failed: %s", e)

    def on_jsf(self, msg: JointState):
        """
        Joint-state follower feed (buffered):
        - Reorder to Franka joint order if names are given.
        - Append to the ring buffer (deque).
        - Record last receive time, and trigger JSF mode on the first hit.
        """
        try:
            if msg.name:
                m = dict(zip(msg.name, msg.position))
                ordered = [m[n] for n in self.ros.FRANKA_JOINT_NAMES]
            else:
                ordered = list(msg.position)
            with self._jsf_lock:
                self._jsf_buffer.append(ordered)           # enqueue
                self._jsf_last_time = rospy.Time.now()
                if not self._jsf_enabled:
                    self._jsf_request_start = True
        except Exception as e:
            rospy.logerr("[unified] on_jsf failed: %s", e)

    # -------- helpers --------
    def _stop_velocity_stream(self):
        """
        Cleanly close velocity stream by sending zero CartesianVelocityMotion and join.
        Prevents 'motion type change' error when switching to JointMotion.
        """
        try:
            zero = np.zeros(6, dtype=float)
            m_cv = CartesianVelocityMotion(
                Twist(zero[:3], zero[3:]),
                relative_dynamics_factor=0.3
            )
            self.robot.move(m_cv, asynchronous=True)
            rospy.sleep(0.05)  # small settle
            self.robot.join_motion()
        except Exception as e:
            rospy.logwarn("stop velocity stream warn: %s", e)

    def velocity_control(self, twist: np.ndarray, rdf=1.0):
        """Send CartesianVelocityMotion only when enabled and twist significantly changes."""
        if not self._vel_enabled:
            return
        if np.linalg.norm(twist - self._last_twist) < self._twist_eps:
            return
        self._last_twist = twist.copy()
        m_cv = CartesianVelocityMotion(
            Twist(twist[:3], twist[3:]),
            relative_dynamics_factor=rdf
        )
        self.robot.move(m_cv, asynchronous=True)

    # -------- threads --------
    def start_threads(self):
        thread_list = [
            threading.Thread(
                target=self.ros.thread_pub_franka_states,
                kwargs={"robot": self.robot}, daemon=True
            ),
            threading.Thread(
                target=self.thread_control,
                kwargs={"robot": self.robot, "ros_interface": self.ros}, daemon=True
            ),
        ]
        for th in thread_list: th.start()
        for th in thread_list: th.join()

    def thread_control(self, robot: Robot, ros_interface: FrankaROS):
        control_rate = rospy.Rate(self.control_rate)

        # Wait for the first state from publisher thread
        while ros_interface.franka_states is None and not rospy.is_shutdown():
            rospy.sleep(0.01)

        # Optional: move to a rest-like pose once at startup
        robot.move(JointMotion([0.0, 0.0, 0.0, -2.0, 0.0, 2.0, 0.0]))
        robot.join_motion()

        while not rospy.is_shutdown():
            # ===== Enter JSF mode if requested =====
            if self._jsf_request_start:
                rospy.loginfo("[unified] ENTER JSF: stop velocity & switch")
                self._vel_enabled = False
                self._stop_velocity_stream()
                self._jsf_enabled = True
                self._jsf_request_start = False
                self._last_twist = np.zeros(6, dtype=float)

            # ===== JSF mode loop (buffered) =====
            if self._jsf_enabled:
                with self._jsf_lock:
                    target = self._jsf_buffer.popleft() if self._jsf_buffer else None
                    last_t = self._jsf_last_time
                    buffer_empty = (len(self._jsf_buffer) == 0)

                # # Execute blocking JointMotion if we have a target
                # if target is not None:
                #     try:
                #         robot.move(JointMotion(target))
                #         robot.join_motion()
                #     except Exception as e:
                #         rospy.logerr("[unified] JSF move failed: %s", e)

                # Execute async JointMotion if we have a target
                if target is not None:
                    try:
                        robot.move(JointMotion(target), asynchronous=True)
                    except Exception as e:
                        rospy.logerr("[unified] JSF async move failed: %s", e)

                # Only when buffer is empty AND timed out, go back to servo
                if buffer_empty and (last_t.to_sec() > 0) and \
                   ((rospy.Time.now() - last_t).to_sec() > self._jsf_timeout):
                    rospy.loginfo("[unified] JSF buffer empty & timeout -> back to SERVO")
                    self._jsf_enabled = False
                    self._vel_enabled = True
                    self._last_twist = np.zeros(6, dtype=float)
                    control_rate.sleep()
                    continue

                control_rate.sleep()
                continue  # stay in JSF

            # ===== One-shot JointMotion has priority over servo =====
            goal = None
            with self._goal_lock:
                if self._pending_joint_goal is not None:
                    goal = self._pending_joint_goal
                    self._pending_joint_goal = None  # consume once

            if goal is not None:
                try:
                    rospy.loginfo("[unified] one-shot JointMotion...")
                    self._vel_enabled = False
                    self._stop_velocity_stream()
                    robot.move(JointMotion(goal))
                    robot.join_motion()
                except Exception as e:
                    rospy.logerr("[unified] one-shot failed: %s", e)
                finally:
                    self._last_twist = np.zeros(6, dtype=float)
                    self._vel_enabled = True
                control_rate.sleep()
                continue

            # ===== Regular velocity servo =====
            scale = np.array([0.1, 0.1, 0.1, 0.6, 0.6, 0.6], dtype=float)
            self.velocity_control(scale * self.teleop_cmd, rdf=0.5)

            control_rate.sleep()

        rospy.loginfo('<Unified Controller> exit ...')
        robot.join_motion()  # ensure clean stop


if __name__ == '__main__':
    fman = FrankaManager()
    fman.start_threads()
