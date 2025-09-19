#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory, MoveGroupActionResult

SUCCESS = 1  # moveit_msgs/MoveItErrorCodes.SUCCESS
GOAL_OK = 3  # actionlib_msgs/GoalStatus.SUCCEEDED

class MoveItFinalGoalBridge:
    def __init__(self):
        # Parameters
        self.trigger = rospy.get_param("~trigger", "end")  # "start" | "end"
        self.out_topic = rospy.get_param("~output_topic", "/franky/target_joint_position")
        # If you want to force joint order, set a list like ["panda_joint1",..., "panda_joint7"]
        self.force_names = rospy.get_param("~joint_names", [])

        # Publisher (latched so late subscribers can still get the last goal)
        self.pub = rospy.Publisher(self.out_topic, JointState, queue_size=1, latch=True)

        # Caches
        self.cached_names = None
        self.cached_final = None
        self.last_goal_id_sent = None
        self.last_goal_id_ready = None  # goal_id of last successful plan

        # Subscribers
        rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory,
                         self.on_display_traj, queue_size=1)
        rospy.Subscriber("/move_group/result", MoveGroupActionResult,
                         self.on_result, queue_size=5)
        rospy.Subscriber("/move_group/trajectory_execution_event", String,
                         self.on_exec_event, queue_size=10)

        rospy.loginfo("[bridge] ready. trigger=%s, out=%s", self.trigger, self.out_topic)

    # ---- helpers ----
    def _extract_final_from_display(self, msg: DisplayTrajectory):
        if not msg.trajectory:
            return None
        jt = msg.trajectory[-1].joint_trajectory
        if not jt.points:
            return None
        return jt.joint_names, jt.points[-1].positions

    def _extract_final_from_result(self, res):
        try:
            if hasattr(res, "planned_trajectory"):
                jt = res.planned_trajectory.joint_trajectory
                if jt.points:
                    return jt.joint_names, jt.points[-1].positions
        except Exception:
            pass
        return None

    def _reorder_if_needed(self, names, pos):
        names = list(names)
        pos = list(pos)
        if not self.force_names:
            return names, pos
        idx = {n: i for i, n in enumerate(names)}
        ordered, missing = [], []
        for n in self.force_names:
            if n in idx:
                ordered.append(pos[idx[n]])
            else:
                missing.append(n)
        if missing:
            rospy.logwarn("[bridge] force_names missing joints: %s", ",".join(missing))
            return names, pos
        return list(self.force_names), ordered

    def _publish_once(self, goal_id):
        if self.cached_names is None or self.cached_final is None:
            rospy.logwarn("[bridge] no cached final joints, skip publish.")
            return
        if self.last_goal_id_sent == goal_id:
            return
        names, pos = self._reorder_if_needed(self.cached_names, self.cached_final)

        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = names
        js.position = list(pos)
        self.pub.publish(js)

        self.last_goal_id_sent = goal_id
        rospy.loginfo("[bridge] published FINAL joint target for goal_id=%s (%d joints) → %s",
                      goal_id, len(js.position), self.out_topic)

    # ---- callbacks ----
    def on_display_traj(self, msg: DisplayTrajectory):
        out = self._extract_final_from_display(msg)
        if out:
            self.cached_names, self.cached_final = out

    def on_result(self, msg: MoveGroupActionResult):
        goal_id = msg.status.goal_id.id if msg and msg.status else None
        ok = (
            msg and
            (msg.result is not None) and
            (msg.result.error_code.val == SUCCESS) and
            (msg.status is not None) and
            (msg.status.status == GOAL_OK)
        )
        if not ok:
            return

        out = self._extract_final_from_result(msg.result)
        if out:
            self.cached_names, self.cached_final = out
        if not self.cached_names or not self.cached_final:
            rospy.logwarn("[bridge] plan OK but result has no trajectory; waiting display_planned_path cache.")

        self.last_goal_id_ready = goal_id
        rospy.loginfo("[bridge] plan SUCCESS, cached final. goal_id=%s", goal_id)

        if self.trigger == "end":
            self._publish_once(goal_id)

    def on_exec_event(self, msg: String):
        if self.trigger != "start":
            return
        if msg.data != "start":
            return
        goal_id = self.last_goal_id_ready
        if not goal_id:
            rospy.logwarn("[bridge] EXEC start but no ready goal_id.")
            return
        self._publish_once(goal_id)

if __name__ == "__main__":
    rospy.init_node("moveit_result_to_franky_goal", anonymous=False)
    MoveItFinalGoalBridge()
    rospy.spin()

