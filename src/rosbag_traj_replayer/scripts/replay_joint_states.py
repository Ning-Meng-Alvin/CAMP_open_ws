#!/home/camp/miniconda3/envs/franka_conda/bin/python
# -*- coding: utf-8 -*-

"""
Replay JointState from a selected rosbag at recorded rate to /joint_state_follower.

UI:
- A simple curses TUI that lists *.bag under a directory (default: /home/camp/ros_bags).
- Refreshes file list every 1s.
- Arrow Up/Down to move cursor; ENTER to start replay; 'r' to refresh immediately; 'q' to quit.

Replay:
- Scans the bag and finds the first topic of type sensor_msgs/JointState
  (or use ~source_topics to force a list of topics).
- Publishes to ~out_topic (default: /joint_state_follower).
- Sleeps using the delta of bag recording time (t from rosbag.read_messages), i.e. same rate.
- Header stamps are replaced with rospy.Time.now() before publishing.

Notes:
- On Ctrl-C or 'q' during replay, we stop cleanly.
- If multiple split bags exist, you can replay them one by one; combining is trivial but omitted for minimalism.
"""

import os
import time
import curses
import signal
import rospy
import rosbag
from sensor_msgs.msg import JointState

DEFAULT_DIR = "/home/camp/ros_bags"

FRANKA_JOINT_ORDER = [
    "panda_joint1", "panda_joint2", "panda_joint3",
    "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
]

def list_bags(dirpath):
    try:
        files = [f for f in os.listdir(dirpath) if f.endswith(".bag")]
    except Exception:
        return []
    files.sort(key=lambda f: os.path.getmtime(os.path.join(dirpath, f)), reverse=True)
    return files

def find_joint_topics(bag, forced_topics=None):
    """Return a list of JointState topics to replay (first found if forced_topics is None)."""
    if forced_topics:
        return forced_topics
    # auto-detect first JointState topic
    try:
        info = bag.get_type_and_topic_info()
        for topic, tinfo in info.topics.items():
            if tinfo.msg_type == "sensor_msgs/JointState":
                return [topic]
    except Exception:
        pass
    return []

def reorder_to_franka(msg):
    """If msg.name provided and not in desired order, reorder positions accordingly."""
    if msg.name:
        name2pos = dict(zip(msg.name, msg.position))
        if all(n in name2pos for n in FRANKA_JOINT_ORDER):
            msg.position = [name2pos[n] for n in FRANKA_JOINT_ORDER]
            msg.name = list(FRANKA_JOINT_ORDER)
    return msg

def replay_bag(bag_path, pub, source_topics, speed=1.0, reorder=True, stop_flag=lambda: False):
    rospy.loginfo("Replaying: %s", bag_path)
    with rosbag.Bag(bag_path, "r") as bag:
        topics = find_joint_topics(bag, forced_topics=source_topics)
        if not topics:
            rospy.logerr("No JointState topic found. Use ~source_topics to specify.")
            return False

        prev_t = None
        for _, msg, t in bag.read_messages(topics=topics):
            if stop_flag() or rospy.is_shutdown():
                break
            # sleep by recorded delta (scaled by speed)
            if prev_t is not None:
                dt = (t - prev_t).to_sec() / max(speed, 1e-6)
                if dt > 0:
                    rospy.sleep(dt)
            prev_t = t

            # sanitize header stamp to "now" (follower usually doesn't need original time)
            msg.header.stamp = rospy.Time.now()
            if reorder:
                msg = reorder_to_franka(msg)

            pub.publish(msg)
    rospy.loginfo("Replay finished: %s", bag_path)
    return True

class TUI:
    def __init__(self, stdscr, base_dir, on_enter):
        self.stdscr = stdscr
        self.base_dir = base_dir
        self.on_enter = on_enter  # callback(path) -> None
        self.cursor = 0
        self.files = []
        self.last_refresh = 0
        self.refresh_interval = 1.0

    def _refresh_files(self, force=False):
        now = time.time()
        if force or (now - self.last_refresh) >= self.refresh_interval:
            self.files = list_bags(self.base_dir)
            self.cursor = min(self.cursor, max(0, len(self.files)-1))
            self.last_refresh = now

    def draw(self, status=""):
        self.stdscr.clear()
        h, w = self.stdscr.getmaxyx()
        title = f"rosbag_traj_replayer  |  dir: {self.base_dir}  |  q:quit  r:refresh  ENTER:replay"
        self.stdscr.addnstr(0, 0, title, w-1, curses.A_BOLD)
        for i, f in enumerate(self.files[:max(1, h-3)]):
            flag = curses.A_REVERSE if i == self.cursor else curses.A_NORMAL
            line = f"{i:>3}  {f}"
            self.stdscr.addnstr(2+i, 2, line, w-4, flag)
        if not self.files:
            self.stdscr.addnstr(2, 2, "(no .bag files)", w-4, curses.A_DIM)
        self.stdscr.addnstr(h-2, 2, status[:w-4], w-4, curses.A_DIM)
        self.stdscr.refresh()

    def loop(self):
        curses.curs_set(0)
        self.stdscr.nodelay(True)
        status = ""
        while True:
            self._refresh_files()
            self.draw(status=status)

            try:
                key = self.stdscr.getch()
            except Exception:
                key = -1

            if key == ord('q'):
                break
            elif key in (curses.KEY_UP, ord('k')):
                self.cursor = max(0, self.cursor - 1)
            elif key in (curses.KEY_DOWN, ord('j')):
                self.cursor = min(max(0, len(self.files)-1), self.cursor + 1)
            elif key == ord('r'):
                self._refresh_files(force=True)
            elif key in (10, 13):  # Enter
                if not self.files:
                    status = "No bag to replay."
                    continue
                path = os.path.join(self.base_dir, self.files[self.cursor])
                status = f"Replaying {os.path.basename(path)} ..."
                self.draw(status=status)
                self.on_enter(path)
                status = "Replay finished."
            else:
                time.sleep(0.02)

def main():
    rospy.init_node("rosbag_traj_replayer", anonymous=False)
    base_dir = rospy.get_param("~dir", DEFAULT_DIR)
    out_topic = rospy.get_param("~out_topic", "/joint_state_follower")
    source_topics = rospy.get_param("~source_topics", [])  # e.g. ["/joint_states"]
    speed = float(rospy.get_param("~speed", 1.0))
    do_reorder = bool(rospy.get_param("~reorder_to_franka", True))

    pub = rospy.Publisher(out_topic, JointState, queue_size=10)
    stop = {"flag": False}
    def _sigint(_a, _b):
        stop["flag"] = True
        rospy.signal_shutdown("SIGINT")
    signal.signal(signal.SIGINT, _sigint)

    def on_enter(path):
        stop["flag"] = False
        replay_bag(path, pub, source_topics, speed=speed, reorder=do_reorder,
                   stop_flag=lambda: stop["flag"])

    # Ensure directory exists
    if not os.path.isdir(base_dir):
        rospy.logwarn("Directory not found: %s", base_dir)
        os.makedirs(base_dir, exist_ok=True)

    curses.wrapper(lambda stdscr: TUI(stdscr, base_dir, on_enter).loop())

if __name__ == "__main__":
    main()
