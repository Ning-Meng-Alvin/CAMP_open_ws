#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot UI Suite (PyQt5, single file)

Panes (left -> right):
1) RViz / Telecontrol Launch
2) Bag → JointState Replayer (GUI version of your TUI)
3) Controller (conda + python)
4) JSON Command Viewer

Notes:
- Uses `setsid` if available for proper Ctrl-C to the whole group.
- All UI labels & comments are in English.
- This version ONLY forwards STDOUT (no STDERR) for process panes by default.
"""

import os, sys, re, json, signal, pathlib, shutil, shlex

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt

# =========================================================
# ANSI → HTML converter (minimal SGR: colors, bold, reset)
# =========================================================
class AnsiToHtml:
    ANSI_RE = re.compile(r'\x1b\[((?:\d|;)+)m')
    COLOR_MAP = {
        30:"#000000",31:"#cc0000",32:"#00aa00",33:"#aa7700",
        34:"#0044cc",35:"#aa00aa",36:"#008888",37:"#bbbbbb",
        90:"#666666",91:"#ff5555",92:"#55ff55",93:"#ffff55",
        94:"#5599ff",95:"#ff55ff",96:"#55ffff",97:"#ffffff",
    }
    BG_MAP = {
        40:"#000000",41:"#cc0000",42:"#00aa00",43:"#aa7700",
        44:"#0044cc",45:"#aa00aa",46:"#008888",47:"#bbbbbb",
        100:"#666666",101:"#ff5555",102:"#55ff55",103:"#ffff55",
        104:"#5599ff",105:"#ff55ff",106:"#55ffff",107:"#ffffff",
    }
    def __init__(self): self.reset()
    def reset(self):
        self.fg=None; self.bg=None; self.bold=False; self.open=False
    def _open_span(self):
        styles=[]
        if self.fg: styles.append(f"color:{self.fg}")
        if self.bg: styles.append(f"background-color:{self.bg}")
        if self.bold: styles.append("font-weight:bold")
        if styles:
            self.open=True
            return f'<span style="{";".join(styles)}">'
        return ""
    def _close_span(self):
        if self.open:
            self.open=False
            return "</span>"
        return ""
    def _apply_codes(self, codes):
        out=""
        for c in codes:
            if c==0:
                out+=self._close_span()
                self.fg=self.bg=None; self.bold=False
            elif c==1:
                self.bold=True
            elif 30<=c<=37 or 90<=c<=97:
                self.fg=self.COLOR_MAP.get(c,self.fg)
            elif 40<=c<=47 or 100<=c<=107:
                self.bg=self.BG_MAP.get(c,self.bg)
        out+=self._close_span()
        out+=self._open_span()
        return out
    def convert(self,text:str)->str:
        def esc(s:str)->str:
            return (s.replace("&","&amp;").replace("<","&lt;")
                     .replace(">","&gt;").replace("\t","    "))
        pos=0; html=[]
        for m in self.ANSI_RE.finditer(text):
            html.append(esc(text[pos:m.start()]))
            codes=[int(x or 0) for x in m.group(1).split(";")]
            html.append(self._apply_codes(codes)); pos=m.end()
        html.append(esc(text[pos:])); html.append(self._close_span())
        return "<pre style='margin:0;white-space:pre-wrap;font-family:monospace;'>"+"".join(html)+"</pre>"

# =========================================================
# Generic process pane with ANSI output and Ctrl-C Restart
# =========================================================
class ProcessPane(QtWidgets.QWidget):
    MAX_BLOCKS = 5000

    def __init__(self, title: str, parent=None, echo_cmd: bool = True,
                 capture_stderr: bool = False, force_chunk_newline: bool = False):
        """
        :param echo_cmd: Whether to echo the full launch command into the UI.
        :param capture_stderr: If True, also forward STDERR; default False (stdout only).
        :param force_chunk_newline: If True, add a newline after EVERY received stdout chunk.
        """
        super().__init__(parent)
        self.setObjectName(title.replace(" ", "_"))
        self.ansi = AnsiToHtml()
        self.proc = None
        self.current_cmd = None
        self.echo_cmd = echo_cmd
        self.capture_stderr = capture_stderr
        self.force_chunk_newline = force_chunk_newline

        v = QtWidgets.QVBoxLayout(self)
        h = QtWidgets.QHBoxLayout()
        title_lbl = QtWidgets.QLabel(f"<b>{title}</b>")
        title_lbl.setTextFormat(Qt.RichText)
        h.addWidget(title_lbl); h.addStretch(1)
        self.btn_start = QtWidgets.QPushButton("Start")
        self.btn_restart = QtWidgets.QPushButton("Restart (Ctrl-C)")
        self.btn_stop = QtWidgets.QPushButton("Stop")
        self.btn_clear = QtWidgets.QPushButton("Clear")
        for b in (self.btn_start,self.btn_restart,self.btn_stop,self.btn_clear): h.addWidget(b)
        v.addLayout(h)

        self.out = QtWidgets.QTextEdit()
        self.out.setReadOnly(True)
        self.out.setAcceptRichText(True)
        self.out.setLineWrapMode(QtWidgets.QTextEdit.NoWrap)  # keep log lines intact
        self.out.setStyleSheet("QTextEdit { background:#000; color:#fff; }")
        v.addWidget(self.out,1)
        self.status = QtWidgets.QLabel("Status: idle")
        v.addWidget(self.status)

        self.btn_start.clicked.connect(self.start)
        self.btn_restart.clicked.connect(self.restart)
        self.btn_stop.clicked.connect(self.stop)
        self.btn_clear.clicked.connect(self.out.clear)

    @staticmethod
    def setsid_command(cmd_str: str):
        """Prefer 'setsid'; fallback to plain bash -lc."""
        if shutil.which("setsid"):
            return ["setsid","bash","-lc",cmd_str]
        return ["bash","-lc",cmd_str]

    def set_command(self, cmd_str: str): self.current_cmd = self.setsid_command(cmd_str)

    def append_ansi(self, data: bytes):
        """Append chunk with guaranteed newline, convert ANSI, and trim doc size."""
        try:
            text = data.decode("utf-8", errors="replace")
        except Exception:
            text = str(data)
        text = text.replace("\r\n", "\n").replace("\r", "\n")
        if not text.endswith("\n"):
            text += "\n"
        html = self.ansi.convert(text)
        self.out.moveCursor(QtGui.QTextCursor.End)
        self.out.insertHtml(html)

        if self.force_chunk_newline:
            self.out.insertHtml("<br/>")
        self.out.moveCursor(QtGui.QTextCursor.End)

        doc = self.out.document()
        if doc.blockCount() > self.MAX_BLOCKS:
            cursor = QtGui.QTextCursor(doc)
            cursor.movePosition(QtGui.QTextCursor.Start)
            cursor.select(QtGui.QTextCursor.BlockUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    def _on_ready_stdout(self):
        if self.proc:
            data = self.proc.readAllStandardOutput().data()
            if data: self.append_ansi(data)

    def _on_ready_stderr(self):
        if self.proc:
            data = self.proc.readAllStandardError().data()
            if data: self.append_ansi(data)

    def _on_finished(self, code, status):
        self.status.setText(f"Status: exited (code={code})"); self.proc=None

    def _send_sigint_group(self):
        if not self.proc: return
        pid = int(self.proc.processId())
        try: os.killpg(pid, signal.SIGINT)
        except Exception:
            try: self.proc.terminate()
            except Exception: pass

    def start(self):
        if not self.current_cmd:
            self.status.setText("Status: no command set"); return
        if self.proc:
            self.status.setText("Status: already running"); return
        self.out.append("<pre style='color:#aaa'>[start]</pre>")
        if self.echo_cmd:
            self.out.append(f"<pre style='color:#888'>{' '.join(self.current_cmd)}</pre>")
        self.proc = QtCore.QProcess(self)
        if self.capture_stderr:
            self.proc.setProcessChannelMode(QtCore.QProcess.MergedChannels)
            self.proc.readyReadStandardOutput.connect(self._on_ready_stdout)
        else:
            self.proc.setProcessChannelMode(QtCore.QProcess.SeparateChannels)
            self.proc.readyReadStandardOutput.connect(self._on_ready_stdout)
            if self.capture_stderr:
                self.proc.readyReadStandardError.connect(self._on_ready_stderr)
        self.proc.finished.connect(self._on_finished)
        self.proc.start(self.current_cmd[0], self.current_cmd[1:])
        self.status.setText("Status: running")

    def stop(self):
        if not self.proc:
            self.status.setText("Status: not running"); return
        self.out.append("<pre style='color:#aaa'>[stop]</pre>")
        self._send_sigint_group()
        if not self.proc.waitForFinished(2000):
            try: os.killpg(int(self.proc.processId()), signal.SIGTERM)
            except Exception: self.proc.kill()
            self.proc.waitForFinished(2000)
        self.proc=None; self.status.setText("Status: stopped")

    def restart(self):
        if self.proc:
            self.out.append("<pre style='color:#aaa'>[restart -> Ctrl-C]</pre>")
            self._send_sigint_group()
            self.proc.waitForFinished(2000)
            if self.proc:
                try: os.killpg(int(self.proc.processId()), signal.SIGTERM)
                except Exception: self.proc.kill()
                self.proc.waitForFinished(2000)
            self.proc=None
        self.start()

# =========================================================
# Bag → JointState Replayer
# =========================================================
FRANKA_JOINT_ORDER = [
    "panda_joint1","panda_joint2","panda_joint3",
    "panda_joint4","panda_joint5","panda_joint6","panda_joint7"
]

def _list_bags(dirpath: str):
    p = pathlib.Path(dirpath).expanduser()
    if not p.exists() or not p.is_dir(): return []
    files = [x for x in p.glob("*.bag*") if x.is_file()]
    files.sort(key=lambda f: f.stat().st_mtime, reverse=True)
    return [f.name for f in files]

def _find_joint_topics(bag, forced_topics=None):
    if forced_topics: return forced_topics
    info = bag.get_type_and_topic_info()
    for topic, tinfo in info.topics.items():
        if tinfo.msg_type == "sensor_msgs/JointState":
            return [topic]
    return []

def _reorder_to_franka(msg):
    if getattr(msg, "name", None):
        name2pos = dict(zip(msg.name, msg.position))
        if all(n in name2pos for n in FRANKA_JOINT_ORDER):
            msg.position = [name2pos[n] for n in FRANKA_JOINT_ORDER]
            msg.name = list(FRANKA_JOINT_ORDER)
    return msg

class BagReplayWorker(QtCore.QThread):
    progress = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal(str)
    error = QtCore.pyqtSignal(str)

    def __init__(self, bag_path: str, base_dir: str, out_topic: str,
                 source_topics_text: str, speed: float, do_reorder: bool, parent=None):
        super().__init__(parent)
        self.bag_path = bag_path
        self.base_dir = base_dir
        self.out_topic = out_topic or "/joint_state_follower"
        st = source_topics_text.strip()
        self.source_topics = [t for t in re.split(r"[,\s]+", st) if t] if st else []
        self.speed = max(1e-6, float(speed or 1.0))
        self.do_reorder = bool(do_reorder)
        self._stop = False

    def stop(self): self._stop = True

    def run(self):
        try:
            import rosbag, rospy
            from sensor_msgs.msg import JointState
            if not rospy.core.is_initialized():
                try:
                    rospy.init_node("bag_jointstate_replayer_gui", anonymous=True, disable_signals=True)
                except Exception as e:
                    self.error.emit(f"Failed to init ROS node: {e}"); return

            pub = rospy.Publisher(self.out_topic, JointState, queue_size=10)
            full_path = str(pathlib.Path(self.base_dir).expanduser() / self.bag_path)
            self.progress.emit(f"Opening bag: {full_path}")

            import time as _time
            with rosbag.Bag(full_path, "r") as bag:
                topics = _find_joint_topics(bag, self.source_topics)
                if not topics:
                    self.error.emit("No JointState topic found. Set 'Source topics' explicitly."); return
                self.progress.emit(f"Using topic(s): {', '.join(topics)}")

                prev_t = None; count = 0
                for _, msg, t in bag.read_messages(topics=topics):
                    if self._stop or rospy.is_shutdown():
                        self.progress.emit("Stopped by user."); break
                    if prev_t is not None:
                        dt = (t - prev_t).to_sec() / self.speed
                        if dt > 0: _time.sleep(dt)
                    prev_t = t
                    try: msg.header.stamp = rospy.Time.now()
                    except Exception: pass
                    if self.do_reorder: msg = _reorder_to_franka(msg)
                    pub.publish(msg); count += 1
                    if count % 200 == 0:
                        self.progress.emit(f"Published {count} messages...")
                self.finished.emit(f"Replay finished. Total messages: {count}")
        except Exception as e:
            self.error.emit(str(e))


class BagReplayerPane(QtWidgets.QWidget):
    """Mouse-friendly JointState bag replayer with stable multi-selection & delete."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.worker = None
        self.cali_proc = None
        self.cali_hand_proc = None
        self.ansi = AnsiToHtml()
        self._refresh_paused = False

        layout = QtWidgets.QVBoxLayout(self)

        hdr = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("<b>Bag → JointState Replayer</b>")
        title.setTextFormat(Qt.RichText)
        hdr.addWidget(title); hdr.addStretch(1)
        self.status = QtWidgets.QLabel("idle"); hdr.addWidget(self.status)
        layout.addLayout(hdr)

        row = QtWidgets.QHBoxLayout()
        self.dir_edit = QtWidgets.QLineEdit(str(pathlib.Path("/home/camp/ros_bags").expanduser()))
        btn_browse = QtWidgets.QPushButton("Browse")
        row.addWidget(QtWidgets.QLabel("Folder:")); row.addWidget(self.dir_edit, 1); row.addWidget(btn_browse)
        layout.addLayout(row)

        self.list_widget = QtWidgets.QListWidget()
        self.list_widget.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.list_widget.setMinimumWidth(420)
        self.list_widget.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.list_widget.setVerticalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
        self.list_widget.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.list_widget.setUniformItemSizes(True)
        self.list_widget.setAlternatingRowColors(True)
        self.list_widget.setStyleSheet(
            "QListWidget::item:selected { background:#3d6dcc; color:#ffffff; }"
            "QListWidget::item:selected:!active { background:#3d6dcc; color:#ffffff; }"
        )
        self.list_widget.installEventFilter(self)

        form = QtWidgets.QFormLayout()
        self.out_topic = QtWidgets.QLineEdit("/joint_state_follower")
        self.source_topics = QtWidgets.QLineEdit()
        self.source_topics.setPlaceholderText("/joint_states (comma/space separated; leave empty to auto-detect)")
        self.speed = QtWidgets.QDoubleSpinBox(); self.speed.setRange(0.01, 100.0); self.speed.setSingleStep(0.1); self.speed.setValue(1.0)
        self.chk_reorder = QtWidgets.QCheckBox("Reorder to Franka joint order"); self.chk_reorder.setChecked(True)
        form.addRow("Out topic:", self.out_topic)
        form.addRow("Source topics:", self.source_topics)
        form.addRow("Speed x:", self.speed)
        form.addRow("", self.chk_reorder)

        btns = QtWidgets.QHBoxLayout()
        self.btn_start   = QtWidgets.QPushButton("Start")
        self.btn_stop    = QtWidgets.QPushButton("Stop")
        self.btn_refresh = QtWidgets.QPushButton("Refresh")
        self.btn_delete  = QtWidgets.QPushButton("Delete")
        self.count_lbl   = QtWidgets.QLabel("0 bags")
        for b in (self.btn_start, self.btn_stop, self.btn_refresh, self.btn_delete): btns.addWidget(b)
        btns.addStretch(1); btns.addWidget(self.count_lbl)

        right = QtWidgets.QWidget(); right_box = QtWidgets.QVBoxLayout(right)
        right_box.addLayout(form); right_box.addLayout(btns)

        inner = QtWidgets.QSplitter(Qt.Horizontal)
        inner.addWidget(self.list_widget); inner.addWidget(right)
        inner.setStretchFactor(0, 1); inner.setStretchFactor(1, 2); inner.setSizes([480, 900])
        
        # --- MODIFICATION START: Adjust stretch factors ---
        # Give the top controls (inner) more weight (2) than the bottom log (out, 1)
        # This makes the log area smaller.
        layout.addWidget(inner, 2) 

        cali_layout = QtWidgets.QHBoxLayout()
        self.btn_cali = QtWidgets.QPushButton("eye-on-base cali")
        self.btn_cali_hand = QtWidgets.QPushButton("eye-on-hand cali")
        cali_layout.addWidget(self.btn_cali)
        cali_layout.addWidget(self.btn_cali_hand)
        cali_layout.addStretch(1)
        layout.addLayout(cali_layout)

        self.out = QtWidgets.QTextEdit(); self.out.setReadOnly(True)
        self.out.setStyleSheet("QTextEdit { background:#000; color:#fff; }")
        layout.addWidget(self.out, 1) # This log area gets less stretch weight
        # --- MODIFICATION END ---

        self.bottom_status_bar = QtWidgets.QLabel("Status: idle")
        layout.addWidget(self.bottom_status_bar)

        btn_browse.clicked.connect(self._choose_dir)
        self.btn_refresh.clicked.connect(self.refresh_list)
        self.btn_start.clicked.connect(self.start_replay)
        self.btn_stop.clicked.connect(self.stop_replay)
        self.btn_delete.clicked.connect(self.delete_selected)
        self.btn_cali.clicked.connect(self.toggle_calibration)
        self.btn_cali_hand.clicked.connect(self.toggle_calibration_hand)

        del_action = QtWidgets.QAction(self); del_action.setShortcut(Qt.Key_Delete)
        del_action.triggered.connect(self.delete_selected); self.addAction(del_action)

        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.refresh_list)
        self.timer.start(1000); self.refresh_list()

    def eventFilter(self, obj, ev):
        if obj is self.list_widget:
            t = ev.type()
            if t in (QtCore.QEvent.Enter, QtCore.QEvent.FocusIn, QtCore.QEvent.MouseButtonPress, QtCore.QEvent.KeyPress):
                self._refresh_paused = True
            elif t in (QtCore.QEvent.Leave, QtCore.QEvent.FocusOut, QtCore.QEvent.MouseButtonRelease, QtCore.QEvent.KeyRelease):
                self._refresh_paused = False
        return super().eventFilter(obj, ev)

    def _choose_dir(self):
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "Select bag folder", self.dir_edit.text())
        if d:
            self.dir_edit.setText(d); self.refresh_list()

    def refresh_list(self):
        if self._refresh_paused:
            return
        folder = self.dir_edit.text().strip()
        items = _list_bags(folder)
        prev_selected = {self.list_widget.item(i).text()
                         for i in range(self.list_widget.count())
                         if self.list_widget.item(i).isSelected()}
        self.list_widget.setUpdatesEnabled(False)
        self.list_widget.clear()
        for name in items:
            it = QtWidgets.QListWidgetItem(name)
            it.setToolTip(name)
            self.list_widget.addItem(it)
        for i in range(self.list_widget.count()):
            it = self.list_widget.item(i)
            if it.text() in prev_selected:
                it.setSelected(True)
        self.list_widget.setUpdatesEnabled(True)
        self.count_lbl.setText(f"{len(items)} bags")

    def _append(self, text: str, dim=False):
        color = "#aaa" if dim else "#fff"
        self.out.append(f"<pre style='color:{color};margin:0;font-family:monospace'>{text}</pre>")

    def start_replay(self):
        if self.worker:
            self._append("[warn] already running", True); return
        sel = self.list_widget.selectedItems()
        if not sel:
            self._append("[error] select a bag first", True); return
        if len(sel) > 1:
            self._append("[error] select exactly ONE bag to Start", True); return
        bag_name = sel[0].text()
        base_dir = self.dir_edit.text().strip()
        out_topic = self.out_topic.text().strip() or "/joint_state_follower"
        stxt = self.source_topics.text()
        speed = float(self.speed.value())
        do_reorder = self.chk_reorder.isChecked()
        self._append(f"[start] {bag_name} -> {out_topic} (speed x{speed})", True)
        self.worker = BagReplayWorker(
            bag_path=bag_name, base_dir=base_dir, out_topic=out_topic,
            source_topics_text=stxt, speed=speed, do_reorder=do_reorder
        )
        self.worker.progress.connect(lambda s: (self._append(s), self._set_status("running")))
        self.worker.finished.connect(self._on_finished)
        self.worker.error.connect(self._on_error)
        self.worker.start(); self._set_status("running")

    def stop_replay(self):
        if self.worker:
            self._append("[stop] requested", True)
            self.worker.stop()

    def delete_selected(self):
        sel = self.list_widget.selectedItems()
        if not sel:
            self._append("[error] select one or more bags to delete", True)
            return
        folder = self.dir_edit.text().strip()
        names = [it.text() for it in sel]
        msg = "Delete these files?\n\n" + "\n".join(names)
        self._refresh_paused = True
        res = QtWidgets.QMessageBox.question(self, "Confirm delete", msg,
                                             QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                                             QtWidgets.QMessageBox.No)
        self._refresh_paused = False
        if res != QtWidgets.QMessageBox.Yes:
            return
        removed, failed = 0, []
        for name in names:
            path = str(pathlib.Path(folder).expanduser() / name)
            try:
                os.remove(path); removed += 1
            except Exception as e:
                failed.append(f"{name}: {e}")
        self._append(f"[delete] removed {removed} file(s)", True)
        if failed:
            self._append("[error] " + "; ".join(failed), True)
        self.refresh_list()

    def _on_finished(self, s: str):
        self._append(s, True); self._cleanup("idle")

    def _on_error(self, s: str):
        self._append(f"[error] {s}", True); self._cleanup("idle")

    def _cleanup(self, status: str):
        if self.worker: self.worker.wait(200)
        self.worker=None; self._set_status(status)

    def _set_status(self, s: str):
        self.status.setText(s)
        self.bottom_status_bar.setText(f"Status: {s}")

    def _get_clean_environment(self):
        env = QtCore.QProcessEnvironment.systemEnvironment()
        for key in env.keys():
            if key.startswith("QT_"):
                env.remove(key)
        return env

    def _create_pty_command(self, base_cmd_str):
        parts = base_cmd_str.split("roslaunch", 1)
        if len(parts) != 2:
            inner_cmd = base_cmd_str
        else:
            setup_part = parts[0]
            roslaunch_part = "roslaunch" + parts[1]
            if "--screen" not in roslaunch_part:
                roslaunch_part = roslaunch_part.replace("roslaunch", "roslaunch --screen", 1)
            inner_cmd = f"{setup_part} stdbuf -oL -eL {roslaunch_part}"
        inner = f"export TERM=xterm-256color; {inner_cmd}"
        wrapped = f"script -q -c {shlex.quote(inner)} /dev/null"
        return ["bash", "-lc", wrapped]

    def toggle_calibration(self):
        if self.cali_proc: self.stop_calibration()
        else: self.start_calibration()

    def start_calibration(self):
        if self.cali_proc:
            self._append("[cali-base] Already running.", True); return
        self._append("[cali-base] Starting...", True)
        self.btn_cali.setText("Stop (base)")
        cmd_str = (
            "cd ~/CampUsers/Pei/open_ws && "
            "source /opt/ros/noetic/setup.bash && "
            "source devel/setup.bash && "
            "roslaunch easy_handeye panda_realsense_eyeonbase.launch"
        )
        final_cmd = self._create_pty_command(cmd_str)
        self._append(f"[cali-base] Running: {' '.join(final_cmd)}", True)
        self.cali_proc = QtCore.QProcess(self)
        clean_env = self._get_clean_environment()
        self.cali_proc.setProcessEnvironment(clean_env)
        self.cali_proc.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.cali_proc.finished.connect(self._on_cali_finished)
        self.cali_proc.start(final_cmd[0], final_cmd[1:])

    def stop_calibration(self):
        self._append("[cali-base] Stopping process...", True)
        if self.cali_proc and self.cali_proc.state() != QtCore.QProcess.NotRunning:
            try: os.kill(self.cali_proc.processId(), signal.SIGINT)
            except OSError: self.cali_proc.terminate()
        QtCore.QTimer.singleShot(2000, self._cleanup_cali)

    def _on_cali_finished(self, code, status):
        self._append(f"[cali-base] Process finished (code={code})", True)
        self._cleanup_cali()

    def _cleanup_cali(self):
        self.cali_proc = None
        self.btn_cali.setText("eye-on-base cali")
        self._append("[cali-base] Sequence finished or stopped.", True)

    def toggle_calibration_hand(self):
        if self.cali_hand_proc: self.stop_calibration_hand()
        else: self.start_calibration_hand()

    def start_calibration_hand(self):
        if self.cali_hand_proc:
            self._append("[cali-hand] Already running.", True); return
        self._append("[cali-hand] Starting...", True)
        self.btn_cali_hand.setText("Stop (hand)")
        cmd_str = (
            "cd ~/CampUsers/Pei/open_ws && "
            "source /opt/ros/noetic/setup.bash && "
            "source devel/setup.bash && "
            "roslaunch easy_handeye panda_realsense_eyeonhand.launch"
        )
        final_cmd = self._create_pty_command(cmd_str)
        self._append(f"[cali-hand] Running: {' '.join(final_cmd)}", True)
        self.cali_hand_proc = QtCore.QProcess(self)
        clean_env = self._get_clean_environment()
        self.cali_hand_proc.setProcessEnvironment(clean_env)
        self.cali_hand_proc.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.cali_hand_proc.finished.connect(self._on_cali_hand_finished)
        self.cali_hand_proc.start(final_cmd[0], final_cmd[1:])

    def stop_calibration_hand(self):
        self._append("[cali-hand] Stopping process...", True)
        if self.cali_hand_proc and self.cali_hand_proc.state() != QtCore.QProcess.NotRunning:
            try: os.kill(self.cali_hand_proc.processId(), signal.SIGINT)
            except OSError: self.cali_hand_proc.terminate()
        QtCore.QTimer.singleShot(2000, self._cleanup_cali_hand)

    def _on_cali_hand_finished(self, code, status):
        self._append(f"[cali-hand] Process finished (code={code})", True)
        self._cleanup_cali_hand()

    def _cleanup_cali_hand(self):
        self.cali_hand_proc = None
        self.btn_cali_hand.setText("eye-on-hand cali")
        self._append("[cali-hand] Sequence finished or stopped.", True)

# =========================================================
# JSON Command Viewer
# =========================================================
class JsonPane(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.commands = []; self.idx = 0
        v_main = QtWidgets.QVBoxLayout(self)

        top_widget = QtWidgets.QWidget()
        v_top = QtWidgets.QVBoxLayout(top_widget)
        v_top.setContentsMargins(0, 0, 0, 0)

        h = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("<b>JSON Command Viewer</b>"); title.setTextFormat(Qt.RichText)
        h.addWidget(title); h.addStretch(1)
        self.btn_load = QtWidgets.QPushButton("Load JSON")
        h.addWidget(self.btn_load)
        v_top.addLayout(h)

        self.path_lbl = QtWidgets.QLabel("File: (none)")
        v_top.addWidget(self.path_lbl)
        v_main.addWidget(top_widget)

        self.view = QtWidgets.QTextEdit(); self.view.setReadOnly(True)
        self.view.setStyleSheet("QTextEdit { background:#111; color:#fff; }")
        v_main.addWidget(self.view, 1)

        nav = QtWidgets.QHBoxLayout()
        self.btn_prev = QtWidgets.QPushButton("Prev")
        self.btn_next = QtWidgets.QPushButton("Next")
        self.index_lbl = QtWidgets.QLabel("0 / 0")
        nav.addWidget(self.btn_prev); nav.addWidget(self.btn_next)
        nav.addStretch(1); nav.addWidget(self.index_lbl)
        v_main.addLayout(nav)

        self.btn_load.clicked.connect(self.load_json)
        self.btn_prev.clicked.connect(self.prev)
        self.btn_next.clicked.connect(self.next)

    def load_json(self):
        fn, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Select JSON file", str(pathlib.Path.home()), "JSON (*.json);;All (*)")
        if not fn: return
        try:
            with open(fn, "r", encoding="utf-8") as f: data = json.load(f)
            if isinstance(data, dict) and "commands" in data and isinstance(data["commands"], list):
                cmds = [str(x) for x in data["commands"]]
            elif isinstance(data, list):
                cmds = [str(x) for x in data]
            else:
                raise ValueError("JSON must be list[str] or {'commands': list[str]}")
            self.commands = cmds; self.idx = 0
            self.path_lbl.setText(f"File: {fn}"); self._render()
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Load error", str(e))

    def _render(self):
        n = len(self.commands)
        if n == 0:
            self.view.setPlainText("(no commands)"); self.index_lbl.setText("0 / 0"); return
        self.idx = max(0, min(self.idx, n-1))
        self.view.setPlainText(self.commands[self.idx])
        self.index_lbl.setText(f"{self.idx+1} / {n}")

    def prev(self):
        if self.commands:
            self.idx = (self.idx - 1) % len(self.commands); self._render()

    def next(self):
        if self.commands:
            self.idx = (self.idx + 1) % len(self.commands); self._render()

# =========================================================
# Main Window
# =========================================================
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Multi-Panel UI")
        self.resize(1680, 960)

        splitter = QtWidgets.QSplitter(Qt.Horizontal)

        # ---------------------------
        # Pane 1: RViz / Telecontrol Launch
        # ---------------------------
        self.p1 = ProcessPane(
            "RViz / Telecontrol Launch",
            echo_cmd=False,
            capture_stderr=True,
            force_chunk_newline=True
        )
        cmd1 = " && ".join([
            "source ~/.bashrc",
            "cd ~/CampUsers/Pei/open_ws",
            "source /opt/ros/noetic/setup.bash",
            "source devel/setup.bash",
            "roslaunch franky_controller franky_telecontrol.launch"
        ])
        self.p1.set_command(cmd1)

        def _start_override():
            if not self.p1.current_cmd:
                self.p1.status.setText("Status: no command set"); return
            if self.p1.proc:
                self.p1.status.setText("Status: already running"); return
            orig_cmd_str = self.p1.current_cmd[-1] if isinstance(self.p1.current_cmd, (list,tuple)) else str(self.p1.current_cmd)
            if "roslaunch" in orig_cmd_str and "--screen" not in orig_cmd_str:
                orig_cmd_str = orig_cmd_str.replace("roslaunch ", "roslaunch --screen ", 1)
            inner = "export TERM=xterm-256color FORCE_COLOR=1; stdbuf -oL -eL " + orig_cmd_str
            wrapped = f"script -q -c {shlex.quote(inner)} /dev/null"
            self.p1.out.append("<pre style='color:#aaa'>[start]</pre>")
            self.p1.proc = QtCore.QProcess(self.p1)
            self.p1.proc.setProcessChannelMode(QtCore.QProcess.MergedChannels)
            self.p1.proc.readyReadStandardOutput.connect(lambda:
                self.p1.append_ansi(self.p1.proc.readAllStandardOutput().data())
            )
            self.p1.proc.finished.connect(self.p1._on_finished)
            base = list(self.p1.current_cmd)
            base[-1] = wrapped
            self.p1.proc.start(base[0], base[1:])
            self.p1.status.setText("Status: running")
        self.p1.start = _start_override

        def _restart_override():
            if self.p1.proc:
                self.p1.out.append("<pre style='color:#aaa'>[restart -> Ctrl-C]</pre>")
                self.p1._send_sigint_group()
                self.p1.proc.waitForFinished(2000)
                if self.p1.proc:
                    try: os.killpg(int(self.p1.proc.processId()), signal.SIGTERM)
                    except Exception: self.p1.proc.kill()
                    self.p1.proc.waitForFinished(2000)
                self.p1.proc = None
            self.p1.start()
        self.p1.restart = _restart_override

        def _stop_override():
            if not self.p1.proc:
                self.p1.status.setText("Status: not running"); return
            self.p1.out.append("<pre style='color:#aaa'>[stop]</pre>")
            self.p1._send_sigint_group()
            if not self.p1.proc.waitForFinished(2000):
                try: os.killpg(int(self.p1.proc.processId()), signal.SIGTERM)
                except Exception: self.p1.proc.kill()
                self.p1.proc.waitForFinished(2000)
            self.p1.proc = None
            self.p1.status.setText("Status: stopped")
        self.p1.stop = _stop_override

        self.p2 = BagReplayerPane()

        self.p3 = ProcessPane(
            "Controller",
            echo_cmd=False, capture_stderr=False, force_chunk_newline=True
        )
        cmd3 = " && ".join([
            'if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then source "$HOME/miniconda3/etc/profile.d/conda.sh"; '
            'elif [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then source "$HOME/anaconda3/etc/profile.d/conda.sh"; '
            'elif command -v conda >/dev/null 2>&1; then source "$(conda info --base)/etc/profile.d/conda.sh"; fi',
            "conda activate franka_conda",
            "cd ~/CampUsers/Pei/open_ws/src/franky_controller/scripts",
            "export PYTHONUNBUFFERED=1",
            "python3 -u franky_controller_moveit.py"
        ])
        self.p3.set_command(cmd3)

        self.p4 = JsonPane()

        splitter.addWidget(self.p1)
        splitter.addWidget(self.p2)
        splitter.addWidget(self.p3)
        splitter.addWidget(self.p4)

        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 3)
        splitter.setStretchFactor(2, 2)
        splitter.setStretchFactor(3, 1)
        self.setCentralWidget(splitter)

        bar = self.menuBar()
        filem = bar.addMenu("File")
        act_quit = filem.addAction("Quit"); act_quit.triggered.connect(self.close)

    def closeEvent(self, e: QtGui.QCloseEvent):
        for fn in (getattr(self.p1, "stop", None),
                   getattr(self.p3, "stop", None),
                   getattr(self.p2, "stop_replay", None),
                   getattr(self.p2, "stop_calibration", None),
                   getattr(self.p2, "stop_calibration_hand", None)):
            try:
                if callable(fn): fn()
            except Exception:
                pass

        import subprocess
        print("[Shutdown] Attempting to kill all ROS processes to ensure a clean exit...")
        commands_to_run = [
            ["killall", "-9", "rosout"],
            ["killall", "-9", "roslaunch"]
        ]
        for cmd in commands_to_run:
            try:
                subprocess.run(cmd, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print(f"[Shutdown] Executed: {' '.join(cmd)}")
            except Exception as ex:
                print(f"[Shutdown] Error executing {' '.join(cmd)}: {ex}")

        super().closeEvent(e)

def main():
    app = QtWidgets.QApplication(sys.argv)
    QtGui.QGuiApplication.setDesktopFileName("robot_ui_suite.desktop")
    app.setApplicationName("Robot UI Suite")
    app.setWindowIcon(QtGui.QIcon("/home/camp/.local/share/icons/robot_ui_suite.jpeg"))
    w = MainWindow(); w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()