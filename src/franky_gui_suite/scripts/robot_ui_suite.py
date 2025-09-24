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

import os, sys, re, json, signal, pathlib, shutil

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
        # 强制每个块都在视觉上换行（即使块内已有换行）
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
        self._refresh_paused = False  # pause auto-refresh while user is interacting

        layout = QtWidgets.QVBoxLayout(self)

        # ---- Header ----
        hdr = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("<b>Bag → JointState Replayer</b>")
        title.setTextFormat(Qt.RichText)
        hdr.addWidget(title); hdr.addStretch(1)
        self.status = QtWidgets.QLabel("idle"); hdr.addWidget(self.status)
        layout.addLayout(hdr)

        # ---- Folder row ----
        row = QtWidgets.QHBoxLayout()
        self.dir_edit = QtWidgets.QLineEdit(str(pathlib.Path("/home/camp/ros_bags").expanduser()))
        btn_browse = QtWidgets.QPushButton("Browse")
        row.addWidget(QtWidgets.QLabel("Folder:")); row.addWidget(self.dir_edit, 1); row.addWidget(btn_browse)
        layout.addLayout(row)

        # ---- Left list ----
        self.list_widget = QtWidgets.QListWidget()
        self.list_widget.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)  # multi-select
        self.list_widget.setMinimumWidth(420)
        self.list_widget.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.list_widget.setVerticalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
        self.list_widget.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.list_widget.setUniformItemSizes(True)
        self.list_widget.setAlternatingRowColors(True)
        # Make inactive selection look the same as active (so it doesn't "disappear")
        self.list_widget.setStyleSheet(
            "QListWidget::item:selected { background:#3d6dcc; color:#ffffff; }"
            "QListWidget::item:selected:!active { background:#3d6dcc; color:#ffffff; }"
        )
        # Pause refresh while user is interacting with the list
        self.list_widget.installEventFilter(self)

        # ---- Right controls ----
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

        layout.addWidget(inner, 1)

        # ---- Output area ----
        self.out = QtWidgets.QTextEdit(); self.out.setReadOnly(True)
        self.out.setStyleSheet("QTextEdit { background:#000; color:#fff; }")
        layout.addWidget(self.out, 1)

        # ---- Hooks ----
        btn_browse.clicked.connect(self._choose_dir)
        self.btn_refresh.clicked.connect(self.refresh_list)
        self.btn_start.clicked.connect(self.start_replay)
        self.btn_stop.clicked.connect(self.stop_replay)
        self.btn_delete.clicked.connect(self.delete_selected)

        del_action = QtWidgets.QAction(self); del_action.setShortcut(Qt.Key_Delete)
        del_action.triggered.connect(self.delete_selected); self.addAction(del_action)

        # Auto-refresh bag list every second
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.refresh_list)
        self.timer.start(1000); self.refresh_list()

    # ---- Pause auto-refresh while interacting with the list ----
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
        if self._refresh_paused:  # do nothing while user is interacting
            return
        folder = self.dir_edit.text().strip()
        items = _list_bags(folder)

        # remember previously selected names
        prev_selected = {self.list_widget.item(i).text()
                         for i in range(self.list_widget.count())
                         if self.list_widget.item(i).isSelected()}

        # rebuild list while keeping selection stable
        self.list_widget.setUpdatesEnabled(False)
        self.list_widget.clear()
        for name in items:
            it = QtWidgets.QListWidgetItem(name)
            it.setToolTip(name)
            self.list_widget.addItem(it)
        # re-select after items are in the widget
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

    def _set_status(self, s: str): self.status.setText(s)

# =========================================================
# JSON Command Viewer
# =========================================================
class JsonPane(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.commands = []; self.idx = 0
        v = QtWidgets.QVBoxLayout(self)

        h = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("<b>JSON Command Viewer</b>"); title.setTextFormat(Qt.RichText)
        h.addWidget(title); h.addStretch(1)
        self.btn_load = QtWidgets.QPushButton("Load JSON")
        h.addWidget(self.btn_load); v.addLayout(h)

        self.path_lbl = QtWidgets.QLabel("File: (none)"); v.addWidget(self.path_lbl)

        self.view = QtWidgets.QTextEdit(); self.view.setReadOnly(True)
        self.view.setStyleSheet("QTextEdit { background:#111; color:#fff; }")
        v.addWidget(self.view,1)

        nav = QtWidgets.QHBoxLayout()
        self.btn_prev = QtWidgets.QPushButton("Prev")
        self.btn_next = QtWidgets.QPushButton("Next")
        self.index_lbl = QtWidgets.QLabel("0 / 0")
        nav.addWidget(self.btn_prev); nav.addWidget(self.btn_next)
        nav.addStretch(1); nav.addWidget(self.index_lbl); v.addLayout(nav)

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
        self.setWindowTitle("Robot Multi-Panel UI (PyQt5)")
        self.resize(1680, 960)

        splitter = QtWidgets.QSplitter(Qt.Horizontal)

        # Pane 1: Telecontrol roslaunch (echo command OK; stdout only)
        self.p1 = ProcessPane("RViz / Telecontrol Launch", echo_cmd=True, capture_stderr=False)
        cmd1 = " && ".join([
            "cd ~/CampUsers/Pei/open_ws",
            "source /opt/ros/noetic/setup.bash",
            "source devel/setup.bash",
            "roslaunch franky_controller franky_telecontrol.launch"
        ])
        self.p1.set_command(cmd1)

        # Pane 2: Bag → JointState Replayer
        self.p2 = BagReplayerPane()

        # Pane 3: Controller (conda + python) — unbuffered output; no command echo; stdout only; chunk newline
        self.p3 = ProcessPane("Controller (conda + python script)",
                              echo_cmd=False, capture_stderr=False,
                              force_chunk_newline=True)
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

        # Pane 4: JSON viewer
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

        self._make_menu()

    def _make_menu(self):
        bar = self.menuBar()
        filem = bar.addMenu("File")
        act_quit = filem.addAction("Quit"); act_quit.triggered.connect(self.close)

  

    def closeEvent(self, e: QtGui.QCloseEvent):
        # Graceful shutdown on window close
        for fn in (getattr(self.p1, "stop", None),
                   getattr(self.p3, "stop", None),
                   getattr(self.p2, "stop_replay", None)):
            try:
                if callable(fn): fn()
            except Exception:
                pass
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
