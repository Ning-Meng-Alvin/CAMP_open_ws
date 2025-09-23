#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TUI shell wrapper for launching and monitoring the inner script.

Inner target:
  cd ~/CampUsers/Pei/open_ws/src/franky_controller/scripts
  conda run -n franka_conda python -u franky_controller_moveit.py

Controls:
  - Enter: restart inner process
  - q: quit wrapper (terminates inner process)

Notes:
  - Unbuffered output enforced (PYTHONUNBUFFERED=1, python -u, stdbuf).
  - PTY mode ON by default so many programs behave like in a real terminal.
"""

import curses
import time
import os
import pty
import fcntl
import signal
import threading
import subprocess
from collections import deque
from datetime import datetime

# ---------- Settings ----------
USE_PTY = True           # give the inner process a pseudo-terminal (recommended)
MAX_LINES = 10000        # ring buffer lines

WORKDIR = os.path.expanduser("~/CampUsers/Pei/open_ws/src/franky_controller/scripts")

# Use conda-run to avoid relying on 'conda activate' in non-interactive shells.
INNER_CMD = (
    "PYTHONUNBUFFERED=1 stdbuf -oL -eL "
    "conda run -n franka_conda python -u franky_controller_moveit.py"
)

class Runner:
    """Manage the inner process and its output stream."""
    def __init__(self, workdir: str, cmd: str, use_pty: bool = True):
        self.workdir = workdir
        self.cmd = cmd
        self.use_pty = use_pty
        self.proc = None
        self.fd_master = None
        self.reader_thread = None
        self.lines = deque(maxlen=MAX_LINES)
        self.lock = threading.Lock()
        self.running = False
        self.restart_count = 0
        self.last_start = None
        self.last_stop = None

    def _append_line(self, text: str):
        """Append a single logical line (with trailing '\n' if available)."""
        with self.lock:
            self.lines.append(text)

    def launch(self):
        """Start inner process and attach reader thread."""
        self.terminate(grace=False)
        self.last_start = datetime.now()
        env = {**os.environ, "PYTHONUNBUFFERED": "1"}

        if self.use_pty:
            m, s = pty.openpty()
            self.fd_master = m
            # make master non-blocking
            fl = fcntl.fcntl(m, fcntl.F_GETFL)
            fcntl.fcntl(m, fcntl.F_SETFL, fl | os.O_NONBLOCK)

            self.proc = subprocess.Popen(
                self.cmd,
                cwd=self.workdir,
                shell=True,
                stdin=s,
                stdout=s,
                stderr=s,
                preexec_fn=os.setsid,  # new process group
                env=env,
                text=False,            # raw bytes from PTY
            )
            os.close(s)  # parent holds only master
        else:
            self.proc = subprocess.Popen(
                self.cmd,
                cwd=self.workdir,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=1,                # line-buffered in text mode
                universal_newlines=True,  # decode as text
                preexec_fn=os.setsid,
                env=env,
            )

        self.running = True
        self._append_line(f"[{self.last_start:%Y-%m-%d %H:%M:%S}] <wrapper> started (pid={self.proc.pid})\n")

        def reader_pipe():
            try:
                for line in self.proc.stdout:
                    # ensure each line ends with '\n'
                    self._append_line(line if line.endswith("\n") else line + "\n")
            except Exception as e:
                self._append_line(f"<wrapper> reader error: {e}\n")
            finally:
                rc = self.proc.poll()
                self._append_line(f"[{datetime.now():%Y-%m-%d %H:%M:%S}] <wrapper> inner exited code={rc}\n")
                self.running = False
                self.last_stop = datetime.now()

        def reader_pty():
            # accumulate bytes and split by '\n' to form logical lines
            buf = bytearray()
            try:
                while True:
                    if self.proc.poll() is not None:
                        break
                    try:
                        chunk = os.read(self.fd_master, 4096)
                        if not chunk:
                            time.sleep(0.02)
                            continue
                        buf.extend(chunk)
                        # extract complete lines
                        while True:
                            idx = buf.find(b"\n")
                            if idx == -1:
                                break
                            line = buf[:idx + 1]
                            del buf[:idx + 1]
                            text = line.decode("utf-8", errors="replace")
                            self._append_line(text)
                    except BlockingIOError:
                        time.sleep(0.02)
                # flush remainder as a final line
                if buf:
                    self._append_line(buf.decode("utf-8", errors="replace"))
            except Exception as e:
                self._append_line(f"<wrapper> reader error: {e}\n")
            finally:
                rc = self.proc.poll()
                self._append_line(f"[{datetime.now():%Y-%m-%d %H:%M:%S}] <wrapper> inner exited code={rc}\n")
                self.running = False
                self.last_stop = datetime.now()

        target = reader_pty if self.use_pty else reader_pipe
        self.reader_thread = threading.Thread(target=target, daemon=True)
        self.reader_thread.start()

    def terminate(self, grace=True, timeout=5.0):
        """Terminate inner process."""
        if self.proc is None:
            return
        try:
            if self.proc.poll() is None:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT if grace else signal.SIGTERM)
                t0 = time.time()
                while time.time() - t0 < timeout:
                    if self.proc.poll() is not None:
                        break
                    time.sleep(0.05)
                if self.proc.poll() is None:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            self.last_stop = datetime.now()
        except Exception as e:
            self._append_line(f"<wrapper> terminate error: {e}\n")
        finally:
            try:
                if self.fd_master is not None:
                    os.close(self.fd_master)
            except Exception:
                pass
            self.fd_master = None
            self.proc = None
            self.running = False

    def restart(self):
        """Restart inner process."""
        self.restart_count += 1
        self._append_line(f"\n[{datetime.now():%Y-%m-%d %H:%M:%S}] <wrapper> restarting (#{self.restart_count})...\n")
        self.launch()


def _wrap_visual_lines(raw_lines, width):
    """Soft-wrap each logical line to fit terminal width."""
    if width <= 1:
        return raw_lines
    out = []
    for ln in raw_lines:
        s = ln.rstrip("\n")
        while len(s) > width - 1:
            out.append(s[:width - 1] + "\n")
            s = s[width - 1:]
        out.append(s + ("\n" if not ln.endswith("\n") else "\n"))
    return out


def draw_tui(stdscr, runner: Runner):
    """Curses-based TUI main loop."""
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(100)  # ms

    while True:
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        # Header
        header = " Shell: franky_controller_moveit | [Enter] restart | [q] quit "
        status = "RUNNING" if runner.running else "STOPPED"
        timestr = f"start={runner.last_start:%H:%M:%S}" if runner.last_start else "start=-"
        stopstr = f"stop={runner.last_stop:%H:%M:%S}" if runner.last_stop else "stop=-"
        meta = f" | status={status} | restarts={runner.restart_count} | {timestr} {stopstr}"
        hdr = (header + meta)[: max(0, w - 1)]
        stdscr.addstr(0, 0, hdr, curses.A_REVERSE)

        # Output window
        out_top = 1
        out_height = max(1, h - out_top)

        with runner.lock:
            raw = list(runner.lines)

        # Soft-wrap lines to terminal width
        wrapped = _wrap_visual_lines(raw, w)
        view_lines = wrapped[-out_height:]

        y = out_top
        for line in view_lines:
            stdscr.addnstr(y, 0, line, max(0, w - 1))
            y += 1
            if y >= h:
                break

        stdscr.refresh()

        # Keys
        try:
            ch = stdscr.getch()
        except curses.error:
            ch = -1

        if ch in (ord("q"), ord("Q")):
            runner._append_line(f"[{datetime.now():%Y-%m-%d %H:%M:%S}] <wrapper> quitting\n")
            break
        elif ch in (curses.KEY_ENTER, 10, 13):
            runner.restart()

        time.sleep(0.05)


def main():
    runner = Runner(WORKDIR, INNER_CMD, use_pty=USE_PTY)
    runner.launch()
    try:
        curses.wrapper(draw_tui, runner)
    finally:
        runner.terminate(grace=True)


if __name__ == "__main__":
    main()
