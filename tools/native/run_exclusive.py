#!/usr/bin/env python3
"""Give a native Maix app exclusive display access, restoring the system menu on exit."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

LAUNCHER = "/maixapp/apps/launcher/launcher"
WATCHDOG = "/maixapp/apps/launcher/launcher_daemon"

def args(pid):
    try:
        return Path(f"/proc/{pid}/cmdline").read_bytes().rstrip(b"\0").split(b"\0")
    except OSError:
        return []

def exited(pid):
    try:
        # The paused watchdog cannot reap the menu child until it is resumed.
        return Path(f"/proc/{pid}/stat").read_text().rsplit(") ", 1)[1].startswith("Z ")
    except OSError:
        return True

def main():
    if len(sys.argv) < 2:
        raise SystemExit("Usage: run_exclusive.py executable [args...]")
    manager = None
    app = None
    try:
        entries = [p for p in Path("/proc").iterdir() if p.name.isdigit()]
        supervisors = [int(p.name) for p in entries if args(p.name) == [WATCHDOG.encode(), b"maixvision_server"]]
        if len(supervisors) > 1:
            raise RuntimeError("More than one system menu supervisor; stop the menu manually")
        if supervisors:
            manager = supervisors[0]
            # Pause even if the menu is between restarts, preventing a race
            # in which it would take the display after this app starts.
            os.kill(manager, signal.SIGSTOP)
        for entry in entries:
            if args(entry.name) != [LAUNCHER.encode(), b"daemon"]:
                continue
            launcher = int(entry.name)
            parent = int((entry / "stat").read_text().rsplit(") ", 1)[1].split()[1])
            if parent != manager:
                raise RuntimeError("Unexpected system menu supervisor; stop the menu manually")
            os.kill(launcher, signal.SIGINT)
            for _ in range(50):
                if exited(launcher):
                    break
                time.sleep(0.1)
            else:
                raise RuntimeError("System menu did not release the display")
            break
        app = subprocess.Popen(sys.argv[1:])
        def stop(_signum, _frame):
            if app.poll() is None:
                app.send_signal(signal.SIGINT)
        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGINT, stop)
        code = app.wait()
        return code if code >= 0 else 128 - code
    finally:
        if manager is not None:
            try:
                os.kill(manager, signal.SIGCONT)
            except ProcessLookupError:
                pass

if __name__ == "__main__":
    raise SystemExit(main())
