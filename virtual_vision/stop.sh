#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
[ -f "$ROOT/live.pid" ] || { echo "No background PID; use Ctrl+C in the foreground session"; exit 1; }
pid=$(cat "$ROOT/live.pid")
case "$pid" in ''|*[!0-9]*) echo "Invalid PID"; exit 1;; esac
if [ "$(readlink "/proc/$pid/exe" 2>/dev/null || true)" = "$ROOT/build/native/VirtualVision" ]; then
 kill -INT "$pid"
 n=0
 while [ -e "/proc/$pid/exe" ] && [ "$n" -lt 5 ]; do sleep 1; n=$((n+1)); done
 [ ! -e "/proc/$pid/exe" ] || { echo "Program is still stopping"; exit 1; }
 echo "Stopped this project's background vision instance"
else
 echo "Recorded vision process is not running; no process was stopped"
fi
