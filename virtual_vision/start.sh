#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
. "$ROOT/camera.conf"
# A stopped child may remain as a zombie until its parent reaps it.
# Only a process with a live executable can still own the camera.
for pid in $(pgrep -x VirtualVision || true) $(pgrep -x RMIC26 || true) $(pgrep -x RMIC26_mount180 || true) $(pgrep -x RMIC26_tracking || true); do
 [ -e "/proc/$pid/exe" ] || continue
 echo "A camera program is already running. Stop it before starting VirtualVision." >&2
 exit 1
done
mkdir -p "$ROOT/logs"
export VISION_LOG_DIR="$ROOT/logs"
export MAIXVISION=1 MAIXVISION_IMG_FMT=1
RUNTIME=${MAIX_RUNTIME:-/usr/local/lib/python3.13/site-packages/maix}
LEGACY=${LEGACY_LIB_DIR:-/root/dist/RMIC26_release/dl_lib}
export LD_LIBRARY_PATH="$LEGACY:$RUNTIME/dl_lib:/usr/local/lib:/usr/lib:/opt/lib:/opt/usr/lib:/soc/lib:${LD_LIBRARY_PATH:-}"
cd "$ROOT/build/native"
[ -e libmaix.so ] || ln -s "$RUNTIME/_maix.so" libmaix.so
exec 9>/tmp/maix_virtual_vision.lock
flock -n 9 || { echo "Camera vision launch is already in progress"; exit 1; }
exec ./VirtualVision "$@"
