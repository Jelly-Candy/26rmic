#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
[ "$ROOT" = /root/MaixCAM2_virtual_vision_20261107 ] || { echo "Extract the project into /root first"; exit 1; }
[ -x "$ROOT/build/native/VirtualVision" ] || { echo "Build first: make -j1 all test"; exit 1; }
if [ -f /root/vx ]; then cp -p /root/vx "/root/vx.before_virtual_vision.$(date +%Y%m%d_%H%M%S)"; fi
cp "$ROOT/vx" /root/vx
printf 'Installed virtual-game vision entry: sh ~/vx\n'
