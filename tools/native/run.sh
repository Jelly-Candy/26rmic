#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
if pgrep -x RMIC26 >/dev/null || pgrep -x RMIC26_mount180 >/dev/null || pgrep -x RMIC26_tracking >/dev/null; then
 echo "Another RMIC26 program is running. Stop it in its original session first."
 exit 1
fi
MAIX_RUNTIME=${MAIX_RUNTIME:-/usr/local/lib/python3.13/site-packages/maix}
LEGACY_LIB_DIR=${LEGACY_LIB_DIR:-/root/dist/RMIC26_release/dl_lib}
export LD_LIBRARY_PATH="$LEGACY_LIB_DIR:$MAIX_RUNTIME/dl_lib:/usr/local/lib:/usr/lib:/soc/lib:${LD_LIBRARY_PATH:-}"
export MAIXVISION=1
export MAIXVISION_IMG_FMT=1
export MAIXVISION_IMG_QUALITY=${MAIXVISION_IMG_QUALITY:-80}
unset MAIX_UART_TEST
cd "$ROOT/build/native"
[ -e libmaix.so ] || ln -s "$MAIX_RUNTIME/_maix.so" libmaix.so
if [ "${1:-}" = "--check-mount" ]; then
 exec ./RMIC26 "$@"
fi
exec python3 "$ROOT/tools/native/run_exclusive.py" ./RMIC26 "$@"
