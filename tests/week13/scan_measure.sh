#!/bin/sh
# scan_measure.sh: Buddy 5's RAM and message sizes before and after the
# minimum-bit change (Week 13 evidence 2 and 3).  Run on the laptop:
#     sh tests/week13/scan_measure.sh [AFTER_REV] > out.txt
# BEFORE is c3dea9d ("before requirement change for obstacle task"); AFTER
# defaults to HEAD.  Each revision is built from a throwaway `git archive`
# copy inside the Docker image, so the working tree is not touched (commit
# first: uncommitted edits are not measured).  Results are written up in
# docs/obstacle/week13_representation.md.

set -eu

BEFORE=c3dea9d

if [ "${1:-}" != "inside" ]; then
    # ---- Part 1, on the laptop: two clean copies, then Docker.
    REPO=$(cd "$(dirname "$0")/../.." && pwd)
    AFTER=$(git -C "$REPO" rev-parse --short "${1:-HEAD}")
    OUT=${MEASURE_OUT:-${TMPDIR:-/tmp}/scan_w13}/$AFTER
    rm -rf "$OUT"
    mkdir -p "$OUT/before" "$OUT/after"
    git -C "$REPO" archive "$BEFORE" | tar -x -C "$OUT/before"
    git -C "$REPO" archive "$AFTER" | tar -x -C "$OUT/after"
    # the size probe always comes from the AFTER tree
    echo "before $BEFORE, after $AFTER, car profile (SMP=1 CONSOLE=usb_cdc)"
    # Git Bash on Windows: hand docker a C:/ path and stop MSYS rewriting /w
    HOST=$(cd "$OUT" && { pwd -W 2> /dev/null || pwd; })
    MSYS_NO_PATHCONV=1 docker run --rm -v "$HOST":/w pico-racer-embedded-build \
        bash -lc "sh /w/after/tests/week13/scan_measure.sh inside"
    exit 0
fi

# ---- Part 2, inside the container: build each copy, then measure.
for V in before after; do
    cd /w/$V/build_make
    make SMP=1 CONSOLE=usb_cdc -j8 > ../car.log 2>&1 \
        || { echo "== $V: build failed"; tail -20 ../car.log; continue; }
    echo
    echo "==== $V ($(grep -c 'warning:' ../car.log) warnings)"
    echo "-- whole image, static RAM (bytes):"
    arm-none-eabi-size -A -d mtk3pico_smp1_usb_cdc.elf \
        | awk '$1 == ".data" || $1 == ".bss" { print "  " $1, $2 }'
    echo "-- scan_task + nav_ctrl objects (bytes; RAM = data + bss):"
    arm-none-eabi-size -t mtkernel_3/app_program/obstacle_task/*.o \
        mtkernel_3/app_program/nav_task/*.o \
        | sed 's|mtkernel_3/app_program/||; s|^|  |'
    echo "-- every scan_task / nav_ctrl RAM variable (bytes, name, object):"
    for o in mtkernel_3/app_program/obstacle_task/*.o \
             mtkernel_3/app_program/nav_task/*.o; do
        arm-none-eabi-nm -S -t d "$o" \
            | awk -v f="${o##*/}" '$3 ~ /^[bBdD]$/ { print "  " $2 + 0, $4, f }'
    done
done

echo
echo "==== type and message sizes from the ARM compiler (bytes, name):"
cd /w/after
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -std=gnu11 -Wall \
    -Iapp_program/common -Iapp_program/obstacle_task \
    -c tests/week13/scan_sizes.c -o /w/scan_sizes.o
arm-none-eabi-nm -S -t d /w/scan_sizes.o | awk '{ print "  " $2 + 0, $4 }' | sort -k2
