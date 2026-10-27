#!/bin/sh
# measure.sh: motion's RAM and message sizes, packed and unpacked
# (Week 13 evidence 2 and 3).  Run on the laptop, from anywhere:
#     sh tests/week13/measure.sh > out.txt
#     sh tests/week13/measure.sh -DMOTION_BENCH=1   # bench images for timing
# It builds the last commit (HEAD, not uncommitted edits) in two throwaway
# copies with the Docker image, once with MOTION_PACKED=1 and once with 0,
# and prints what the compiler and linker made.  The repo is not touched.

set -eu

if [ "${1:-}" != "inside" ]; then
    # ---- Part 1, on the laptop: two clean copies of HEAD, then Docker.
    REPO=$(cd "$(dirname "$0")/../.." && pwd)
    REV=$(git -C "$REPO" rev-parse --short HEAD)
    OUT=${MEASURE_OUT:-${TMPDIR:-/tmp}/motion_w13}/$REV
    rm -rf "$OUT"
    for P in 1 0; do
        mkdir -p "$OUT/p$P"
        git -C "$REPO" archive HEAD | tar -x -C "$OUT/p$P"
    done
    command -v docker > /dev/null \
        || PATH="/Applications/Docker.app/Contents/Resources/bin:$PATH"
    echo "commit $REV, car profile (SMP=1 CONSOLE=usb_cdc), extra: ${*:-none}"
    echo "copies and both .uf2 files in $OUT" >&2
    docker run --rm -v "$OUT":/w -e EXTRA="$*" pico-racer-embedded-build \
        bash -lc "sh /w/p1/tests/week13/measure.sh inside"
    exit 0
fi

# ---- Part 2, inside the container: build, then measure, each setting.
M=mtkernel_3/app_program/motor_task     # motion's objects
for P in 1 0; do
    cd /w/p$P/build_make
    # The only difference between the two copies is this -D on the compiler.
    make SMP=1 CONSOLE=usb_cdc \
        GCC="arm-none-eabi-gcc -DMOTION_PACKED=$P $EXTRA" -j8 > ../car.log 2>&1 \
        || { tail -20 ../car.log; exit 1; }
    echo
    echo "==== MOTION_PACKED=$P ($(grep -c 'warning:' ../car.log) warnings)"
    echo "-- whole image, static RAM (bytes):"
    arm-none-eabi-size -A -d mtk3pico_smp1_usb_cdc.elf \
        | awk '$1 == ".data" || $1 == ".bss" { print "  " $1, $2 }'
    echo "-- motion objects (bytes; RAM = data + bss):"
    arm-none-eabi-size -t $M/*.o | sed "s|$M/||; s|^|  |"
    echo "-- every motion RAM variable (bytes, name):"
    for o in $M/*.o; do
        arm-none-eabi-nm -S -t d $o \
            | awk -v f="${o##*/}" '$3 ~ /^[bBdD]$/ { print "  " $2 + 0, $4, f }'
    done | tee ../ram.txt
    echo "-- message sizes from the ARM compiler (bytes, name):"
    arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -std=gnu11 \
        -DMOTION_PACKED=$P -I../app_program/common -I../app_program/motor_task \
        -c ../tests/week13/sizes.c -o ../sizes.o
    arm-none-eabi-nm -S -t d ../sizes.o | awk '{ print "  " $2 + 0, $4 }'
    cp mtk3pico_smp1_usb_cdc.uf2 /w/motion_packed$P.uf2   # for car timing
    # Build check only: the default profile must build in this setting too.
    make GCC="arm-none-eabi-gcc -DMOTION_PACKED=$P $EXTRA" -j8 \
        > ../default.log 2>&1 || { tail -20 ../default.log; exit 1; }
    echo "-- default profile (make -j8) builds too," \
         "$(grep -c 'warning:' ../default.log) warnings"
done

echo
echo "==== RAM variables that differ (< packed, > unpacked):"
diff /w/p1/ram.txt /w/p0/ram.txt || true
