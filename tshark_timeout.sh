#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FILE="${1}"

tshark -i COM9 -w "$FILE" &
TSHARK_PID=$!

sleep 5

probe-rs reset --chip nRF52840_xxAA

LAST_SIZE=0
IDLE=0

while kill -0 $TSHARK_PID 2>/dev/null; do
    SIZE=$(stat -c%s "$FILE" 2>/dev/null || echo 0)

    if [ "$SIZE" -eq "$LAST_SIZE" ]; then
        IDLE=$((IDLE + 1))
    else
        IDLE=0
    fi

    LAST_SIZE=$SIZE

    if [ "$IDLE" -ge 10 ]; then
        echo "[-] Capture inactive for 10s"
        kill $TSHARK_PID
        break
    fi

    sleep 1
done