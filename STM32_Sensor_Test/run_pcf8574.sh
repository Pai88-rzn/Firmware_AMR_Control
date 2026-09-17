#!/bin/bash
# Auto-detect available CDC-ACM port (e.g. /dev/ttyACM0 or /dev/ttyACM1)
if [ -n "$1" ]; then
    PORT="$1"
else
    PORT=$(ls /dev/ttyACM* 2>/dev/null | head -n 1)
    if [ -z "$PORT" ]; then
        PORT="/dev/ttyACM0"
    fi
fi
python3 "$(dirname "$0")/monitor_pcf8574.py" "$PORT"
