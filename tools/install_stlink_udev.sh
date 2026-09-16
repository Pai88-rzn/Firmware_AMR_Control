#!/bin/bash
set -e

RULE_SRC="/home/rifai/.arduino15/packages/esp32/tools/openocd-esp32/v0.12.0-esp32-20251215/share/openocd/contrib/60-openocd.rules"
RULE_DST="/etc/udev/rules.d/60-openocd.rules"

echo "Installing ST-Link udev rules..."
cp "$RULE_SRC" "$RULE_DST"
udevadm control --reload-rules
udevadm trigger
echo "Done! Please replug the ST-Link USB cable now."
