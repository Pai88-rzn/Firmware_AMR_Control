#!/bin/bash
set -e

OPENOCD_SCRIPTS="/home/rifai/.arduino15/packages/esp32/tools/openocd-esp32/v0.12.0-esp32-20251215/share/openocd/scripts"
ELF_FILE="/home/rifai/Firmware_AMR_Control/STM32_Sensor_Test/build/TFmini_I2C_Converter.elf"

if [ ! -f "$ELF_FILE" ]; then
    echo "Error: $ELF_FILE not found. Build first!"
    exit 1
fi

echo "Flashing TFmini_I2C_Converter via ST-Link V2 (OpenOCD)..."
/home/rifai/.local/bin/openocd -s "$OPENOCD_SCRIPTS" \
  -f interface/stlink.cfg \
  -f target/stm32f4x.cfg \
  -c "adapter speed 2000" \
  -c "program $ELF_FILE verify reset exit"

echo "Flash complete! Hubungkan kabel TFmini-S ke PA9 & PA10, lalu jalankan:"
echo "./STM32_Sensor_Test/run_monitor.sh"
