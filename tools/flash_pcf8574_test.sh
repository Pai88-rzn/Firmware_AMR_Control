#!/bin/bash
set -e

OPENOCD_SCRIPTS="/home/rifai/.arduino15/packages/esp32/tools/openocd-esp32/v0.12.0-esp32-20251215/share/openocd/scripts"
ELF_FILE="/home/rifai/Firmware_AMR_Control/STM32_Sensor_Test/build/STM32_PCF8574_Test.elf"

if [ ! -f "$ELF_FILE" ]; then
    echo "Error: $ELF_FILE not found. Building now..."
    export PATH="/home/rifai/.local/bin:$PATH"
    cmake --build /home/rifai/Firmware_AMR_Control/STM32_Sensor_Test/build --target STM32_PCF8574_Test.elf
fi

echo "Flashing STM32_PCF8574_Test via ST-Link V2 (OpenOCD)..."
/home/rifai/.local/bin/openocd -s "$OPENOCD_SCRIPTS" \
  -f interface/stlink.cfg \
  -f target/stm32f4x.cfg \
  -c "adapter speed 2000" \
  -c "program $ELF_FILE verify reset exit"

echo ""
echo "===================================================================="
echo "Flash Complete! STM32 is reset and running PCF8574 Diagnostics."
echo "Port: /dev/ttyACM0"
echo "To monitor and control: ./STM32_Sensor_Test/run_monitor.sh"
echo "===================================================================="
