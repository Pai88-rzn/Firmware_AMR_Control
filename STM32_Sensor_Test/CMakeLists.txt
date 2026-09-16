cmake_minimum_required(VERSION 3.20)
project(STM32_Sensor_Test C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Device target definitions for STM32F401CC
add_definitions(
    -DSTM32F401xC
    -DUSE_HAL_DRIVER
    -D__ARM_ARCH_7EM__=1
    -DARM_MATH_CM4
)

# Compiler flags for Cortex-M4F with Hardware FPU
set(CPU_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CPU_FLAGS} -Wall -Wextra -fdata-sections -ffunction-sections -Os")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${CPU_FLAGS} -x assembler-with-cpp")

# Linker flags (including -u _printf_float for newlib-nano float printf support)
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32F401CCUx_FLASH.ld")
set(CMAKE_EXE_LINKER_FLAGS
    "${CPU_FLAGS} -T\"${LINKER_SCRIPT}\" -Wl,-Map=${PROJECT_NAME}.map,--cref -Wl,--gc-sections -specs=nano.specs -u _printf_float -lc -lm -Wl,--no-warn-rwx-segments"
)

# Include directories
include_directories(
    Core/Inc
    Drivers/BSP
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    USB_DEVICE/App
    USB_DEVICE/Target
)

# Common shared files
set(COMMON_SOURCES
    Core/Src/stm32f4xx_it.c
    Core/Src/system_stm32f4xx.c
    Core/Src/syscalls.c
    Core/Src/startup_stm32f401xc.s

    # Drivers BSP
    Drivers/BSP/onboard_leds.c

    # USB Device Stack
    USB_DEVICE/App/usb_device.c
    USB_DEVICE/App/usbd_desc.c
    USB_DEVICE/App/usbd_cdc_if.c
    USB_DEVICE/Target/usbd_conf.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c

    # STM32F4xx HAL Drivers
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
)

# ----------------------------------------------------
# TARGET 1: STM32_Sensor_Test (Main Sensor Diagnostics)
# ----------------------------------------------------
set(SENSOR_TEST_SOURCES
    Core/Src/main.c
    Drivers/BSP/tfmini_s_lidar.c
    Drivers/BSP/dyp_a22_ultrasonic.c
    ${COMMON_SOURCES}
)

add_executable(STM32_Sensor_Test.elf ${SENSOR_TEST_SOURCES})

add_custom_command(TARGET STM32_Sensor_Test.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:STM32_Sensor_Test.elf> STM32_Sensor_Test.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:STM32_Sensor_Test.elf> STM32_Sensor_Test.bin
    COMMAND ${CMAKE_SIZE_UTIL} --format=berkeley $<TARGET_FILE:STM32_Sensor_Test.elf>
)

# ----------------------------------------------------
# TARGET 2: TFmini_I2C_Converter (One-shot Mode Changer)
# ----------------------------------------------------
set(CONVERTER_SOURCES
    Core/Src/main_converter.c
    Drivers/BSP/tfmini_s_lidar.c
    Drivers/BSP/dyp_a22_ultrasonic.c
    ${COMMON_SOURCES}
)

add_executable(TFmini_I2C_Converter.elf ${CONVERTER_SOURCES})

add_custom_command(TARGET TFmini_I2C_Converter.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:TFmini_I2C_Converter.elf> TFmini_I2C_Converter.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:TFmini_I2C_Converter.elf> TFmini_I2C_Converter.bin
    COMMAND ${CMAKE_SIZE_UTIL} --format=berkeley $<TARGET_FILE:TFmini_I2C_Converter.elf>
)
