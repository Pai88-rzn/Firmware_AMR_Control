cmake_minimum_required(VERSION 3.20)
project(STM32_Bridge C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Device target definitions
add_definitions(
    -DSTM32F411xE
    -DUSE_HAL_DRIVER
    -D__ARM_ARCH_7EM__=1
    -DARM_MATH_CM4
)

# Compiler flags for Cortex-M4F with Hardware FPU
set(CPU_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CPU_FLAGS} -Wall -Wextra -fdata-sections -ffunction-sections -Os")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${CPU_FLAGS} -x assembler-with-cpp")

# Linker flags
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32F411CEUx_FLASH.ld")
set(CMAKE_EXE_LINKER_FLAGS
    "${CPU_FLAGS} -T\"${LINKER_SCRIPT}\" -Wl,-Map=${PROJECT_NAME}.map,--cref -Wl,--gc-sections -specs=nano.specs -lc -lm -Wl,--no-warn-rwx-segments"
)

# Include directories
include_directories(
    Core/Inc
    Drivers/BSP
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
    Middlewares/Third_Party/FreeRTOS/include
    Middlewares/Third_Party/FreeRTOS/portable/GCC/ARM_CM4F
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    USB_DEVICE/App
    USB_DEVICE/Target
    App
    Middlewares/micro_ros_stm32
    Middlewares/micro_ros_stm32/include
)

# Source files
set(SOURCES
    # Core System & Startup
    Core/Src/main.c
    Core/Src/stm32f4xx_it.c
    Core/Src/system_stm32f4xx.c
    Core/Src/syscalls.c
    Core/Src/startup_stm32f411xe.s

    # Drivers BSP
    Drivers/BSP/onboard_leds.c
    Drivers/BSP/pcf8574.c
    Drivers/BSP/tfmini_s_lidar.c
    Drivers/BSP/dyp_a22_ultrasonic.c
    Drivers/BSP/lora_module.c

    # Application Layer
    App/rtos_queues.c
    App/rtos_tasks.c
    App/safety_monitor.c
    App/io_controller.c
    App/microros_interface.c

    # FreeRTOS Kernel
    Middlewares/Third_Party/FreeRTOS/croutine.c
    Middlewares/Third_Party/FreeRTOS/event_groups.c
    Middlewares/Third_Party/FreeRTOS/list.c
    Middlewares/Third_Party/FreeRTOS/queue.c
    Middlewares/Third_Party/FreeRTOS/stream_buffer.c
    Middlewares/Third_Party/FreeRTOS/tasks.c
    Middlewares/Third_Party/FreeRTOS/timers.c
    Middlewares/Third_Party/FreeRTOS/portable/MemMang/heap_4.c
    Middlewares/Third_Party/FreeRTOS/portable/GCC/ARM_CM4F/port.c

    # USB Device Stack
    USB_DEVICE/App/usb_device.c
    USB_DEVICE/App/usbd_desc.c
    USB_DEVICE/App/usbd_cdc_if.c
    USB_DEVICE/Target/usbd_conf.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c

    # micro-ROS Middleware & Transport
    Middlewares/micro_ros_stm32/custom_transport.c

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

add_executable(${PROJECT_NAME}.elf ${SOURCES})

# Link precompiled micro-ROS static library for Cortex-M4F
target_link_libraries(${PROJECT_NAME}.elf PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/micro_ros_stm32/libmicroros.a
)

# Post-build: Generate .hex, .bin, and print memory footprint
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE_UTIL} --format=berkeley $<TARGET_FILE:${PROJECT_NAME}.elf>
    COMMENT "Building ${PROJECT_NAME}.hex and ${PROJECT_NAME}.bin and reporting memory size..."
)
