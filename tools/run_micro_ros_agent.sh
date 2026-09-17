#!/bin/bash
# Convenient runner for micro-ROS Agent on Host PC

PORT="$1"
if [ -z "$PORT" ]; then
    PORT=$(ls /dev/ttyACM* 2>/dev/null | head -n 1)
    if [ -z "$PORT" ]; then
        PORT="/dev/ttyACM0"
    fi
fi

if [ ! -e "$PORT" ]; then
    echo "Error: Device $PORT not found! Please ensure STM32 USB cable is connected."
    exit 1
fi

echo "===================================================================="
echo " Starting micro-ROS Agent on $PORT (115200 baud)..."
echo " Press Ctrl+C to stop."
echo "===================================================================="

# Source ROS 2 Humble
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Error: /opt/ros/humble/setup.bash not found!"
    exit 1
fi

# Source micro-ROS workspace
if [ -f "/home/rifai/microros_ws/install/setup.bash" ]; then
    source /home/rifai/microros_ws/install/setup.bash
elif [ -f "/home/rifai/uros_ws/install/setup.bash" ]; then
    source /home/rifai/uros_ws/install/setup.bash
fi

ros2 run micro_ros_agent micro_ros_agent serial --dev "$PORT" -b 115200
