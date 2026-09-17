# AGENTS.MD - AMR FIRMWARE CONTROL SYSTEM
**General Purpose AMR Platform (GPRP)**
*Autonomous Mobile Robot Dual-MCU Firmware Architecture & Hardware Reference*

---

## 1. Project Overview & System Architecture

This repository contains the firmware development projects for the **General Purpose AMR Platform (GPRP)**. The embedded control architecture is distributed across two dedicated STM32 microcontroller units (MCUs):

1. **STM32_Bridge (Microcontroller Bridge Board)**:
   - **MCU**: **STM32F411CEU6TR** (Arm Cortex-M4 @ 100 MHz, 512 KB Flash, 128 KB SRAM, UFQFPN-48).
   - **Role**: Central I/O coordinator, hard real-time safety monitor, sensor hub, actuator relay controller, and LoRa telemetry interface.
   - **RTOS & Middleware**: **FreeRTOS** preemptive scheduler with thread-safe Inter-Task Queues (`xTelemetryQueue`, `xRelayCmdQueue`, `xEmergencyQueue`).
   - **Host Interfacing**: Direct Native USB FS (CDC-ACM) running **micro-ROS Client (XRCE-DDS)** directly connecting to ROS 2 Humble/Iron/Jazzy (`micro-ros-agent`) on the Host IPC / SBC (e.g. Jetson Orin / x86 ROS 2 stack).

2. **STM32_IMU (Inertial Measurement Unit Board)**:
   - **MCU**: **STM32F103C8T6** (Arm Cortex-M3 @ 72 MHz, 64/128 KB Flash, 20 KB SRAM, LQFP-48).
   - **Sensor**: **Bosch BNO055** 9-axis intelligent absolute orientation sensor with hardware sensor fusion running on an external 32.768 kHz precision crystal.
   - **Role**: Dedicated high-frequency IMU calculation, preprocessing to SI units, and direct ROS 2 publication.
   - **Host Interfacing**: Dedicated USB Type-C port via onboard **FT231XS-R** USB-to-UART running **micro-ROS Client (XRCE-DDS @ 921600 baud)**, directly publishing `/imu` (`sensor_msgs/msg/Imu`) to ROS 2 without intermediate host Python scripts.

```
 +---------------------------------------------------------------------------------------------+
 |                                  HOST COMPUTER (IPC / ROS 2)                                |
 |                                                                                             |
 |   +---------------------------------------------+             +-------------------------+   |
 |   |          micro-ros-agent Daemon             |             |  diff_drive_controller  |   |
 |   |   (Bridge: /dev/ttyACM0 | IMU: /dev/ttyUSB0)|             |                         |   |
 |   +----------------------+----------------------+             +------------+------------+   |
 +--------------------------|-------------------------------------------------|----------------+
                            |                                                 | Dual RS-485
              +-------------+-------------+                                   |
              |                           |                                   v
              | USB (CDC-ACM micro-ROS)   | USB (FT231XS UART @ 921600 baud) +-------------------------+
              | (Isolated ADuM4160)       | Native micro-ROS Client          | ZLTech Motor Drivers    |
              v                           v                                  | (Left & Right Wheels)   |
 +----------------------------+   +--------------------------------------+   +-------------------------+
 |  STM32_Bridge: TUSB4041I   |   |              STM32_IMU               |
 |  USB HUB 1                 |   |  - Bosch BNO055 (I2C1)               |
 +-------+------------+-------+   |  - Ext 32.768kHz Crystal             |
         | Port 1A    | Port 2/3  |  - Bare-Metal micro-ROS Client       |
         v            v           |  - Direct Publisher: /imu @ 100 Hz   |
  +--------------+ +-----------+  +--------------------------------------+
  | STM32F411CEU | |  FT231XS  | ---> RS485 Isolation ---> ZLTech Motor Drivers (Left/Right)
  | (Bridge MCU) | +-----------+
  |  - FreeRTOS  |
  |  - micro-ROS |
  +------+-------+
         |
         v
  +------+--------------------------------------------+
  | - 2x Ultrasonic Sensors (I2C1)                    |
  | - 2x Single Point LiDAR Sensors (I2C2)            |
  | - LoRa Telemetry / Manual Override (USART1)       |
  | - PCF8574 Output -> 8x Opto -> 8x 24V Relays      |
  | - PCF8574 Input  <- 8x Opto <- Bumpers & Switches |
  | - System LEDs (PC13, PB12)                        |
  +---------------------------------------------------+
```

---

## 2. Hardware Specification & Pin Mapping

### 2.1 STM32_Bridge (Board 3: Microcontroller Bridge)

#### A. Power Architecture
- **Main Power**: 24V Fused Power (`PWR_IN`, `PGND` via `CN1` / `CN2`).
- **Primary DC-DC**: `PW1` (TURB2405YMD-30WR3, 18–36V to isolated 5V, 30W).
- **Core 3.3V Step-down**: `PW2` (TLV62569PDRLR, Buck regulator 5V to +3V3).
- **USB Hub 1.1V Core**: `PW3` (TLV62569PDRLR, Buck regulator 5V to +1V1).
- **Isolated Power for RS-485**:
  - `PW4` (B0505S-1WR3): +5V to `+5V_ISO1` / `ISO1_GND` (Left Motor).
  - `PW5` (B0505S-1WR3): +5V to `+5V_ISO2` / `ISO2_GND` (Right Motor).
  - `PW6` (B0505S-1WR3): +5V to `+5V_ISO3` / `ISO3_GND` (Head Unit / Reserved).

#### B. MCU Pin Allocation (STM32F411CEU6)

| Pin | Pin Name | Signal / Net Name | Function / Peripheral | Target Device / Destination |
|:---:|:---:|:---:|:---:|:---|
| **2** | `PC13` | `LED1` | GPIO Output (Active LOW) | Onboard Status Indicator LED1 |
| **3** | `PC14` | `OSC32_IN` | RCC LSE Crystal In | 32.768 kHz RTC Crystal (`X1`) |
| **4** | `PC15` | `OSC32_OUT` | RCC LSE Crystal Out | 32.768 kHz RTC Crystal (`X1`) |
| **5** | `PH0` | `OSC_IN` | RCC HSE In | 25.000 MHz System Crystal (`X2`) |
| **6** | `PH1` | `OSC_OUT` | RCC HSE Out | 25.000 MHz System Crystal (`X2`) |
| **7** | `NRST` | `NRST` | Hardware Reset | Reset Pushbutton `SW1` |
| **21** | `PB10` | `I2C2_SCL` | I2C2 Clock (AF4) | Single Point LiDAR Sensors (`CN15`, `CN17`), 2.2k pullup |
| **25** | `PB12` | `LED2` | GPIO Output (Active LOW) | Onboard Status Indicator LED2 |
| **29** | `PA8` | `I2C3_SCL` | I2C3 Clock (AF4) | PCF8574 Expanders (Output `IC2` & Input `IC3`), 4.7k pullup |
| **30** | `PA9` | `UART1_TX` | USART1 TX (AF7) | LoRa Transceiver Header (`CN18` Pin 3) |
| **31** | `PA10` | `UART1_RX` | USART1 RX (AF7) | LoRa Transceiver Header (`CN18` Pin 4) |
| **32** | `PA11` | `USBA_DM_DN1` | USB_OTG_FS_DM | TUSB4041I Hub 1 Downstream Port 1 (via 24Ω `R17`) |
| **33** | `PA12` | `USBA_DP_DN1` | USB_OTG_FS_DP | TUSB4041I Hub 1 Downstream Port 1 (via 24Ω `R19`) |
| **34** | `PA13` | `SWDIO` | SWD Debug Data | SWD Header (`CN19` Pin 3) |
| **37** | `PA14` | `SWCLK` | SWD Debug Clock | SWD Header (`CN19` Pin 4) |
| **39** | `PB3` | `I2C2_SDA` | I2C2 Data (AF9) | Single Point LiDAR Sensors (`CN15`, `CN17`), 2.2k pullup |
| **40** | `PB4` | `I2C3_SDA` | I2C3 Data (AF9) | PCF8574 Expanders (Output `IC2` & Input `IC3`), 4.7k pullup |
| **42** | `PB6` | `I2C1_SCL` | I2C1 Clock (AF4) | Ultrasonic Sensors (`CN14`, `CN16`), 2.2k pullup |
| **43** | `PB7` | `I2C1_SDA` | I2C1 Data (AF4) | Ultrasonic Sensors (`CN14`, `CN16`), 2.2k pullup |
| **44** | `BOOT0` | `BOOT0` | Boot Config | BOOT DIP Switch `SW2` |

#### C. I2C3 Expander Mapping (PCF8574T)

##### 1. Digital Output Expander (`IC2` - PCF8574T):
- **Address**: `0x20` (A0=0, A1=0, A2=0) [7-bit `0x20`, Write Byte `0x40`, Read Byte `0x41`].
- **Logic**: Active LOW. Writing bit `0` activates the Optocoupler (`PC817C`), turns ON NPN transistor (`BC848C`), and energizes the 24V relay (`G5V-1-DC24`). Writing bit `1` de-energizes the relay.
- **Channels**:
  - `P0` -> `OUT_P0` -> `OC1` -> `RY1` -> `DOUT1` (Power Relay / System Main)
  - `P1` -> `OUT_P1` -> `OC2` -> `RY2` -> `DOUT2` (Motor Power Enable / Contactor)
  - `P2` -> `OUT_P2` -> `OC3` -> `RY3` -> `DOUT3` (Payload / Auxiliary Power)
  - `P3` -> `OUT_P3` -> `OC4` -> `RY4` -> `DOUT4` (Indicator Lamp / Tower Light Green)
  - `P4` -> `OUT_P4` -> `OC5` -> `RY5` -> `DOUT5` (Indicator Lamp / Tower Light Yellow)
  - `P5` -> `OUT_P5` -> `OC6` -> `RY6` -> `DOUT6` (Indicator Lamp / Tower Light Red)
  - `P6` -> `OUT_P6` -> `OC7` -> `RY7` -> `DOUT7` (Buzzer / Audio Siren)
  - `P7` -> `OUT_P7` -> `OC8` -> `RY8` -> `DOUT8` (Headlight / Reserved Relay)

##### 2. Digital Input Expander (`IC3` - PCF8574T):
- **Address**: `0x21` (A0=1, A1=0, A2=0) [7-bit `0x21`, Write Byte `0x42`, Read Byte `0x43`].
- **Logic**: Active LOW. External signal pulling `DINx` to `EXT_GND` turns ON `PC817C` optoisolator, driving `IN_Px` LOW (`0`). Open/Inactive state is pulled to 3.3V via 2k resistor networks (`RN3`, `RN4`).
- **Channels**:
  - `P0` <- `IN_P0` <- `OC9`  <- `DIN1` (E-Stop / Emergency Stop Button NC/NO)
  - `P1` <- `IN_P1` <- `OC10` <- `DIN2` (Power Push Button / System Start)
  - `P2` <- `IN_P2` <- `OC11` <- `DIN3` (Front Bumper Collision Limit Switch L)
  - `P3` <- `IN_P3` <- `OC12` <- `DIN4` (Front Bumper Collision Limit Switch R)
  - `P4` <- `IN_P4` <- `OC13` <- `DIN5` (Rear Bumper Collision Limit Switch L)
  - `P5` <- `IN_P5` <- `OC14` <- `DIN6` (Rear Bumper Collision Limit Switch R)
  - `P6` <- `IN_P6` <- `OC15` <- `DIN7` (Manual / Auto Mode Selector Switch)
  - `P7` <- `IN_P7` <- `OC16` <- `DIN8` (Auxiliary / Charging Dock Contact Sense)

#### D. Motor & Head Unit Pass-Through Paths (Hardware Independent from MCU)
- Host communicates with ZLTech Motor Drivers and Head Unit via onboard FT231XS chips cascaded under TUSB4041I USB hubs.
- **Left Motor Port**: `CN11` (`A1`, `B1`, `ISO1_GND`) <- `IC7` MAX485 <- `IC9` ADuM1401 <- `IC8` FT231XS (Hub 1 Port 2).
- **Right Motor Port**: `CN12` (`A2`, `B2`, `ISO2_GND`) <- `IC10` MAX485 <- `IC12` ADuM1401 <- `IC11` FT231XS (Hub 1 Port 3).
- **Head Unit RS-485**: `CN13` (`A3`, `B3`, `ISO3_GND`) <- `IC13` MAX485 <- `IC15` ADuM1401 <- `IC14` FT231XS (Hub 2 Port 1).
- *Note for Developers/Agents*: The Bridge MCU does not directly sample the motor RS-485 lines; the Host IPC handles closed-loop motor control directly over virtual serial ports created by the FT231XS converters. However, the MCU controls motor power and safety cutoffs via the `DOUT` relays.

#### E. Distance Sensors Hardware Specifications & Protocols (Verified Datasheet Reference)

##### 1. Single-Point ToF LiDAR: Benewake TFmini-S (I2C2: PB10 SCL, PB3 SDA, Connectors CN15, CN17)
- **Datasheet**: `TFmini-S Datasheet.pdf` (Benewake Beijing Co., Ltd).
- **Physical Operating Parameters**:
  - Operating Range: **0.10 m – 12.0 m** (@ 90% reflectivity), 0.10 m – 7.0 m (@ 10% reflectivity).
  - Blind Zone: **0.10 m (10 cm)**.
  - Accuracy: **±6 cm** (@ 0.1–6 m), **±1%** (@ 6–12 m).
  - Distance Resolution: **1 cm (0.01 m)**.
  - Field of View (FoV): **2°** (cone angle, $0.035\text{ rad}$).
  - Operating Voltage: **5.0V ± 0.1V**, Current: average $\le 140\text{ mA}$, peak $200\text{ mA}$.
  - Logic Level: **3.3V LVTTL**.
- **I2C Protocol & Addressing**:
  - Bus: `I2C2` (`PB10` SCL, `PB3` SDA, 2.2k pull-up to +3.3V).
  - Max transmission rate: Fast mode (**400 kbps**).
  - Default 7-bit Slave Address: **`0x10`** (Write byte `0x20`, Read byte `0x21`).
  - Dual Sensor Configuration (AMR Docking Assist):
    - Both sensors are mounted at the rear of the chassis facing backwards (baseline distance $D$ apart) to measure distance and skew angle $\theta$ relative to the docking station wall/plate during auto-docking.
    - **Rear Left LiDAR (`CN15`)**: Address **`0x10`** (default), Topic: `/sensor/range/lidar_rear_left`, Frame: `lidar_rear_left_link`.
    - **Rear Right LiDAR (`CN17`)**: Address **`0x11`** (reconfigured address), Topic: `/sensor/range/lidar_rear_right`, Frame: `lidar_rear_right_link`.
  - **Data Acquisition Sequence**:
    1. Master transmits 5-byte trigger/query frame to slave: `[0x5A, 0x05, 0x00, 0x01, 0x60]`.
    2. Master reads 9-byte response frame:
       - `Byte 0..1`: Frame header `0x59 0x59`.
       - `Byte 2`: `Dist_L` (Distance LSB in cm).
       - `Byte 3`: `Dist_H` (Distance MSB in cm).
       - `Byte 4..5`: Signal strength (`Strength_L`, `Strength_H`).
       - `Byte 6..7`: Chip temperature in $0.1^\circ\text{C}$.
       - `Byte 8`: Checksum (lower 8 bits of sum of bytes 0..7).
    3. Conversion formula to ROS 2: $\text{range\_m} = (\text{Dist\_H} \ll 8 \mid \text{Dist\_L}) \times 0.01\text{f}$.

##### 2. Ultrasonic Sensor Module: Dianyingpu DYP-A22 (I2C1: PB6 SCL, PB7 SDA, Connectors CN14, CN16)
- **Datasheet**: `A22-Datasheet1.pdf` & `A22-Output-Interfaces.pdf` (Shenzhen Dianyingpu Technology Co., Ltd).
- **Model in Use**: **DYP-A22YYCW-V1.0** (IIC output variant).
- **Physical Operating Parameters**:
  - Operating Range: **0.02 m – 3.50 m (2 cm – 350 cm)**.
  - Blind Zone: **$\le 0.02\text{ m}$ (2 cm)**.
  - Accuracy: **$\pm(1 + S \times 0.3\%)\text{ cm}$** (with onboard temperature compensation).
  - Field of View (Angle): **$30^\circ \sim 60^\circ$** (programmable, default level 4 = $60^\circ$ / $1.047\text{ rad}$).
  - Operating Voltage: **3.3V – 12V** (Typical 5.0V from board +5V rail).
  - Operating Current: Average $\le 10\text{ mA}$, Peak $150\text{ mA}$, Standby $\le 5\ \mu\text{A}$.
- **I2C Protocol & Addressing**:
  - Bus: `I2C1` (`PB6` SCL, `PB7` SDA, 2.2k pull-up to +3.3V).
  - Clock Speed: Standard mode (**100 kbps**, 10–100 kbit/s supported).
  - Default 8-bit Write Address: **`0xE8`** $\longrightarrow$ **7-bit I2C Address is `0x74`** (`0xE8 >> 1`).
  - Default 8-bit Read Address: **`0xE9`**.
  - Dual Sensor Configuration:
    - **Sonar Left (`CN14`)**: Address **`0x74`** (8-bit `0xE8`, default).
    - **Sonar Right (`CN16`)**: Address **`0x75`** (8-bit `0xEA`) or **`0x70`** (8-bit `0xE0`).
  - **Register Map & Command Flow**:
    - Register `0x10` (Write-only): Trigger ranging command:
      - Write `0xB4`: Trigger range level 4 (max 350 cm, mm unit output, response time 35–110 ms).
      - Write `0xB8`: Trigger range level 3 (max 250 cm, mm unit output, response time 25–100 ms).
    - Register `0x02..0x03` (Read-only): Real-time distance output (16-bit unsigned, Big-Endian):
      - `Byte 0 (Reg 0x02)`: Distance MSB in mm.
      - `Byte 1 (Reg 0x03)`: Distance LSB in mm.
      - *Note*: If read before measurement completes, register returns `0xFFFF`.
    - Register `0x05` (Read/Write): Slave address modification (persists across power cycles).
    - Register `0x07` (Read/Write): Detection angle level ($1=30^\circ, 2=40^\circ, 3=50^\circ, 4=60^\circ$).
  - **Reading Routine**:
    1. Write `0xB4` to register `0x10` on target sensor address.
    2. Non-blocking delay in FreeRTOS `vUltrasonicTask` (~50 ms).
    3. Read 2 bytes from register `0x02`.
    4. Conversion formula to ROS 2: $\text{range\_m} = ((\text{reg\_0x02} \ll 8) \mid \text{reg\_0x03}) \times 0.001\text{f}$.

---

### 2.2 STM32_IMU (Board 4: IMU Module)

#### A. Power Architecture
- Power source: USB VBUS (+5V) from USB Type-C `U4`.
- Fuse & Surge: PPTC `F1` (1206L025/30NR) + TVS diodes `D4`, `D5` (P6SMB6.8CA).
- Voltage Regulator: `U5` (AMS1117-3.3, 5V to 3.3V).
- Power LED: `LED1` (Yellow/Green).

#### B. MCU Pin Allocation (STM32F103C8T6)

| Pin | Pin Name | Signal / Net Name | Function / Peripheral | Target Device / Destination |
|:---:|:---:|:---:|:---:|:---|
| **2** | `PC13` | `PC13` | GPIO Output (Active LOW) | Status Indicator LED2 (`XL-1608SURC-06`) |
| **5** | `PD0` | `OSC_IN` | RCC HSE In | 8.000 MHz System Crystal (`X2`) |
| **6** | `PD1` | `OSC_OUT` | RCC HSE Out | 8.000 MHz System Crystal (`X2`) |
| **7** | `NRST` | `NRST` | Hardware Reset | Pushbutton `SW1` |
| **30** | `PA9` | `U0TXD` | USART1 TX (AF Push-Pull) | FT231XS `RXD` (Pin 4) |
| **31** | `PA10` | `U0RXD` | USART1 RX (Input Floating) | FT231XS `TXD` (Pin 20) |
| **34** | `PA13` | `SWDIO` | SWD Debug Data | SWD Header (`HDR-M_2.54_1x4P` Pin 3) |
| **37** | `PA14` | `SWCLK` | SWD Debug Clock | SWD Header (`HDR-M_2.54_1x4P` Pin 4) |
| **42** | `PB6` | `SCL` | I2C1 SCL (Open-Drain, AF) | BNO055 `SCL1` (Pin 19), 4.7k pullup (`R12`) |
| **43** | `PB7` | `SDA` | I2C1 SDA (Open-Drain, AF) | BNO055 `SDA1` (Pin 20), 4.7k pullup (`R11`) |
| **44** | `BOOT0` | `BOOT0` | Boot Option | Slide Switch `SW2` |

#### C. BNO055 Configuration
- **I2C Address**: `0x28` (`COM3` Pin 17 tied to GND).
- **Communication Protocol**: Standard I2C (`PS0` Pin 6 = GND, `PS1` Pin 5 = GND).
- **Oscillator**: High-precision external 32.768 kHz quartz crystal (`X3`) on pins `XIN32` / `XOUT32` for superior gyro integration and low thermal drift.
- **Operating Modes to support**:
  - `NDOF` (9 Degrees of Freedom Fusion mode: Accel + Gyro + Magnetometer, absolute yaw referenced to magnetic north).
  - `IMU` (Inertial Measurement Unit mode: Accel + Gyro fusion, relative yaw, immune to magnetic interference from large metal structures or AMR batteries/motors).
- **Interrupt / Reset Lines**:
  - `RESET_BNO` (Pin 11): 4.7k pullup to 3.3V.
  - `INT_BNO` (Pin 14): 10k pulldown to GND.

---

## 3. Communication Architecture & Packet Protocols

### 3.1 Host <-> STM32_Bridge Protocol (USB CDC-ACM)

The Bridge MCU communicates with the Host IPC over USB Full Speed CDC Virtual COM Port.

#### Recommended Protocol: Binary Packet Framing (COBS / CRC16) or JSON-RPC Lightweight
For deterministic latency and high telemetry throughput (50–100 Hz), use **Packet-based Framing**:
- **Frame Header**: `0xAA 0x55`
- **Message ID**: 1 byte
- **Payload Length**: 1 byte
- **Payload**: N bytes
- **Checksum**: CRC-16-CCITT (2 bytes)

#### Message Definition Matrix:

| Msg ID | Direction | Name | Payload Description |
|:---:|:---:|:---|:---|
| `0x01` | Host -> MCU | `CMD_HEARTBEAT` | Sequence counter, Keep-alive timeout (ms) |
| `0x02` | MCU -> Host | `ACK_HEARTBEAT` | MCU Uptime (ms), System status flags |
| `0x10` | Host -> MCU | `CMD_SET_RELAYS` | Bitmask (8 bits) for DOUT1–DOUT8 states |
| `0x11` | Host -> MCU | `CMD_PULSE_RELAY`| Relay index (0–7), Duration (ms) |
| `0x20` | MCU -> Host | `TELEMETRY_FAST` | Collision Bumper states (8 bits), E-stop flag, Button states (100 Hz) |
| `0x21` | MCU -> Host | `TELEMETRY_SENSORS`| 2x Ultrasonic distance (mm), 2x LiDAR distance (mm) (20–50 Hz) |
| `0x30` | Both | `LORA_TUNNEL` | Transparent payload for LoRa packet transmission / reception |
| `0xEE` | MCU -> Host | `ALARM_EMERGENCY`| Immediate event packet triggered on collision bumper hit or E-stop |

#### Fail-Safe Behavior (Safety Watchdog):
1. **Communication Watchdog**: If the Bridge MCU fails to receive a valid heartbeat from the Host within a configurable timeout (default: `500 ms`):
   - Immediately cut motor power relay (`DOUT2`).
   - Trigger yellow/red warning beacons (`DOUT5`/`DOUT6`).
   - Sound warning buzzer (`DOUT7`).
2. **Bumper Collision Event**: If any bumper limit switch (`DIN3`–`DIN6`) triggers:
   - Hardware/Firmware immediate interrupt response cuts motor drive relay without waiting for Host confirmation.
   - Dispatches emergency alarm packet `ALARM_EMERGENCY` to Host.

---

### 3.2 Host <-> STM32_IMU micro-ROS Interface (XRCE-DDS over FT231XS UART @ 921600 baud)

The `STM32_IMU` board runs a lightweight bare-metal **micro-ROS Client (XRCE-DDS)** communicating with `micro-ros-agent` on the Host IPC via its dedicated USB Type-C port (FT231XS-R USB-to-UART converter on `USART1` @ **921600 baud**).

#### A. Architecture & Transport
- **Client Stack**: eProsima Micro XRCE-DDS Client + `rclc` running bare-metal (super-loop architecture optimized for 64 KB Flash / 20 KB SRAM).
- **Physical Link**: `USART1` (`PA9` TX / `PA10` RX) connected to FT231XS at **921600 baud** (8N1). High baud rate is mandatory to stream serialized CDR `sensor_msgs/msg/Imu` messages (~300 bytes) at **100 Hz** without serial buffer congestion.
- **Clock Synchronization**: Periodically invokes `rmw_uros_sync_session()` to synchronize MCU timestamp directly with Host ROS 2 clock (`header.stamp` in microsecond resolution).

#### B. Published Topic (STM32_IMU -> ROS 2 Computation Graph)
| Topic Name | Message Type | Rate | Description |
|:---|:---|:---:|:---|
| `/imu` | `sensor_msgs/msg/Imu` | 100 Hz | Precomputed clean IMU data (Quaternion, Gyro rad/s, Linear Accel m/s², Covariance) published directly for EKF |

#### C. Onboard Precomputing & Scaling Transformations (Raw Sensor to SI Units)
The `STM32_IMU` firmware performs all mathematical unit conversions onboard before serializing to micro-ROS, ensuring the Host IPC receives clean data:
- **Orientation (Quaternion $w, x, y, z$)**:
  $$q = \frac{\text{raw}}{16384.0f} \quad (\text{normalized range } [-1.0, 1.0])$$
- **Angular Velocity / Gyroscope ($\omega_x, \omega_y, \omega_z$)**:
  $$\omega = \text{raw} \times \frac{1}{16.0f} \times \frac{\pi}{180.0f} \quad (\text{converted from deg/s to rad/s})$$
- **Linear Acceleration ($a_x, a_y, a_z$)**:
  $$a = \frac{\text{raw}}{100.0f} \quad (\text{converted to } \text{m/s}^2)$$
  *(Note: BNO055 internal sensor fusion already removes the $9.81\text{ m/s}^2$ Earth gravity vector from register `LIA`).*
- **Header Metadata**:
  - `header.frame_id = "imu_link"`
  - `header.stamp = rmw_uros_epoch_nanos()`

#### D. Measurement Covariance Matrix Assignment
`STM32_IMU` populates the diagonal elements of the $3 \times 3$ covariance matrices in `sensor_msgs/msg/Imu` for EKF weighting:
- `angular_velocity_covariance`: `[1e-4, 0, 0,  0, 1e-4, 0,  0, 0, 1e-4]` (High precision gyro with external 32.768 kHz crystal).
- `orientation_covariance`: `[1e-3, 0, 0,  0, 1e-3, 0,  0, 0, 1e-3]` (Can be dynamically adjusted based on `calib_stat`).
- `linear_acceleration_covariance`: `[1e-2, 0, 0,  0, 1e-2, 0,  0, 0, 1e-2]` (Accommodates chassis/wheel mechanical vibration).

#### E. Alignment with ROS 2 EKF Configuration (`ekf_cfg.yaml`)
- **Gravity Flag**: Must set `imu0_remove_gravitational_acceleration: false` since linear acceleration has already subtracted gravity.
- **Indoor Heading Stability**: Recommended to fuse gyro yaw rate (`vyaw = true`, `yaw = false`) to eliminate vulnerability to steel structure magnetic distortion:
  ```yaml
  imu0_config: [false, false, false,   # x, y, z
                false, false, false,   # roll, pitch, yaw
                false, false, false,   # vx, vy, vz
                false, false, true,    # vroll, vpitch, vyaw (fused gyro rate)
                false, false, false]   # ax, ay, az
  ```
- **Clock Mode**: On real hardware, `use_sim_time: false` is required so EKF uses system wall-clock time.
- **Covariance Dimensions**: `initial_estimate_covariance` must strictly contain 225 elements ($15 \times 15$).

#### F. Diagnostic / Fallback Binary Protocol
For standalone hardware testing, calibration visualization, and desktop bringup without ROS 2, the 36-byte packed binary protocol (`IMU_PKT_SYNC` `0x55 0xAA` with CRC16-CCITT) remains supported by the desktop tools:
- [tools/web_imu_visualizer.html](file:///d:/Firmware_AMR_Control/STM32_IMU/tools/web_imu_visualizer.html) (Web Serial 3D visualizer).
- [tools/imu_receiver.py](file:///d:/Firmware_AMR_Control/STM32_IMU/tools/imu_receiver.py) (Terminal CLI data monitor).

---

### 3.3 Host <-> STM32_Bridge micro-ROS Interface (XRCE-DDS over USB CDC)

When operating in ROS 2 mode, the `STM32_Bridge` runs a native micro-ROS Client communicating with `micro-ros-agent` on the Host IPC via USB CDC-ACM.

#### A. Published Topics (MCU -> ROS 2 Computation Graph)
| Topic Name | Message Type | Rate | Description |
|:---|:---|:---:|:---|
| `/sensor/range/lidar_rear_left` | `sensor_msgs/msg/Range` | 50 Hz | Rear Left ToF LiDAR distance (m) for AMR docking via I2C2 |
| `/sensor/range/lidar_rear_right`| `sensor_msgs/msg/Range` | 50 Hz | Rear Right ToF LiDAR distance (m) for AMR docking via I2C2 |
| `/sensor/range/sonar_left` | `sensor_msgs/msg/Range` | 20 Hz | Left ultrasonic range measurement (m) via I2C1 |
| `/sensor/range/sonar_right` | `sensor_msgs/msg/Range` | 20 Hz | Right ultrasonic range measurement (m) via I2C1 |
| `/amr/safety/status` | `std_msgs/msg/UInt8` | 100 Hz | 8-bit mask of digital inputs (E-Stop, 4x Bumpers, Power buttons) |
| `/amr/safety/emergency` | `std_msgs/msg/Bool` | Event | Instant event publication on bumper collision or E-stop trigger |
| `/amr/system/heartbeat` | `std_msgs/msg/UInt32` | 2 Hz | MCU uptime in milliseconds |

#### B. Subscribed Topics (ROS 2 Host -> MCU)
| Topic Name | Message Type | Description |
|:---|:---|:---|
| `/amr/cmd/relays` | `std_msgs/msg/UInt8` | Direct bitmask to actuate DOUT1..DOUT8 (Power, Aux, Lights) |
| `/amr/cmd/tower_light` | `std_msgs/msg/UInt8` | High-level status lamp mode (0=Off, 1=Green, 2=Yellow, 3=Red) |
| `/amr/cmd/buzzer` | `std_msgs/msg/UInt16` | Audio alert pulse duration in milliseconds |

#### C. Advertised Services
| Service Name | Service Type | Description |
|:---|:---|:---|
| `/amr/safety/reset` | `std_srvs/srv/Trigger` | Unlock motor relay after collision recovery and bumper clearance |

---

## 4. Firmware Architecture & Directory Structure

Both firmware projects should follow clean, modular embedded architecture separating Hardware Abstraction (HAL), Drivers, Middleware, and Application Logic.

```
Firmware_AMR_Control/
├── agents.md                                <-- System blueprint & operational instructions
├── STM32_Bridge/
│   ├── Diagram Blok System-Microcontroller Bridge PCB ver-0.5.png
│   ├── SCH_MCU Bridge Schematic.pdf
│   ├── CMakeLists.txt                       <-- CMake build script
│   ├── arm-none-eabi-toolchain.cmake        <-- Cross-compilation toolchain
│   ├── STM32F411CEUx_FLASH.ld               <-- Flash 512K, RAM 128K linker script
│   ├── Core/
│   │   ├── Inc/
│   │   │   ├── main.h
│   │   │   ├── stm32f4xx_it.h
│   │   │   ├── stm32f4xx_hal_conf.h
│   │   │   └── FreeRTOSConfig.h             <-- FreeRTOS kernel configuration
│   │   └── Src/
│   │       ├── main.c
│   │       ├── stm32f4xx_it.c
│   │       ├── system_stm32f4xx.c
│   │       └── syscalls.c
│   ├── Drivers/
│   │   ├── BSP/
│   │   │   ├── pcf8574.h / pcf8574.c        <-- Dual expander driver (Output & Input)
│   │   │   ├── ultrasonic_sensor.h / .c     <-- I2C1 Ultrasonic reading (2x Sonar)
│   │   │   ├── tfmini_lidar.h / .c          <-- I2C2 Single Point LiDAR (2x TFmini)
│   │   │   ├── lora_module.h / .c           <-- UART1 LoRa AT/packet handler
│   │   │   └── onboard_leds.h / .c          <-- Status LED control (PC13, PB12)
│   │   ├── STM32F4xx_HAL_Driver/
│   │   └── CMSIS/
│   ├── Middlewares/
│   │   ├── Third_Party/FreeRTOS/            <-- FreeRTOS Kernel (Cortex-M4F port)
│   │   ├── ST/STM32_USB_Device_Library/     <-- USB CDC Virtual COM Port stack
│   │   └── micro_ros_stm32/                 <-- micro-ROS Client & custom USB CDC transport
│   ├── App/
│   │   ├── rtos_tasks.h / .c                <-- FreeRTOS task definitions & scheduler init
│   │   ├── rtos_queues.h / .c               <-- Thread-safe Inter-Task Queues
│   │   ├── safety_monitor.h / .c            <-- Bumper cutoff (<10ms), E-stop, watchdog logic
│   │   ├── microros_interface.h / .c        <-- micro-ROS Node, Publishers & Subscribers
│   │   └── io_controller.h / .c             <-- Relay & tower light animation management
│   └── STM32_Bridge.ioc                     <-- STM32CubeMX project file
│
└── STM32_IMU/
    ├── SCH_IMU Schematic_2026-09-04.pdf
    ├── CMakeLists.txt                       <-- CMake build script (Clean: 0 error, 0 warning)
    ├── arm-none-eabi-toolchain.cmake        <-- Cross-compilation toolchain
    ├── STM32F103C8Tx_FLASH.ld               <-- Flash 64K, RAM 20K linker script
    ├── Core/
    │   ├── Inc/
    │   │   ├── main.h
    │   │   ├── stm32f1xx_it.h
    │   │   └── stm32f1xx_hal_conf.h
    │   └── Src/
    │       ├── main.c
    │       ├── stm32f1xx_it.c
    │       ├── system_stm32f1xx.c
    │       ├── syscalls.c
    │       └── startup_stm32f103xb.s
    ├── Drivers/
    │   ├── BSP/
    │   │   ├── bno055.h / bno055.c          <-- Bosch BNO055 driver (32.768kHz xtal, 40B burst, 22B calib)
    │   │   └── onboard_leds.h / .c          <-- Status LED control (PC13, Active LOW)
    │   ├── STM32F1xx_HAL_Driver/
    │   └── CMSIS/
    ├── Middlewares/
    │   └── micro_ros_stm32/                 <-- Bare-metal micro-ROS Client & UART custom transport (921600 baud)
    ├── App/
    │   ├── imu_app.h / .c                   <-- 100 Hz supervisor state machine
    │   ├── microros_imu.h / .c              <-- micro-ROS Publisher node (/imu sensor_msgs/msg/Imu)
    │   └── serial_protocol.h / .c           <-- Diagnostic binary frame fallback + CRC16-CCITT
    └── tools/
        ├── web_imu_visualizer.html          <-- 3D AMR Web Visualizer, Virtual Sim & Web Serial
        ├── imu_receiver.py                  <-- Python live telemetry receiver
        └── mock_imu_sender.py               <-- Virtual telemetry stream generator
```

---

## 5. Development Guidelines & Rules for AI Agents

When implementing, modifying, or testing code in this workspace, all AI agents and developers must strictly adhere to the following rules:

### 5.1 Safety-Critical Design Rules
1. **Relay Failsafe State**: All relays (`DOUT1`..`DOUT8`) must default to `OFF` (de-energized) on system startup, brownout, or MCU reset. Because PCF8574 outputs default high (weak pull-up) on power-up and the circuit is Active Low, the pins will naturally be high (relays off). The firmware initialization must explicitly write `0xFF` to `IC2` before enabling normal operation.
2. **Bumper Response Latency**: The collision bumper polling and debounce logic must run with maximum latency $\le 10\ \text{ms}$. If any collision switch opens/closes, the motor enable relay must be de-asserted immediately.
3. **I2C Bus Lockup Recovery**: I2C peripherals (especially on STM32F1 and STM32F4) can lock the bus if a slave is interrupted during a clock pulse. Implement an **I2C Bus Clear / Clock Toggling sequence** (clocking SCL 9 times via GPIO before initializing the I2C peripheral) in the hardware startup code.
4. **Non-blocking I/O**: Never use blocking delays (`HAL_Delay`) inside communication or safety loops. Use hardware timers, tick counters (`HAL_GetTick()`), or FreeRTOS task delays.

### 5.2 Sensor Integration Specifics
1. **BNO055 Clock Configuration**: The BNO055 board features an external 32.768 kHz crystal (`X3`). The firmware **must** configure the BNO055 to use the external crystal (`CLK_SEL` bit in `SYS_TRIGGER` register set to `1`) during initialization. This reduces drift by up to 5x compared to the internal RC oscillator.
2. **LiDAR & Ultrasonic Bus Isolation**:
   - `I2C1` (`PB6`/`PB7`) is strictly dedicated to Ultrasonic sensors.
   - `I2C2` (`PB10`/`PB3`) is strictly dedicated to Single Point LiDAR sensors.
   - `I2C3` (`PA8`/`PB4`) is strictly dedicated to the PCF8574 I/O Expanders.
   - Do **not** mix these busses; keeping them physically separate prevents sensor errors or addressing conflicts on one bus from impacting the others.

### 5.3 Code Style & Embedded Conventions
- **Standard**: C11, compile with `-Wall -Wextra -Werror -pedantic`.
- **Naming Conventions**:
   - Functions: `Module_ActionName()` (e.g., `PCF8574_WriteRelays()`, `BNO055_ReadQuaternion()`).
   - Constants & Macros: `ALL_CAPS_WITH_UNDERSCORES` (e.g., `BRIDGE_WATCHDOG_TIMEOUT_MS`).
   - Types/Structs: `PascalCase_t` or `TypeName_t` (e.g., `ImuDataPacket_t`).
   - Strict fixed-width integers: Use `stdint.h` (`uint8_t`, `int16_t`, `uint32_t`, etc.), never raw `int` or `long`.

### 5.4 FreeRTOS & micro-ROS Specific Guidelines
1. **Preemptive Task Priorities (STM32_Bridge)**:
   - Priority 4 (Highest): `vSafetyTask` (100 Hz / latency $\le 10\text{ ms}$).
   - Priority 3: `vMicroRosTask` (XRCE-DDS Spin Executor & Host USB communication).
   - Priority 2: `vLidarTask` (50 Hz I2C2 ToF sampling).
   - Priority 1: `vUltrasonicTask` (20 Hz I2C1 sonar ranging, non-blocking delay).
   - Priority 1: `vIoAnimationTask` (10 Hz tower lamp & buzzer rhythm).
2. **Thread-Safe Queue Interfacing**:
   - Tasks must never share raw memory without mutexes or queues. Sensor data must be passed to micro-ROS publishers via `xTelemetryQueue`.
   - Emergency alarms must use `xQueueSendToFront()` to bypass pending messages.
3. **micro-ROS Reconnection Engine**:
   - The micro-ROS transport must detect agent disconnection (USB replug or host reboot) and automatically enter a reconnection state machine without hanging the safety task.
4. **STM32_IMU micro-ROS Architecture**:
   - Runs bare-metal without FreeRTOS to respect the 64 KB Flash / 20 KB RAM envelope of the STM32F103C8T6.
   - Uses `rclc_executor` in non-blocking mode inside `main()` super-loop.

---

## 6. Implementation Roadmap & Milestones

### Phase 1: Foundation & Project Scaffolding
- [x] Initialize CMake configuration & toolchains for `STM32_IMU` (STM32F103C8T6).
- [x] Configure clock trees:
  - IMU: HSE 8 MHz -> PLL -> 72 MHz SysClk. [Done]
  - Bridge: HSE 25 MHz -> PLL -> 100 MHz SysClk, 48 MHz USB clock. [Pending]
- [x] Implement GPIO and LED status driver for `STM32_IMU` (PC13).
- [ ] Initialize CMake configuration for `STM32_Bridge` (STM32F411CEU6).

### Phase 2: Driver Development
- [x] **IMU**: BNO055 register driver with external 32.768 kHz crystal initialization.
- [x] **IMU**: 40-byte atomic burst reading (`0x0E` to `0x35`) & 22-byte calibration profile persistence.
- [x] **IMU**: Hardware axis remapping & sign inversion (REP-103 alignment).
- [x] **Bridge**: PCF8574 dual-bus driver supporting dual addresses (0x20 Output Relay, 0x21 Input Bumper on I2C3 PCB, and 0x24 Dual-Bus I2C1/I2C2 for prototype).
- [ ] **Bridge**: Relay control abstraction layer with active-low inversion and safe state enforcement (`0xFF` on boot).
- [ ] **Bridge**: Opto-isolated digital input debouncing engine (100 Hz sampling).
- [ ] **Bridge**: I2C2 driver for 2x Single Point LiDAR sensors (TFmini-Plus).
- [ ] **Bridge**: I2C1 driver for 2x Ultrasonic distance sensors.
- [ ] **Bridge**: LoRa module UART communication handler (USART1).

### Phase 3: Communication & RTOS Middleware
- [x] **IMU**: High-speed binary telemetry protocol (36 bytes, 100 Hz) with CRC16-CCITT for standalone tools.
- [x] **IMU**: Web 3D Visualizer & simulator with Web Serial API (`web_imu_visualizer.html`).
- [ ] **IMU**: Bare-metal micro-ROS Client integration over USART1 @ 921600 baud.
- [ ] **IMU**: Direct micro-ROS publisher for `/imu` (`sensor_msgs/msg/Imu` @ 100 Hz).
- [ ] **Bridge**: FreeRTOS Preemptive Kernel integration on Cortex-M4F.
- [ ] **Bridge**: Inter-Task Queues (`xHostCmdQueue`, `xTelemetryQueue`, `xRelayCmdQueue`).
- [ ] **Bridge**: Safety Watchdog timer (500 ms) and immediate bumper cutoff.
- [ ] **Bridge**: micro-ROS (XRCE-DDS) Client integration over USB CDC-ACM.

### Phase 4: System Integration & ROS 2 Host Interfacing
- [ ] Implement micro-ROS publishers on Bridge (`/sensor/range/...`, `/amr/safety/status`) and subscribers (`/amr/cmd/relays`).
- [ ] Create ROS 2 launch file (`amr_bringup.launch.py`) for dual-client `micro-ros-agent` (Bridge on `/dev/ttyACM0`, IMU on `/dev/ttyUSB0`), EKF node, and motor controllers.
- [ ] End-to-end integration test with ZLTech motor drivers, bumper collision cutoff, and Nav2 costmap testing.
