# STM32 Sensor Test & Hardware Diagnostics

Firmware penguji mandiri (standalone bare-metal) untuk memverifikasi dan menampilkan pembacaan sensor **Benewake TFmini-S (LiDAR ToF)** dan **Dianyingpu DYP-A22 (Ultrasonic)** langsung ke terminal serial (USB Virtual COM Port `/dev/ttyACM0`) tanpa memerlukan ROS 2 atau micro-ROS agent.

---

## 1. Skema Pengkabelan (Wiring Pinout)

Target Board: **STM32F401CCU6 (BlackPill)**

### A. Dianyingpu DYP-A22 Ultrasonic (Bus I2C1)
Sensor ultrasonik tahan air untuk deteksi rintangan depan/samping:

| Pin Sensor DYP-A22 | Pin BlackPill STM32 | Keterangan |
| :--- | :--- | :--- |
| **VCC** (Pin 1 - Merah) | **5V** atau **3.3V** | Tegangan suplai operasional |
| **GND** (Pin 2 - Hitam) | **GND** | Ground bersama |
| **SCL / TX** (Pin 3 - Kuning) | **PB6** | I2C1 SCL (Pull-up 2.2kΩ–4.7kΩ ke 3.3V) |
| **SDA / RX** (Pin 4 - Putih) | **PB7** | I2C1 SDA (Pull-up 2.2kΩ–4.7kΩ ke 3.3V) |

- **Alamat I2C Default (7-bit):**
  - Sensor Kiri: `0x74` (8-bit Write: `0xE8`)
  - Sensor Kanan: `0x75` (8-bit Write: `0xEA`)

---

### B. Benewake TFmini-S ToF LiDAR (Bus I2C2)
Sensor ToF LiDAR presisi tinggi untuk docking assist dan deteksi jarak dekat:

| Pin Sensor TFmini-S | Pin BlackPill STM32 | Keterangan |
| :--- | :--- | :--- |
| **VCC** (Kabel Merah) | **5V** | **Wajib 5.0V** (Arus puncak ~140mA) |
| **GND** (Kabel Hitam) | **GND** | Ground bersama |
| **SCL / TX** (Kabel Hijau) | **PB10** | I2C2 SCL (Pull-up 2.2kΩ–4.7kΩ ke 3.3V) |
| **SDA / RX** (Kabel Putih) | **PB3** | I2C2 SDA (AF9, Pull-up 2.2kΩ–4.7kΩ ke 3.3V) |

- **Alamat I2C Default (7-bit):**
  - Sensor Kiri: `0x10` (8-bit Write: `0x20`)
  - Sensor Kanan: `0x11` (8-bit Write: `0x22`)

> **CATATAN RESISTOR PULL-UP:**
> Jalur I2C adalah tipe open-drain. Jika modul sensor Anda tidak memiliki resistor pull-up bawaan pada modulnya, pasang resistor 2.2 kΩ hingga 4.7 kΩ dari masing-masing pin SCL dan SDA ke pin **3.3V** STM32.

---

## 2. Cara Mengompilasi (Build)

Jika Anda melakukan perubahan pada kode:
```bash
export PATH="/home/rifai/.local/bin:$PATH"
cmake -B STM32_Sensor_Test/build -S STM32_Sensor_Test -DCMAKE_TOOLCHAIN_FILE=arm-none-eabi-toolchain.cmake -G Ninja
cmake --build STM32_Sensor_Test/build
```

---

## 3. Cara Flashing ke STM32 (ST-Link V2)

Pastikan ST-Link V2 terhubung ke pin SWD (SWDIO, SWCLK, GND, 3.3V):
```bash
./tools/flash_sensor_test.sh
```

---

## 4. Cara Menjalankan & Membaca Output Sensor

Setelah flashing selesai, STM32 akan mereset dan terdeteksi sebagai port USB CDC `/dev/ttyACM0`.

### Menggunakan Monitor Python Bawaan:
```bash
./STM32_Sensor_Test/run_monitor.sh
```

### Atau Menggunakan Perintah Terminal Langsung:
```bash
stty -F /dev/ttyACM0 115200 raw -echo
cat /dev/ttyACM0
```

---

## 5. Contoh Tampilan Output Diagnostik

```text
[#00125 |  25420 ms]
  [A22 Ultrasonic - I2C1]
    Left  (0x74):  45.30 cm ( 453 mm)  [OK]
    Right (0x75):  82.10 cm ( 821 mm)  [OK]
  [TFmini-S LiDAR - I2C2]
    Left  (0x10):  34.00 cm (Strength:  1820, Temp: 28.5 C)  [OK]
    Right (0x11): 112.00 cm (Strength:  1540, Temp: 28.3 C)  [OK]
```

Jika sensor belum dicolokkan atau pin kabel salah, sistem akan langsung memberi tahu:
- `NO_RESPONSE / NACK`: Sensor tidak merespons alamat I2C (cek kabel VCC, GND, SCL, SDA, atau resistor pull-up).
- `OUT_OF_RANGE`: Target terlalu dekat (<2 cm) atau terlalu jauh (>350 cm).
- `LOW_SIGNAL`: Sinyal pantulan inframerah LiDAR lemah (<100).
