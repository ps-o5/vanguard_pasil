# 🚁 VANGUARD-PASIL
### A Safety-Critical, Distributed Flight Control Testbed

**Document Revision:** E (Production-Ready)

---

## 📌 Overview

VANGUARD-PASIL is a custom-built, RTOS-driven avionics platform designed to validate deterministic flight control, real-time sensor fusion, and hardware-level fault isolation in electrically noisy environments such as multirotor UAV systems.

The system evolved from a bare-metal scheduler into a fully distributed architecture built on Apache NuttX, enabling strict real-time guarantees and modular subsystem validation.

---

## 🧩 System Architecture

![Architecture Diagram](docs/img/0001.png)
![Architecture Diagram](docs/img/0002.png)

**Core Components:**

- **ESP32 Ground Station**
  - User interface and telemetry control
- **NRF24L01 RF Link**
  - Bidirectional low-latency communication
- **STM32F411 Flight Controller**
  - Real-time control loop execution (NuttX RTOS)

**Interfaces:**
- SPI → IMU / high-speed sensors  
- I2C → Environmental sensors (BMP280, etc.)  
- PWM → ESCs / motor control  

**Power Design:**
- Star-ground topology to isolate noise domains  
- Dedicated ultra-low-noise 3.3V LDO for RF subsystem  
- Separation of digital and high-current return paths  

---

## 🎯 Architectural Highlights

### ⏱️ Hard Real-Time Execution
- Deterministic control loop at **500 Hz** (Δt = 0.002 s)  
- NuttX scheduler configured at **Priority 255**  
- Zero tolerance for blocking operations in control path  

---

### 🧠 Advanced State Estimation
- Custom **7×7 Extended Kalman Filter (EKF)**  
- Implemented using ARM CMSIS-DSP library  
- Hardware FPU acceleration on STM32F411  

**Capabilities:**
- Attitude estimation (quaternion-based)  
- Dynamic gyroscope bias tracking  
- Stable operation under high-frequency vibration  

---

### 🔌 Hardware-Level Fault Isolation
- Electrical noise mitigation from ESC flyback  
- Dedicated LDO rail for RF communication  
- Ground loop prevention via star grounding  

---

### 🔄 Asynchronous I/O Architecture
- Kernel-level blocking drivers bypassed  
- Application-level sensor polling implemented  

**Example:**
- BMP280 operated in forced mode via non-blocking I2C  
- Guarantees uninterrupted control loop execution  

---

## 📊 Performance Validation

![Control Loop Jitter](docs/img/0003.png)

- **Control Loop Jitter:** < 10 µs variance  
- **EKF Stability:** Maintains converged quaternion under high-RPM vibration  
- **System Behavior:** No observable control degradation under induced electrical noise  

The system architecture is aggressively optimized for embedded constraints. By bypassing native kernel bloat and utilizing hardware DSP, the RTOS and flight control stack leave >90% of SRAM available for future safety-monitor and payload integration.

| Memory Region | Used Size | Total Capacity | Utilization |
| :--- | :--- | :--- | :--- |
| **Flash (ROM)** | 122.2 KB | 512 KB | 23.31% |
| **SRAM (RAM)** | 9.9 KB | 128 KB | 7.59% |
---

## 🛡️ Safety & Failsafe State Machine

### 📡 Signal Loss Mitigation
- NRF24 packet timeout detection: **< 100 ms**  
- Immediate transition to:
  - Auto-level stabilization  
  - Zero-throttle command  

---

### ⚠️ Actuator Kill-Switch
- Trigger condition: sustained telemetry loss  
- Timeout: **2.0 seconds**  
- Action:
  - Deterministic PWM shutdown  
  - Motor output hard lock  

---

### 🔁 State Transitions
| State            | Condition                      | Action                    |
|------------------|------------------------------|---------------------------|
| NORMAL           | Valid telemetry               | Full control              |
| DEGRADED         | Packet loss detected          | Stabilize + throttle cut  |
| FAILSAFE         | >2.0 s signal loss            | PWM termination           |

---

## ⚙️ Build & Deployment

### 🔧 1. Toolchain Setup

Requirements:
- `arm-none-eabi-gcc`
- Apache NuttX build system  

Verify installation:

```bash
arm-none-eabi-gcc --versioni
```

### 🛠️ 2. Configuration & Compilation

Ensure:

Floating-point support enabled:

CONFIG_LIBC_FLOATINGPOINT=y
Disable kernel I2C drivers (except required IMU drivers)

Build:
```bash
make distclean
./tools/configure.sh -l ../custom_boards/blackpill-f411/configs/nsh
make -j4
```


### ⚡ 3. Flashing (STM32F411)

Using st-flash:
```bash
st-flash write nuttx.bin 0x8000000
```

Or via OpenOCD / STM32CubeProgrammer as preferred.

## 🚀 Quick Start
```bash
git clone https://github.com/<your-repo>/vanguard-pasil.git
cd vanguard-pasil/nuttx/

make distclean
./tools/configure.sh -l ../custom_boards/blackpill-f411/configs
make -j4

st-flash write nuttx.bin 0x8000000
```

## 📁 Project Structure
```bash
custom_boards/
└── blackpill-f411
    ├── CMakeLists.txt
    ├── configs
    │   ├── mcp2515-extid
    │   │   └── defconfig
    │   └── nsh
    │       └── defconfig
    ├── include
    │   └── board.h
    ├── Kconfig
    ├── scripts
    │   ├── flash.ld
    │   └── Make.defs
    └── src
        ├── arm_mat_add_f32.o
        ├── arm_mat_init_f32.o
        ├── arm_mat_mult_f32.o
        ├── arm_mat_trans_f32.o
        ├── blackpill-f411.h
        ├── CMakeLists.txt
        ├── libboard.a
        ├── Make.defs
        ├── Make.dep
        ├── Makefile
        ├── pasil_ekf.c
        ├── pasil_ekf.h
        ├── pasil_ekf.o
        ├── pasil_imu.c
        ├── pasil_imu.o
        ├── stm32_adc.c
        ├── stm32_ajoystick.c
        ├── stm32_appinit.c
        ├── stm32_appinit.o
        ├── stm32_autoleds.c
        ├── stm32_boot.c
        ├── stm32_boot.o
        ├── stm32_bringup.c
        ├── stm32_bringup.o
        ├── stm32_buttons.c
        ├── stm32_lcd_ssd1306.c
        ├── stm32_mcp2515.c
        ├── stm32_spi.c
        ├── stm32_spi.o
        ├── stm32_userleds.c
        ├── stm32_userleds.o
        ├── vl53l0x_api
        │   ├── vl53l0x_api.c
        │   ├── vl53l0x_api_calibration.c
        │   ├── vl53l0x_api_calibration.h
        │   ├── vl53l0x_api_core.c
        │   ├── vl53l0x_api_core.h
        │   ├── vl53l0x_api.h
        │   ├── vl53l0x_api_ranging.c
        │   ├── vl53l0x_api_ranging.h
        │   ├── vl53l0x_api_strings.c
        │   ├── vl53l0x_api_strings.h
        │   ├── vl53l0x_def.h
        │   ├── vl53l0x_device.h
        │   ├── vl53l0x_i2c_platform.h
        │   ├── vl53l0x_interrupt_threshold_settings.h
        │   ├── vl53l0x_platform.c
        │   ├── vl53l0x_platform.h
        │   ├── vl53l0x_platform_log.h
        │   ├── vl53l0x_tuning.h
        │   └── vl53l0x_types.h
        ├── vl53l0x_api_calibration.o
        ├── vl53l0x_api_core.o
        ├── vl53l0x_api.o
        ├── vl53l0x_api_strings.o
        └── vl53l0x_platform.o
docs/
├── digital.csv
├── img
│   ├── 0001.png
│   ├── 0002.png
│   ├── 0003.png
│   ├── 0004.png
│   ├── 0005.png
│   ├── 0006.png
│   └── 0007.png
└── VANGUARD-PASIL-Pratik_Suryawanshi.pdf
Ground-Control-Station/
├── GCS
│   ├── GCS.ino
│   └── payload.h
└── README.md
KiCAD_Files/
└── VANGUARD-PASIL
    ├── GCS.kicad_sch
    ├── VANGAURD-PASIL_Symbols.bak
    ├── VANGAURD-PASIL_Symbols.kicad_sym
    ├── VANGUARD-PASIL-backups
    │   ├── VANGUARD-PASIL-2026-04-28_141102.zip
    │   ├── VANGUARD-PASIL-2026-04-28_175334.zip
    │   ├── VANGUARD-PASIL-2026-04-28_181030.zip
    │   ├── VANGUARD-PASIL-2026-04-28_195817.zip
    │   ├── VANGUARD-PASIL-2026-04-28_195952.zip
    │   ├── VANGUARD-PASIL-2026-04-28_200031.zip
    │   ├── VANGUARD-PASIL-2026-04-28_200310.zip
    │   ├── VANGUARD-PASIL-2026-04-29_194642.zip
    │   ├── VANGUARD-PASIL-2026-04-29_200738.zip
    │   ├── VANGUARD-PASIL-2026-04-29_210452.zip
    │   ├── VANGUARD-PASIL-2026-04-29_210541.zip
    │   ├── VANGUARD-PASIL-2026-04-29_214114.zip
    │   ├── VANGUARD-PASIL-2026-04-29_214334.zip
    │   ├── VANGUARD-PASIL-2026-04-29_214513.zip
    │   ├── VANGUARD-PASIL-2026-04-29_214648.zip
    │   ├── VANGUARD-PASIL-2026-04-29_214719.zip
    │   ├── VANGUARD-PASIL-2026-04-29_214737.zip
    │   ├── VANGUARD-PASIL-2026-04-29_224135.zip
    │   ├── VANGUARD-PASIL-2026-04-29_225311.zip
    │   ├── VANGUARD-PASIL-2026-04-29_225843.zip
    │   ├── VANGUARD-PASIL-2026-04-29_234853.zip
    │   ├── VANGUARD-PASIL-2026-04-29_235208.zip
    │   ├── VANGUARD-PASIL-2026-04-30_000211.zip
    │   ├── VANGUARD-PASIL-2026-04-30_000333.zip
    │   ├── VANGUARD-PASIL-2026-04-30_001110.zip
    │   ├── VANGUARD-PASIL-2026-04-30_001237.zip
    │   ├── VANGUARD-PASIL-2026-04-30_001340.zip
    │   ├── VANGUARD-PASIL-2026-04-30_001416.zip
    │   ├── VANGUARD-PASIL-2026-04-30_001615.zip
    │   ├── VANGUARD-PASIL-2026-04-30_001829.zip
    │   ├── VANGUARD-PASIL-2026-04-30_002008.zip
    │   ├── VANGUARD-PASIL-2026-04-30_095227.zip
    │   ├── VANGUARD-PASIL-2026-04-30_110327.zip
    │   ├── VANGUARD-PASIL-2026-04-30_132318.zip
    │   └── VANGUARD-PASIL-2026-04-30_133915.zip
    ├── VANGUARD-PASIL.kicad_pcb
    ├── VANGUARD-PASIL.kicad_prl
    ├── VANGUARD-PASIL.kicad_pro
    └── VANGUARD-PASIL.kicad_sch
```


## 🧪 Validation Environment
High-RPM motor vibration testing
RF packet loss injection
Power noise injection from ESC switching

## 📌 Notes
Designed for deterministic execution under worst-case conditions
Prioritizes control loop integrity over peripheral convenience
Suitable for research, validation, and safety-critical prototyping

## ⚠️ Disclaimer

This system is intended for research and development purposes.
Improper use in real-world flight systems may result in hardware damage or safety risks.
