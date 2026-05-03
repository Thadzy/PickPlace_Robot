# STM32 Firmware

Firmware for the FRA502 Circular Pick and Place Robot, built for the **STM32G474RET6** (Nucleo-G474RE).

This firmware implements a high-performance PID motor controller with Modbus RTU communication, trajectory generation, and safety monitoring.

---

## 🚀 Quick Start

### 1. Prerequisites
- **STM32CubeIDE** (Version 1.12.0 or newer recommended)
- **ST-Link Drivers** (included with CubeIDE)
- A **Nucleo-G474RE** board

### 2. Setup & Development
1. **Import:** Open STM32CubeIDE and select `File > Import > Existing Projects into Workspace`. Browse to the `Firmware/STM32_Firmware` folder.
2. **Build:** Click the **Hammer icon** (Build) to compile the code.
3. **Flash:** Click the **Play icon** (Run) or **Bug icon** (Debug) to flash the firmware to your Nucleo board via USB.

---

## 🔌 Hardware Connection

The firmware uses the following pin configuration. If you are using a standard Nucleo-G474RE, the LPUART1 is internally connected to the USB ST-Link Virtual COM Port.

| Feature | Pin | Details |
|---------|-----|---------|
| **Modbus (LPUART1)** | PA2 (TX), PA3 (RX) | 19200 Baud, 8 Data bits, **Even Parity**, 1 Stop bit |
| **Joystick / Debug** | PB10 (TX), PB11 (RX) | 115200 Baud, 8N1. Used for `printf` and Joystick input |
| **Motor PWM** | PC7 (TIM8_CH2) | 20kHz PWM Frequency |
| **Motor Direction** | PC6 | Digital Output (High/Low) |
| **Encoder** | PA6 (A), PA7 (B) | Quadrature Encoder (TIM3) |
| **Gripper PWM** | [TBD] | Servo-style control for gripper |
| **Status LED** | PA5 (LD2) | Blinks during normal operation |

---

## 🧠 How the Code Works

The firmware is structured into three main layers:

### 1. Protocol Layer (`modbus_rtu.c` & `modbus_bridge.c`)
- **Modbus RTU Slave:** Implements a standard Modbus stack listening on **Slave ID 21**.
- **Bridge:** Maps Modbus registers to internal variables like `target_position` or `current_speed`.

### 2. Control Layer (`motor_controller.c`)
- **PID Loops:** Runs a dual-loop control system at **100Hz**.
  - **Inner Loop:** Speed control (PI with Feed-Forward).
  - **Outer Loop:** Position control (PID).
- **Trajectory Generation:** Uses S-Curve / Trapezoidal profiles for smooth starts and stops.
- **Autotuning:** Built-in relay-based autotuning for both speed and position loops.

### 3. Safety Layer
- **Stall Detection:** Triggers a fault if the motor is drawing power but not moving.
- **Soft Limits:** Prevents rotation beyond defined degree limits (default 720°).
- **Emergency Stop:** Immediate shutdown logic triggered via Modbus or hardware.

---

## 📊 Modbus Register Map

Use these registers to control the robot from the Base System or custom scripts.

| Address | Type | Name | Description |
|---------|------|------|-------------|
| **0x01** | W | Command Bits | bit 0: Home, bit 1: Trigger Jog, bit 2: Auto Mode, bit 3: Reset Encoder |
| **0x03** | W | Gripper Cmd | 1: Pick Sequence, 2: Place Sequence |
| **0x05** | W | Jog Step | Incremental move (signed degrees) |
| **0x24** | W | Target Pos | Absolute target position (degrees) |
| **0x25** | W | Safety Cmd | 1: Emergency Stop, 2: Resume |
| **0x28** | R | Current Pos | Measured position in degrees (multiplied by 10) |
| **0x29** | R | Current Speed | Measured speed in RPM (multiplied by 10) |
| **0x31** | R | Safety Status | 0: OK, 1: Emergency Stop Active |

---

## 🛠️ Development Workflow

- **Tuning:** Adjust PID gains in `Inc/params.h` for permanent changes, or use the Base System UI for real-time tuning.
- **Monitoring:** Open a Serial Monitor (like PuTTY or the CubeIDE Console) on the ST-Link COM port at **115200 baud** to see debug logs and telemetry.
- **Faults:** If the motor stops responding, check the `Safety Status` register. You may need to send a "Resume" command (0x25 = 2).

---

## ⚠️ Troubleshooting

- **No Communication:** Ensure your Modbus master is set to **19200, 8E1**. Check that you are targeting Slave ID 21.
- **Motor Jitters:** Check the Encoder connections (PA6/PA7). Inverting the A/B lines may be necessary if the motor runs away.
- **Build Errors:** Ensure you have imported the project correctly into STM32CubeIDE. Do not move files out of the `STM32_Firmware` directory.
