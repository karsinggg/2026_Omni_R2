# 2026 Omni-directional Robot (v1)

An advanced holonomic robot control system implemented on the STM32H7 platform, featuring multi-loop PID control, sensor fusion, and omni-directional kinematics.

## 🚀 System Architecture

The system is built as a multi-threaded real-time application using **FreeRTOS**, ensuring deterministic execution of critical control loops.

### Hardware Stack
- **Microcontroller**: STM32H723ZGTx (ARM Cortex-M7)
- **Actuators**: 
  - M3508 / M2006 Brushless Motors
  - Servos
- **Sensors**:
  - BNO085 IMU (Orientation & Gesture)
  - High-resolution AS5047P Magnetic Encoders
  - BLF Laser Sensors (Distance/Positioning)
- **Communication**:
  - **FDCAN**: High-speed internal communication.
  - **SPI**: Communication protocol for AS5047P encoders.
  - **UART**: Communication between BLF Laser Sensors and STM32H7 MCU
  - **ESP32 (via I2C)**: Wireless bridge for remote control and telemetry.

### Software Architecture
The firmware is organized into several specialized FreeRTOS tasks:
- **PID Task**: Executes high-frequency control loops for motor speed and position.
- **Sensor Fusion Task**: Combines IMU and encoder data to maintain accurate robot state.
- **Navigation Task**: Implements autonomous navigation and state machine logic.
- **Communication Tasks**: Handles FDCAN transmission and ESP32 I2C interfacing.
- **Motion Profiling**: Implements virtual current position/velocity tracking to ensure smooth acceleration and deceleration.

## 🛠 Technical Stack

- **Language**: C
- **Framework**: STM32Cube HAL (Hardware Abstraction Layer)
- **RTOS**: FreeRTOS
- **IDE**: STM32CubeIDE
- **Communication Protocols**: I2C, SPI, FDCAN, UART

## 🧠 Key Components & Algorithms

### 1. Omni-directional Kinematics
The robot uses a holonomic drive system. The `omni_algorithm` translates the desired robot chassis velocity (translational $v_x$, $v_y$ and angular velocity $\omega$) into specific target speeds for each wheel.

### 2. Multi-loop PID Control
To achieve high precision, the system employs various PID configurations:
- **Cascade Control**: Horizontal cascade RPM-to-Current PID loops for the M3508 motors.
- **Orientation Control**: Dedicated PID loops for Heading (Yaw) and Pitch using the BNO085 IMU.
- **Positioning**: XY distance PID control for precise target reaching.

### 3. Motion Profiling
To prevent jerky movements and protect hardware, the system uses `MotionProfile_t` structures to implement:
- Velocity capping (Max speed)
- Controlled acceleration/deceleration ramps

### 4. Sensor Fusion
The `sensorFusion` module integrates data from the BNO085 IMU and AS5047P encoders to filter noise and provide a stable estimate of the robot's global pose.

## 📁 Project Structure
- `Core/Src`: Application source code (Main, Kinematics, PID, Drivers).
- `Core/Inc`: Header files for all system modules.
- `Drivers/`: STM32 HAL and CMSIS drivers.
- `Middlewares/`: FreeRTOS kernel and third-party libraries.
