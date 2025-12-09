# STM32 Monitor: MPU6050 (DMP) + AHT20 + OLED

> **STM32 implementation using MPU6050 (DMP) for attitude tracking and AHT20 for temperature/humidity monitoring, with real-time OLED display.**

![Status](https://img.shields.io/badge/Status-Active-brightgreen)
![Platform](https://img.shields.io/badge/Platform-STM32-blue)
![Language](https://img.shields.io/badge/Language-C++-orange)

## 📂 1. Project File Description

This project is primarily developed in **C++** (to ensure compatibility with the MPU6050 DMP library). The core driver files are as follows:

* **`MPU6050.cpp` / `MPU6050.h`**: Main driver (Low-level register operations).
* **`MPU6050_6Axis_MotionApps_V6_12.h`**: DMP Firmware (Core binary code, critical file).
* **`I2Cdev.cpp` / `I2Cdev.h`**: I2C Communication Layer (Interfaces with STM32 HAL library).
* **`helper_3dmath.h`**: Math library (Handles Quaternion to Euler angle conversion).
* **`ArduinoWrapper.cpp` / `ArduinoWrapper.h`**: Porting Layer (Maps `millis`/`delay` functions to the HAL library).

> **⚠️ Important Notes**:
> 1. Since C++ libraries are used, **`main.c` must be renamed to `main.cpp`**.
> 2. Ensure the C++ compiler is correctly configured in Keil or STM32CubeIDE.

## 🔌 2. Hardware & Setup

### 2.1 Hardware Connection

| Module | Interface | STM32 Pin | Note |
| :--- | :--- | :--- | :--- |
| **MCU** | Core | - | STM32F103 (or similar) |
| **Debug** | UART | UART2 | Serial Debug Output |
| **MPU6050** | I2C | I2C2 | Motion Sensor |
| **AHT20** | I2C | *Check Code* | Temp/Humidity Sensor |
| **OLED** | I2C | *Check Code* | Display Screen |

### 2.2 Core Driver Files
Please ensure the following files are included in your project directory (usually in the `User` or `Core/Src` folder):

* **DMP Firmware & Algorithms**
  * `MPU6050_6Axis_MotionApps_V6_12.h`: **(Critical)** DMP Firmware binary code.
  * `helper_3dmath.h`: Helper library for Quaternion and Euler angle calculations.

* **Sensor Drivers**
  * `MPU6050.cpp` / `MPU6050.h`: MPU6050 register operation layer.
  * `I2Cdev.cpp` / `I2Cdev.h`: I2C abstraction layer.

* **Porting Layer**
  * `ArduinoWrapper.cpp` / `ArduinoWrapper.h`: Maps Arduino functions (`millis`, `delay`) to STM32 HAL library to ensure the library compiles correctly.

---

## 💻 3. Core Implementation

### 3.1 Includes
Include the necessary libraries in `main.cpp`.

```c++
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include <stdio.h>
#include <string.h>
#include "MPU6050_6Axis_MotionApps_V6_12.h" // The most important DMP firmware library
/* USER CODE END Includes */
```

### 3.2 Global Variables

Define the global variables required for DMP calculation and data display.

```C++
/* USER CODE BEGIN PV */
// --- MPU6050 DMP Objects ---
MPU6050 mpu;             // Instantiate object

// DMP Control/Status Variables
uint8_t devStatus;       // Operation status (0 = success, !0 = error)
uint16_t packetSize;     // DMP packet size
uint16_t fifoCount;      // Current bytes in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// Orientation/Motion Variables
Quaternion q;            // [w, x, y, z] Quaternion
VectorFloat gravity;     // [x, y, z] Gravity vector
float ypr[3];            // [yaw, pitch, roll] Euler angles

// Display Buffer
char message_angle_1[30];
char message_angle_2[30];
char message_angle_3[30];
int16_t currentOffsets[6]; // Store offsets after calibration
/* USER CODE END PV */
```

### 3.3 Printf Redirect

To use `printf` for debugging, redirect `fputc` (based on HAL_UART_Transmit).

```C++
/* USER CODE BEGIN 0 */
// Redirect printf to UART (Using huart2)
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
    return ch;
}
/* USER CODE END 0 */
```

### 3.4 Initialization

Add this code inside the `main()` function within the `USER CODE BEGIN 2` section.

```C++
/* USER CODE BEGIN 2 */
HAL_Delay(100); // Wait for sensors to power up and stabilize

printf("UART READY\r\n");
printf("Initializing DMP...\r\n");

mpu.initialize(); // Initialize I2C device

// Check connection
if(mpu.testConnection()){
    printf("MPU6050 connection successful\r\n");
} else {
    printf("MPU6050 connection failed\r\n");
}

// Initialize DMP
devStatus = mpu.dmpInitialize();

// Verify DMP initialization result
if (devStatus == 0) {
    // Enable Auto-Calibration (Note: Keep device still and flat during startup)
    printf("Calibrating... DO NOT MOVE\r\n");
    mpu.CalibrateAccel(30);
    mpu.CalibrateGyro(30);
    mpu.PrintActiveOffsets();
    
    // Enable DMP
    printf("Enabling DMP...\r\n");
    mpu.setDMPEnabled(true);
    
    packetSize = mpu.dmpGetFIFOPacketSize();
    printf("DMP Ready!\r\n");
    
    // Save current calibration offsets
    currentOffsets[0] = mpu.getXAccelOffset();
    currentOffsets[1] = mpu.getYAccelOffset();
    currentOffsets[2] = mpu.getZAccelOffset();
    currentOffsets[3] = mpu.getXGyroOffset();
    currentOffsets[4] = mpu.getYGyroOffset();
    currentOffsets[5] = mpu.getZGyroOffset();
} else {
    // Initialization failed (1 = Memory load failed, 2 = Configuration failed)
    printf("DMP Init failed (code %d)\r\n", devStatus);
}
/* USER CODE END 2 */
```

### 3.5 Main Loop

Read FIFO data and calculate Euler angles inside `while(1)`.

```C++
/* USER CODE BEGIN 3 */
if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
{
    // 1. Get Quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    
    // 2. Get Gravity Vector
    mpu.dmpGetGravity(&gravity, &q);
    
    // 3. Calculate Euler Angles (Yaw, Pitch, Roll)
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    // Convert Radians to Degrees
    float yaw_deg   = ypr[0] * 180 / M_PI;
    float pitch_deg = ypr[1] * 180 / M_PI;
    float roll_deg  = ypr[2] * 180 / M_PI;
    
    // Format string for OLED display (Offsets used for debug reference)
    sprintf(message_angle_1, "Yaw : %6.1f %5d", yaw_deg,   currentOffsets[5]);
    sprintf(message_angle_2, "Pitch: %6.1f %5d", pitch_deg, currentOffsets[4]);
    sprintf(message_angle_3, "Roll : %6.1f %5d", roll_deg,  currentOffsets[3]);
    
    // Serial debug print (Suggestion: Add delay to control print frequency)
    printf("Yaw:%.1f Pitch:%.1f Roll:%.1f\r\n", yaw_deg, pitch_deg, roll_deg);
    
    // TODO: Call AHT20 reading function
    // TODO: Call OLED display function (OLED_ShowString...)
}
/* USER CODE END 3 */
```

## 📜 License

MIT License