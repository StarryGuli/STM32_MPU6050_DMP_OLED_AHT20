# STM32 Monitor: MPU6050 (DMP) + AHT20 + OLED

> **STM32 implementation using MPU6050 (DMP) for attitude tracking and AHT20 for temperature/humidity monitoring, with real-time OLED display.**

![Status](https://img.shields.io/badge/Status-Active-brightgreen)
![Platform](https://img.shields.io/badge/Platform-STM32-blue)
![Language](https://img.shields.io/badge/Language-C++-orange)

## 📂 1. 项目文件说明 (File Description)

本项目主要基于 C++ 开发（为了兼容 MPU6050 DMP 库），核心驱动文件如下：

* **`MPU6050.cpp` / `MPU6050.h`**: 主驱动 (寄存器底层操作)。
* **`MPU6050_6Axis_MotionApps_V6_12.h`**: DMP 固件 (核心二进制代码，最关键文件)。
* **`I2Cdev.cpp` / `I2Cdev.h`**: I2C 通信层 (对接 STM32 HAL 库)。
* **`helper_3dmath.h`**: 数学计算库 (负责四元数转欧拉角)。
* **`ArduinoWrapper.cpp` / `ArduinoWrapper.h`**: 兼容层 (提供 millis/delay 函数转接到 HAL 库)。

> **⚠️ 注意事项**:
> 1. 由于引入了 C++ 库，**`main.c` 必须重命名为 `main.cpp`**。
> 2. 在 Keil/CubeIDE 中需确保 C++ 编译器配置正确。

## 🔌 2. 硬件与文件配置 (Setup)

### 2.1 硬件连接 (Hardware Connection)

| 模块 (Module) | 接口 (Interface) | STM32引脚 (Pin) | 备注 (Note) |
| :--- | :--- | :--- | :--- |
| **MCU** | Core | - | STM32F103 (或其他型号) |
| **Debug** | UART | UART2 | 串口调试输出 |
| **MPU6050** | I2C | I2C2 | 运动传感器 |
| **AHT20** | I2C | *Check Code* | 温湿度传感器 |
| **OLED** | I2C | *Check Code* | 显示屏 |

### 2.2 核心驱动文件 (Core Drivers)
请确保以下文件已包含在工程目录中（通常放在 `User` 或 `Core/Src` 文件夹）：

* **DMP 固件与算法**
  * `MPU6050_6Axis_MotionApps_V6_12.h`: **(关键)** DMP 固件二进制代码 (修复了文件名截断)。
  * `helper_3dmath.h`: 四元数与欧拉角计算辅助库。

* **传感器驱动**
  * `MPU6050.cpp` / `MPU6050.h`: MPU6050 寄存器操作层。
  * `I2Cdev.cpp` / `I2Cdev.h`: I2C 通信抽象层。

* **平台兼容层 (Porting Layer)**
  * `ArduinoWrapper.cpp` / `ArduinoWrapper.h`: 将 Arduino 函数 (`millis`, `delay`) 映射到 STM32 HAL 库，确保库文件能正常编译。

---

## 💻 3. 核心代码实现 (Core Implementation)


### 3.1 头文件引用 (Includes)
在 `main.cpp` 中引入库文件。

```cpp
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include <stdio.h>
#include <string.h>
#include "MPU6050_6Axis_MotionApps_V6_12.h" // 最重要的 DMP 固件库
/* USER CODE END Includes */
```

### 3.2 变量定义 (Variables)

定义 DMP 解算与数据显示所需的全局变量。

```C++
/* USER CODE BEGIN PV */
// --- MPU6050 DMP 相关变量 ---
MPU6050 mpu;             // 实例化对象

// DMP 控制/状态变量
uint8_t devStatus;       // 操作状态 (0 = 成功, !0 = 错误)
uint16_t packetSize;     // DMP 数据包大小
uint16_t fifoCount;      // FIFO 当前字节数
uint8_t fifoBuffer[64];  // FIFO 存储缓冲区

// 方向/运动变量
Quaternion q;            // [w, x, y, z] 四元数
VectorFloat gravity;     // [x, y, z] 重力向量
float ypr[3];            // [yaw, pitch, roll] 偏航/俯仰/翻滚角

// 数据显示缓存
char message_angle_1[30];
char message_angle_2[30];
char message_angle_3[30];
int16_t currentOffsets[6]; // 存储校准后的偏移量
/* USER CODE END PV */
```

### 3.3 串口重定向 (Printf Redirect)

为了使用 `printf` 调试，需要重定向 fputc（基于 HAL_UART_Transmit）。

```C++
/* USER CODE BEGIN 0 */
// 重定向 printf 到串口 (使用 huart2)
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
    return ch;
}
/* USER CODE END 0 */
```

### 3.4 初始化 MPU6050 (Initialization)

在 `main()` 函数的 `USER CODE BEGIN 2` 区域添加。

```C++
/* USER CODE BEGIN 2 */
HAL_Delay(100); // 等待传感器上电稳定

printf("UART READY\r\n");
printf("Initializing DMP...\r\n");

mpu.initialize(); // 初始化 I2C 设备

// 检测连接
if(mpu.testConnection()){
    printf("MPU6050 connection successful\r\n");
} else {
    printf("MPU6050 connection failed\r\n");
}

// 初始化 DMP
devStatus = mpu.dmpInitialize();

// 验证 DMP 初始化结果
if (devStatus == 0) {
    // 开启自动校准 (注意：开机时请静止平放 2 秒)
    printf("Calibrating... DO NOT MOVE\r\n");
    mpu.CalibrateAccel(30);
    mpu.CalibrateGyro(30);
    mpu.PrintActiveOffsets();
    
    // 开启 DMP
    printf("Enabling DMP...\r\n");
    mpu.setDMPEnabled(true);
    
    packetSize = mpu.dmpGetFIFOPacketSize();
    printf("DMP Ready!\r\n");
    
    // 保存当前校准偏移量
    currentOffsets[0] = mpu.getXAccelOffset();
    currentOffsets[1] = mpu.getYAccelOffset();
    currentOffsets[2] = mpu.getZAccelOffset();
    currentOffsets[3] = mpu.getXGyroOffset();
    currentOffsets[4] = mpu.getYGyroOffset();
    currentOffsets[5] = mpu.getZGyroOffset();
} else {
    // 初始化失败 (1 = 内存加载失败, 2 = 配置失败)
    printf("DMP Init failed (code %d)\r\n", devStatus);
}
/* USER CODE END 2 */
```

### 3.5 主循环 (Main Loop)

在 `while(1)` 中读取 FIFO 数据并解算欧拉角。

```C++
/* USER CODE BEGIN 3 */
if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
{
    // 1. 获取四元数
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    
    // 2. 获取重力向量
    mpu.dmpGetGravity(&gravity, &q);
    
    // 3. 计算欧拉角 (Yaw, Pitch, Roll)
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    // 转换弧度为角度
    float yaw_deg   = ypr[0] * 180 / M_PI;
    float pitch_deg = ypr[1] * 180 / M_PI;
    float roll_deg  = ypr[2] * 180 / M_PI;
    
    // 格式化字符串供 OLED 显示 (Offset用于调试参考)
    sprintf(message_angle_1, "Yaw : %6.1f %5d", yaw_deg,   currentOffsets[5]);
    sprintf(message_angle_2, "Pitch: %6.1f %5d", pitch_deg, currentOffsets[4]);
    sprintf(message_angle_3, "Roll : %6.1f %5d", roll_deg,  currentOffsets[3]);
    
    // 串口打印调试 (建议增加延时控制打印频率)
    printf("Yaw:%.1f Pitch:%.1f Roll:%.1f\r\n", yaw_deg, pitch_deg, roll_deg);
    
    // TODO: 调用 AHT20 读取函数
    // TODO: 调用 OLED 显示函数 (OLED_ShowString...)
}
/* USER CODE END 3 */
```

## 📜 License

MIT License