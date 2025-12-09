# STM32_MPU6050_DMP_OLED_AHT20
TM32 implementation using MPU6050 (DMP) for attitude tracking and AHT20 for temperature/humidity monitoring, with real-time OLED display.
### MPU6050_DMP
1. 导入文件(在本文件中串口使用UART2,MPU6050连接在IIC2)
![[IMG-20251205170130097.png]]
`MPU6050.cpp` 和 `MPU6050.h` : 主驱动 (寄存器操作) 
`MPU6050_6Axis_MotionApps_V6_12.h` : DMP固件 (核心二进制代码) 
`I2Cdev.cpp` 和 `I2Cdev.h` : I2C通信层 (对接STM32 HAL库)
`helper_3dmath.h` : 数学计算库 (负责四元数转欧拉角) 
`ArduinoWrapper.cpp` 和 `ArduinoWrapper.h` : 兼容层 (提供 millis/delay 函数转接到HAL库)

2. 因为有其他文件是是C++的,所以main文件要用C++

3. 头文件引用 (Includes)
```C++
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include <stdio.h>
#include <string.h>
#include "MPU6050_6Axis_MotionApps_V6_12.h" // 最重要的 DMP 固件库
/* USER CODE END Includes */
```

4. 变量定义 (Variables)
```C++
/* USER CODE BEGIN PV */
// --- MPU6050 DMP 相关变量 ---
MPU6050 mpu; // 实例化对象

// DMP 控制/状态变量
uint8_t devStatus; // 操作状态 (0 = 成功, !0 = 错误)
uint16_t packetSize; // DMP 数据包大小
uint16_t fifoCount; // FIFO 当前字节数
uint8_t fifoBuffer[64]; // FIFO 存储缓冲区

// 方向/运动变量
Quaternion q; // [w, x, y, z] 四元数
VectorFloat gravity; // [x, y, z] 重力向量
float ypr[3]; // [yaw, pitch, roll] 偏航/俯仰/翻滚角
/* USER CODE END PV */
```

5. 串口重定向 (便于调试)
放在 区域 (这一步是为了让 `printf` 能用，如果新工程已经配好了 printf 可忽略)
```C++
/* USER CODE BEGIN 0 */
// 重定向 printf 到串口 (假设用的是 huart2，根据实际修改)
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
    return ch;
}
/* USER CODE END 0 */
```

5. 主函数准备
```c++
/* Private function prototypes -----------------------------------------------*/ void SystemClock_Config(void); 

HAL_Init();
/* Configure the system clock */
SystemClock_Config();

MX_GPIO_Init();
MX_USART2_UART_Init();
MX_I2C2_Init();
```

6. 初始化MPU6050
```c++
// 1. 初始化 MPU6050 DMP
HAL_Delay(100);
printf("UART READY\r\n");
printf("Initializing DMP...\r\n");
mpu.initialize(); // 初始化 I2C 设备
if(mpu.testConnection()){
printf("MPU6050 connection successful\r\n");
} else {
printf("MPU6050 connection failed\r\n");
}
devStatus = mpu.dmpInitialize();
// 2. 验证 DMP 初始化结果
if (devStatus == 0) {
// 开启自动校准 (开机时请静止平放 2 秒)
printf("Calibrating... DO NOT MOVE\r\n");
mpu.CalibrateAccel(30);
mpu.CalibrateGyro(30);
mpu.PrintActiveOffsets();
// 开启 DMP
printf("Enabling DMP...\r\n");
mpu.setDMPEnabled(true);
packetSize = mpu.dmpGetFIFOPacketSize();
printf("DMP Ready!\r\n");
} else {
// 初始化失败 (1 = 内存加载失败, 2 = 配置失败)
printf("DMP Init failed (code %d)\r\n", devStatus);
}
// 定义一个结构体或者数组来存这些值
int16_t currentOffsets[6];
// 在你需要获取数据的地方调用：
currentOffsets[0] = mpu.getXAccelOffset();
currentOffsets[1] = mpu.getYAccelOffset();
currentOffsets[2] = mpu.getZAccelOffset();
currentOffsets[3] = mpu.getXGyroOffset();
currentOffsets[4] = mpu.getYGyroOffset();
currentOffsets[5] = mpu.getZGyroOffset();
```

7. while循环
```c++
while (1)
{
if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
{
// 1. 获取四元数
mpu.dmpGetQuaternion(&q, fifoBuffer);
// 2. 获取重力向量
mpu.dmpGetGravity(&gravity, &q);
// 3. 计算欧拉角 (Yaw, Pitch, Roll)
mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
// 转换弧度为角度
float yaw_deg = ypr[0] * 180 / M_PI;
float pitch_deg = ypr[1] * 180 / M_PI;
float roll_deg = ypr[2] * 180 / M_PI;
// 格式化字符串供 OLED 显示
sprintf(message_angle_1, "Yaw : %6.1f %5d", yaw_deg,currentOffsets[5]);
sprintf(message_angle_2, "Pitch: %6.1f %5d", pitch_deg,currentOffsets[4]);
sprintf(message_angle_3, "Roll : %6.1f %5d", roll_deg,currentOffsets[3]);
// 串口打印调试 (不需要每次循环都打印，可以加个延时判断)
printf("Yaw:%.1f Pitch:%.1f Roll:%.1f\r\n", yaw_deg, pitch_deg, roll_deg);
}
