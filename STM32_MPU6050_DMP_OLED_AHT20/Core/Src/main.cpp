/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include<stdio.h>
#include<string.h>
#include "MPU6050_6Axis_MotionApps_V6_12.h"
extern "C"
{
#include "oled.h"      // 请确认你的 OLED 头文件具体名字 (比如 oled.h 或 OLED.h)
#include "AHT20.h"     // 请确认你的 AHT20 头文件具体名字
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//0:初始状态;1:正在发送测量命令;2:测量命令发送完完成;3:读取数据中;4:数据读取完成
uint8_t aht20state = 0;
uint32_t lastTick = 0;
uint32_t animTick = 0;

// DMP 控制/状态变量
uint8_t devStatus;      // 操作状态 (0 = 成功, !0 = 错误)
uint16_t packetSize;    // DMP 数据包大小
uint16_t fifoCount;     // FIFO 当前字节数
uint8_t fifoBuffer[64]; // FIFO 存储缓冲区

// 方向/运动变量
Quaternion q;           // [w, x, y, z]         四元数
VectorFloat gravity;    // [x, y, z]            重力向量
float ypr[3];           // [yaw, pitch, roll]   偏航/俯仰/翻滚角

float T, H;
char message[50];
char messageT[50];
char messageH[50];
char message_mpu_A[50];
char message_mpu_G[50];
char message_angle_1[50]; // 原 message_mpu_A 改名
char message_angle_2[50]; // 原 message_mpu_G 改名
char message_angle_3[50]; // 原 message_mpu_G 改名

//显示时间
uint32_t current_min = 0;      // 当前运行分钟数
uint32_t last_min_update = 0xFFFFFFFF; // 上一次刷新的分钟数 (设为最大值确保第一次开机必刷)
char time_msg[20];

uint8_t Key1_Pressed = 0;
uint8_t Key2_Pressed = 0;
MPU6050 mpu;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//int fputc(int ch, FILE *f)
//{
//	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
//	return ch;
//}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_0)
    {
        // 这里就是按键上升沿事件
        Key1_Pressed = 1;
    }
    if(GPIO_Pin == GPIO_PIN_13)
        {
            // 这里就是按键上升沿事件
            Key2_Pressed = 1;
        }
}


void OLED_Animation()
{
	static uint8_t i=1;
	static char state[2]={0,0};
	if(HAL_GetTick()-animTick>=10)
	{

		if((i*2)==2)
		{
			state[0] = 1;
			state[1] = 1;
		}
		else
		{
			if((i*4)>=128){state[0] = 0;}
			if((i*6)>=128){state[1] = 0;}
		}


		OLED_NewFrame();
		//		 OLED_DrawCircle(128, 0, i*2, OLED_COLOR_NORMAL);
		//		 OLED_DrawCircle(128, 0, i*4, OLED_COLOR_NORMAL);
		//		 OLED_DrawCircle(128, 0, i*6, OLED_COLOR_NORMAL);

		OLED_DrawCircle(128, 0, (i*2) , OLED_COLOR_NORMAL);
		OLED_DrawCircle(0, 64, (i*2) , OLED_COLOR_NORMAL);
		//		 if(((i*2) % 128)>=100||(i*2)==2)
		if(state[0]==1)
		{
			OLED_DrawCircle(128, 0, (i*4) % 128 , OLED_COLOR_NORMAL);
			OLED_DrawCircle(0, 64, (i*4) % 128 , OLED_COLOR_NORMAL);
		}
		if(state[1]==1)
		{
			OLED_DrawCircle(128, 0, (i*6) % 128, OLED_COLOR_NORMAL);
			OLED_DrawCircle(0, 64, (i*6) % 128, OLED_COLOR_NORMAL);
		}

		//		 OLED_DrawCircle(128, 0, (i + 0)  % 128, OLED_COLOR_NORMAL);
		//		 OLED_DrawCircle(128, 0, (i + 20) % 128, OLED_COLOR_NORMAL);
		//		 OLED_DrawCircle(128, 0, (i + 40) % 128, OLED_COLOR_NORMAL);


		//		 OLED_PrintString(64-2*i , 40 , "波特律动", &font16x16, OLED_COLOR_NORMAL);
		//		 OLED_PrintString(128-4*i, 20, "Starry", &font21x25, OLED_COLOR_NORMAL);
		OLED_PrintString(0, 0, messageT, &font16x16, OLED_COLOR_NORMAL);
		OLED_PrintString(65, 0, messageH, &font16x16, OLED_COLOR_NORMAL);
		//		OLED_PrintASCIIString(0,34,message_mpu_A,&afont8x6,0);
		//		OLED_PrintASCIIString(0,40,message_mpu_G,&afont8x6,0);
		OLED_PrintASCIIString(0, 19, message_angle_1, &afont8x6, OLED_COLOR_NORMAL);
		OLED_PrintASCIIString(0, 29, message_angle_2, &afont8x6, OLED_COLOR_NORMAL);
		OLED_PrintASCIIString(0, 39, message_angle_3, &afont8x6, OLED_COLOR_NORMAL);
		//		 OLED_PrintASCIIString(0,34,state,&afont16x8,0);
		OLED_PrintASCIIString(0, 48, (char*) "MPU6050 DMP", &afont16x8, OLED_COLOR_NORMAL);

        OLED_PrintASCIIString(88, 56, time_msg, &afont8x6, OLED_COLOR_NORMAL);
;
		OLED_ShowFrame();

		i+=1;

		if (i >= 64) i = 1;
		animTick = HAL_GetTick();
	}

}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */


	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_USART2_UART_Init();
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */
	// 1. 初始化外设
	AHT20_Init();
	HAL_Delay(20);
	OLED_Init();


	// 2. 初始化 MPU6050 DMP
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
	// 3. 验证 DMP 初始化结果
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


	//  OLED_NewFrame();
	//  OLED_SetPixel(0, 0, OLED_COLOR_NORMAL);
	//
	//  OLED_ShowFrame();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		//	  for(uint8_t i=0;i<64;i++)
		//	  {
		//		 OLED_NewFrame();
		//		 OLED_DrawCircle(64, 32, i, OLED_COLOR_NORMAL);
		//		 OLED_DrawCircle(64, 32, i*2, OLED_COLOR_NORMAL);
		//		 OLED_DrawCircle(64, 32, i*3, OLED_COLOR_NORMAL);
		////		 OLED_PrintString(64-2*i , 40 , "波特律动", &font16x16, OLED_COLOR_NORMAL);
		//		 OLED_PrintString(128-4*i, 20, "Starry", &font21x25, OLED_COLOR_NORMAL);
		//		 OLED_ShowFrame();
		//		 HAL_Delay(10);
		//	  }


		//	  AHT20_Read(&T, &H);
		//	  sprintf(message,"温度:%.1f℃\n湿度:%.1f%%\n",T,H);
		//	  sprintf(messageT,"温度:%.1f℃",T);
		//	  sprintf(messageH,"湿度:%.1f %%",H);
		//	  HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
		//	  OLED_NewFrame();
		////	  OLED_DrawCircle(64, 32, 50, OLED_COLOR_NORMAL);
		//	  OLED_PrintString(0,0,messageT,&font16x16,0);
		//	  OLED_PrintString(0,17,messageH,&font16x16,0);
		//	  OLED_PrintASCIIString(56, 48, "V2 Starry", &afont16x8, OLED_COLOR_NORMAL);
		//
		//	  OLED_ShowFrame();
		//
		//	  HAL_Delay(1000);

		OLED_Animation();

		//mpu6050
		//		char uart_buf[128];
		//
		//		if (MPU6050_ReadAll(&mpu_data) == HAL_OK)
		//		{
		//			int len = sprintf(uart_buf,
		//					"AX=%d AY=%d AZ=%d | GX=%d GY=%d GZ=%d\r\n",
		//					mpu_data.ax, mpu_data.ay, mpu_data.az,
		//					mpu_data.gx, mpu_data.gy, mpu_data.gz);
		//			sprintf(message_mpu_A,"AX=%d AY=%d AZ=%d", mpu_data.ax, mpu_data.ay, mpu_data.az);
		//			sprintf(message_mpu_G,"GX=%d GY=%d GZ=%d", mpu_data.gx, mpu_data.gy, mpu_data.gz);
		//
		//			HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, len, HAL_MAX_DELAY);
		//		}
		// --- 任务 2: MPU6050 数据读取与解算 ---
		// dmpGetCurrentFIFOPacket 会自动检查数据是否这就绪，并且处理 FIFO 溢出
//		uint16_t fifoCount = mpu.getFIFOCount();
//		printf("Current FIFO: %d\n", fifoCount); // 看看这个数是不是 0
//		HAL_Delay(100); // 慢点打印，别刷屏
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

			// 格式化字符串供 OLED 显示
			sprintf(message_angle_1, "Yaw  : %6.1f  %5d", yaw_deg,currentOffsets[5]);
			sprintf(message_angle_2, "Pitch: %6.1f  %5d", pitch_deg,currentOffsets[4]);
			sprintf(message_angle_3, "Roll : %6.1f  %5d", roll_deg,currentOffsets[3]);

			// 串口打印调试 (不需要每次循环都打印，可以加个延时判断)
			 printf("Yaw:%.1f Pitch:%.1f Roll:%.1f\r\n", yaw_deg, pitch_deg, roll_deg);
		}

//		HAL_Delay(50);
		//运行中更改GyroOffset
		if(Key1_Pressed==1)
		{
			mpu.setXGyroOffset(++currentOffsets[5]);
//			while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==GPIO_PIN_SET)
//			{
//				mpu.setXGyroOffset(++currentOffsets[5]);
//				HAL_Delay(10);
//			}
			Key1_Pressed=0;
		}

		if(Key2_Pressed==1)
		{
			mpu.setXGyroOffset(--currentOffsets[5]);
//			while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==GPIO_PIN_SET)
//			{     927645




//				mpu.setXGyroOffset(--currentOffsets[5]);
//				HAL_Delay(10);
//			}
			Key2_Pressed=0;
		}

		//时间计算
		// 1. 获取当前分钟数 (HAL_GetTick返回毫秒，除以60000得到分钟)
		      current_min = HAL_GetTick() / 60000;

		      // 2. 核心逻辑：只有当分钟数发生变化时，才执行 OLED 操作
		      // 这样平时 CPU 只是做了一个简单的减法和比较，几乎不占资源
		      if (current_min != last_min_update)
		      {
		          last_min_update = current_min; // 更新记录

		          // 3. 格式化字符串 (这里用 %ld 因为是 long int)
		          // 这里的 %2ld 表示占2位，%02ld 表示不足2位补0
		          sprintf(time_msg, "%4ld m", current_min);
		      }

		if(aht20state==0)
		{
			AHT20_Measure();
			aht20state = 1;
		}
		else if(aht20state==2)
		{
			if(HAL_GetTick() - lastTick >= 75)
			{
				AHT20_Get();
				aht20state = 3;
			}
		}
		else if(aht20state==4)
		{
			//		  AHT20_Analysis(&T, &H);
			//		  sprintf(message,"温度:%.1f℃\n湿度:%.1f%%\n",T,H);
			//		  sprintf(messageT,"温度:%.1f℃",T);
			//		  sprintf(messageH,"湿度:%.1f %%",H);
			//		  HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
			//		  OLED_NewFrame();
			////		  OLED_DrawCircle(64, 32, 50, OLED_COLOR_NORMAL);
			//		  OLED_PrintString(0,0,messageT,&font16x16,0);
			//		  OLED_PrintString(0,17,messageH,&font16x16,0);
			//		  OLED_PrintASCIIString(56, 48, "V2 Starry", &afont16x8, OLED_COLOR_NORMAL);
			//
			//		  OLED_ShowFrame();

			if(HAL_GetTick() - lastTick >= 1000)
			{
				AHT20_Analysis(&T, &H);
				sprintf(message,"温度:%.1f℃\n湿度:%.1f%%\n",T,H);
				sprintf(messageT,"T:%.1f℃",T);
				sprintf(messageH,"H:%.1f %%",H);
				HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
				aht20state = 0;
			}
		}


		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
