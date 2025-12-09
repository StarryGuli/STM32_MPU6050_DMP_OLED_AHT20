/*
 * aht20.c
 *
 *  Created on: Sep 23, 2025
 *      Author: Hallo
 */

//需要在i2c.c中添加以下状态回传

//void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
//{
//	if(hi2c==&hi2c2)
//	{
//		aht20state = 2;
//		lastTick = HAL_GetTick();
//	}
//
//}
//void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
//{
//	if(hi2c==&hi2c2)
//	{
//		aht20state = 4;
//		lastTick = HAL_GetTick();
//	}
//}



//在main.c中的使用方法

//uint8_t aht20state = 0;		//需要的变量和状态指示
//float T, H;
//
//	  if(aht20state==0)
//		  {
//		  AHT20_Measure();
//	  	  aht20state = 1;
//		  }
//	  else if(aht20state==2)
//	  {
//		  if(HAL_GetTick() - lastTick >= 75)
//		  {
//		  AHT20_Get();
//		  aht20state = 3;
//		  }
//	  }
//	  else if(aht20state==4)
//	  {
//		  if(HAL_GetTick() - lastTick >= 1000)
//		  {
//		  AHT20_Analysis(&T, &H);
//		  sprintf(message,"温度:%.1f℃\n湿度:%.1f%%\n",T,H);
//		  sprintf(messageT,"温度:%.1f℃",T);
//		  sprintf(messageH,"湿度:%.1f %%",H);
//		  HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
//		  aht20state = 0;
//		  }
//	  }



#include "aht20.h"

#define AHT20_ADDRESS 0x70

static uint8_t sendBuffer[3]={0xAC,0x33,0x00};
uint8_t readBuffer[6] = {0};

void AHT20_Init()
{
	uint8_t readBuffer;
	HAL_Delay(40);
	HAL_I2C_Master_Receive(&hi2c2, AHT20_ADDRESS, &readBuffer,1,HAL_MAX_DELAY);
	if((readBuffer & 0x08) == 0)
	{
		uint8_t sendBuffer[3]={0xBE,0x08,0x00};
		HAL_I2C_Master_Transmit(&hi2c1, AHT20_ADDRESS,sendBuffer, 3, HAL_MAX_DELAY);
	}

}


//发送测量指令
void AHT20_Measure()
{
	HAL_I2C_Master_Transmit_DMA(&hi2c2, AHT20_ADDRESS,sendBuffer, 3);
}

//发送接受指令
void AHT20_Get()
{
	HAL_I2C_Master_Receive_DMA(&hi2c2, AHT20_ADDRESS, readBuffer,6);
}

//温度数据解析
void AHT20_Analysis(float *temperature,float *humidity)
{
	if((readBuffer[0] & 0x80) == 0)
	{
		uint32_t date=0;
		date = ((uint32_t)readBuffer[3]>>4)+((uint32_t)readBuffer[2]<<8)+((uint32_t)readBuffer[1]<<12);
		*humidity = date*100.0f/(1<<20);
		date = (((uint32_t)readBuffer[3] & 0x0F) << 16) + ((uint32_t)readBuffer[4] << 8) + (uint32_t)readBuffer[5];
		*temperature = date*200.0/(1 << 20)-50;
	}
}

//三合一版本
//void AHT20_Read(float *temperature,float *humidity)
//{
//	HAL_I2C_Master_Transmit(&hi2c2, AHT20_ADDRESS,sendBuffer, 3, HAL_MAX_DELAY);
//	HAL_Delay(75);
//	HAL_I2C_Master_Receive(&hi2c2, AHT20_ADDRESS, readBuffer, 6, HAL_MAX_DELAY);
//	if((readBuffer[0] & 0x80) == 0)
//	{
//		uint32_t date=0;
//		date = ((uint32_t)readBuffer[3]>>4)+((uint32_t)readBuffer[2]<<8)+((uint32_t)readBuffer[1]<<12);
//		*humidity = date*100.0f/(1<<20);
//		date = (((uint32_t)readBuffer[3] & 0x0F) << 16) + ((uint32_t)readBuffer[4] << 8) + (uint32_t)readBuffer[5];
//		*temperature = date*200.0/(1 << 20)-50;
//	}
//
//}


