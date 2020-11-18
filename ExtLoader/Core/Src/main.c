/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Loader_Src.h"
#include "Dev_Inf.h"

int main(void)
{
#define BUFFER_SIZE	20
	uint8_t buffer_rx[BUFFER_SIZE] = {0};
	uint8_t buffer_tx[BUFFER_SIZE] = {0,1,2,3,4,5,6};
	uint64_t ret_val = 0;
	Init();
	if(Read(MMC_ASSIGNED_ADDRESS, BUFFER_SIZE, buffer_rx) == 0)
		buffer_tx[0] = 0xFF;
	if(Write(MMC_ASSIGNED_ADDRESS, BUFFER_SIZE, buffer_tx) == 0)
		buffer_tx[0] = 0xFF;
	ret_val = Verify(MMC_ASSIGNED_ADDRESS, (uint32_t)buffer_tx, BUFFER_SIZE);
	CheckSum(MMC_ASSIGNED_ADDRESS, 0x800000, 0);
	Init();
	if(Read(MMC_ASSIGNED_ADDRESS, BUFFER_SIZE, buffer_rx) == 0)
		buffer_tx[0] = 0xFF;
	while (1)
	{
	}
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
