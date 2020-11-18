/**
  ******************************************************************************
  * @file    Loader_Src.c
  * @author  MCD Tools Team
  * @date    October-2015
  * @brief   This file defines the operations of the external loader for
  *          N25Q256A QSPI memory of STM32L476G-EVAL.
  ******************************************************************************
  */

#include "Loader_Src.h"
#include "Dev_Inf.h"
#include <string.h>

extern uint32_t SystemCoreClock;


void SystemClock_Config(void);
void Init_Error_Handler();
int MMC_INIT();
int MMC_read_bytes(uint8_t* dst, uint8_t* address, uint32_t size);
int MMC_write_bytes(uint8_t* src, uint8_t* address, uint32_t size);
HAL_StatusTypeDef ReadMMCBlock(uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks, uint32_t Timeout);
HAL_StatusTypeDef WriteMMCBlock(uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks, uint32_t Timeout);

MMC_HandleTypeDef hmmc;


void dbg_led_on(led_colour_t led_colour){
	switch(led_colour){
	case GREEN:
		  HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_5, GPIO_PIN_SET);
		break;
	case RED:
		  HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_13, GPIO_PIN_SET);
		break;
	}
}
void dbg_led_off(led_colour_t led_colour){
	switch(led_colour){
	case GREEN:
		  HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_5, GPIO_PIN_RESET);
		break;
	case RED:
		  HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_13, GPIO_PIN_RESET);
		break;
	}
}

void DBG_LEDS_Init(){
	__HAL_RCC_GPIOJ_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);
}
#define DELAY	20
/**
  * @brief  System initialization.
  * @param  None
  * @retval  1      : Operation succeeded
  * @retval  0      : Operation failed
  */
int Init ()
{
	//HAL_Init();
	__disable_irq();
	DBG_LEDS_Init();
	dbg_led_off(GREEN);
	dbg_led_off(RED);
	SystemClock_Config();
	SystemCoreClock = 16000000;
	if (MMC_INIT()){
		return 1;
	}else{
		return 0;
	}
}



int Write (uint32_t Address, uint32_t Size, uint8_t* buffer){
	dbg_led_on(RED);
	return MMC_write_bytes(buffer, (uint8_t*)(Address-MMC_ASSIGNED_ADDRESS), Size);
}



int Read (uint32_t Address, uint32_t Size, uint16_t* buffer){
	dbg_led_on(GREEN);
	return MMC_read_bytes((uint8_t*)buffer, (uint8_t*)(Address-MMC_ASSIGNED_ADDRESS), Size);
}

int SectorErase (uint32_t StartAddress, uint32_t EndAddress){
	dbg_led_on(GREEN);
	return 1;
}


/**
  * Description :
  * Calculates checksum value of the memory zone
  * Inputs    :
  *      StartAddress  : Flash start address
  *      Size          : Size (in WORD)  
  *      InitVal       : Initial CRC value
  * outputs   :
  *     R0             : Checksum value
  * Note: Optional for all types of device
  */
uint32_t CheckSum(uint32_t StartAddress, uint32_t Size, uint32_t InitVal)
{
	StartAddress -= MMC_ASSIGNED_ADDRESS;
	uint8_t buf[512];
	uint32_t block_num = StartAddress >> 9;
	uint32_t shift = (StartAddress & 0x1ff);
	if(shift){
		uint32_t first_read = 512 - shift;
		if(ReadMMCBlock(buf, block_num, 1, 100) != HAL_OK)
			return 0;
		for(uint32_t i=0;i<first_read;i++){
			InitVal += buf[shift+i];
		}
		Size -= first_read;
	}
	uint32_t whole_blocks = Size>>9;

	for(uint32_t i=0;i<whole_blocks;i++){
		if(ReadMMCBlock(buf, block_num, 1, 100) != HAL_OK)
			return 0;
		for(uint32_t i=0;i<512;i++){
			InitVal += buf[i];
		}
	}
	uint32_t last_read = Size & 0x1ff;
	if(last_read){
		if(ReadMMCBlock(buf, block_num, 1, 100) != HAL_OK)
			return 0;
		for(uint32_t i=0;i<last_read;i++){
			InitVal += buf[i];
		}
	}
	return InitVal;
}



/**
  * Description :
  * Verify flash memory with RAM buffer and calculates checksum value of
  * the programmed memory
  * Inputs    :
  *      FlashAddr     : Flash address
  *      RAMBufferAddr : RAM buffer address
  *      Size          : Size (in WORD)  
  *      InitVal       : Initial CRC value
  * outputs   :
  *     R0             : Operation failed (address of failure)
  *     R1             : Checksum value
  * Note: Optional for all types of device
  */
//KeepInCompilation uint64_t Verify (uint32_t MemoryAddr, uint32_t RAMBufferAddr, uint32_t Size, uint32_t InitVal)
KeepInCompilation uint64_t Verify (uint32_t MemoryAddr, uint32_t RAMBufferAddr, uint32_t Size)
{
	uint32_t InitVal = 0;
	uint32_t cur_add = 0;
	uint8_t* ram_buf = (uint8_t*) RAMBufferAddr;
	MemoryAddr -= MMC_ASSIGNED_ADDRESS;
	uint8_t buf[512];
	uint32_t block_num = MemoryAddr >> 9;
	uint32_t shift = (MemoryAddr & 0x1ff);
	if(shift){
		uint32_t first_read = 512 - shift;
		if(ReadMMCBlock(buf, block_num, 1, 100) != HAL_OK)
			return ((((uint64_t)InitVal)<<32) | (cur_add+MMC_ASSIGNED_ADDRESS));
		for(uint32_t i=0;i<first_read;i++){
			InitVal += buf[shift+i];
			if(ram_buf[cur_add] != buf[cur_add]){
				return ((((uint64_t)InitVal)<<32) | (cur_add+MMC_ASSIGNED_ADDRESS));
			}else{
				cur_add++;
			}
		}
		Size -= first_read;
	}
	uint32_t whole_blocks = Size>>9;

	for(uint32_t i=0;i<whole_blocks;i++){
		if(ReadMMCBlock(buf, block_num, 1, 100) != HAL_OK)
			return ((((uint64_t)InitVal)<<32) | (cur_add+MMC_ASSIGNED_ADDRESS));
		for(uint32_t i=0;i<512;i++){
			InitVal += buf[i];
			if(ram_buf[cur_add] != buf[cur_add]){
				return ((((uint64_t)InitVal)<<32) | (cur_add+MMC_ASSIGNED_ADDRESS));
			}else{
				cur_add++;
			}
		}
	}
	uint32_t last_read = Size & 0x1ff;
	if(last_read){
		if(ReadMMCBlock(buf, block_num, 1, 100) != HAL_OK)
			return ((((uint64_t)InitVal)<<32) | (cur_add+MMC_ASSIGNED_ADDRESS));
		for(uint32_t i=0;i<last_read;i++){
			InitVal += buf[i];
			if(ram_buf[cur_add] != buf[cur_add]){
				return ((((uint64_t)InitVal)<<32) | (cur_add+MMC_ASSIGNED_ADDRESS));
			}else{
				cur_add++;
			}
		}
	}
	return (((uint64_t)InitVal)<<32);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Init_Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Init_Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC2;
  PeriphClkInitStruct.Sdmmc2ClockSelection = RCC_SDMMC2CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Init_Error_Handler();
  }
}

void SystemClock_Config2(void)
{
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	  /** Configure LSE Drive Capability
	  */
	  HAL_PWR_EnableBkUpAccess();
	  /** Configure the main internal regulator output voltage
	  */
	  __HAL_RCC_PWR_CLK_ENABLE();
	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	  /** Initializes the CPU, AHB and APB busses clocks
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
	                              |RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PLLM = 25;
	  RCC_OscInitStruct.PLL.PLLN = 432;
	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	  RCC_OscInitStruct.PLL.PLLQ = 4;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Init_Error_Handler();
	  }
	  /** Activate the Over-Drive mode
	  */
	  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	  {
	    Init_Error_Handler();
	  }
	  /** Initializes the CPU, AHB and APB busses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
	  {
	    Init_Error_Handler();
	  }
	  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC2|RCC_PERIPHCLK_CLK48;
	  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
	  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
	  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 3;
	  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
	  PeriphClkInitStruct.PLLSAIDivQ = 1;
	  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
	  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
	  PeriphClkInitStruct.Sdmmc2ClockSelection = RCC_SDMMC2CLKSOURCE_CLK48;
	  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	  {
	    Init_Error_Handler();
	  }
}

void MMC_GPIO_INIT(){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_SDMMC2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_SDMMC2;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_SDMMC2;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

int MMC_SDMMC_Init(void)
{
	memset(&hmmc, 0, sizeof(hmmc));
	hmmc.Instance = SDMMC2;
	hmmc.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
	hmmc.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
	hmmc.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
	hmmc.Init.BusWide = SDMMC_BUS_WIDE_1B;
	hmmc.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
	hmmc.Init.ClockDiv = 31;
	__HAL_RCC_SDMMC2_CLK_ENABLE();
	if (HAL_MMC_Init(&hmmc) != HAL_OK)
	{
		return 0;
	}
	if (HAL_MMC_ConfigWideBusOperation(&hmmc, SDMMC_BUS_WIDE_4B) != HAL_OK)
	{
		return 0;
	}
	return 1;
}

HAL_StatusTypeDef ReadMMCBlock(uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks, uint32_t Timeout){
	if(NumberOfBlocks > 0){
		uint32_t tmp = 0;
		uint32_t iteration = 0;
		HAL_StatusTypeDef result;
		tmp = HAL_MMC_GetCardState(&hmmc);
		while(tmp == HAL_MMC_CARD_PROGRAMMING){
			HAL_Delay(DELAY);
			tmp = HAL_MMC_GetCardState(&hmmc);
			if(++iteration>3) return HAL_ERROR;
		};
		result = HAL_MMC_ReadBlocks(&hmmc, pData, BlockAdd, NumberOfBlocks, Timeout);
		return result;
	}else{
		return HAL_OK;
	}
}

HAL_StatusTypeDef WriteMMCBlock(uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks, uint32_t Timeout){
	if(NumberOfBlocks > 0){
		uint32_t tmp = 0;
		uint32_t iteration = 0;
		HAL_StatusTypeDef result;
		tmp = HAL_MMC_GetCardState(&hmmc);
		while(tmp == HAL_MMC_CARD_PROGRAMMING){
			HAL_Delay(DELAY);
			tmp = HAL_MMC_GetCardState(&hmmc);
			if(++iteration>3) return HAL_ERROR;
		};
		result = HAL_MMC_WriteBlocks(&hmmc, pData, BlockAdd, NumberOfBlocks, Timeout);
		return result;
	}else{
		return HAL_OK;
	}
}

int MMC_INIT(){
	MMC_GPIO_INIT();
	return MMC_SDMMC_Init();
}

int MMC_read_bytes(uint8_t* dst, uint8_t* address, uint32_t size){
	uint8_t buf[512];
	uint32_t bytes_read;
	uint32_t block_num = ((uint32_t)address)>>9;
	uint32_t misalignment = ((uint32_t)address) & 0x1FF;
	if(misalignment){
		uint32_t byte_shift = (((uint32_t)address) & 0x1ff);
		bytes_read = 512-byte_shift;
		if(ReadMMCBlock(buf, block_num, 1, 100) != HAL_OK)
			return 0;
		memcpy(dst, &buf[byte_shift], bytes_read);
		block_num++;
	}else{
		bytes_read = 0;
	}
	{
		uint32_t whole_blocks = ((size - bytes_read)>>9);
		if(ReadMMCBlock(&dst[bytes_read], block_num, whole_blocks, 1000) != HAL_OK)
			return 0;
		uint32_t bytes_left = (size - bytes_read) & 0x1ff;
		if(bytes_left){
			if(ReadMMCBlock(buf, block_num + whole_blocks, 1, 100) != HAL_OK){
				return 0;
			}
			memcpy(&dst[size-bytes_left], buf, bytes_left);
		}
	}
	return 1;
}

int MMC_write_bytes(uint8_t* src, uint8_t* address, uint32_t size){
	uint8_t buf[512];
	uint32_t bytes_written;
	uint32_t block_num = ((uint32_t)address)>>9;
	if(((uint32_t)address) & 0x1FF){
		uint32_t byte_shift = (((uint32_t)address) & 0x1ff);
		bytes_written = 512 - byte_shift;
		if(ReadMMCBlock(buf, block_num, 1, 100) != HAL_OK)
			return 0;
		memcpy(&buf[byte_shift], src, bytes_written);
		if(WriteMMCBlock(buf, block_num, 1, 100) != HAL_OK)
			return 0;
		block_num++;
	}else{
		bytes_written = 0;
	}
	{
		uint32_t whole_blocks = ((size - bytes_written)>>9);
		if(WriteMMCBlock(&src[bytes_written], block_num, whole_blocks, 1000) != HAL_OK)
			return 0;
		uint32_t bytes_left = (size - bytes_written) & 0x1ff;
		if(bytes_left){
			if(ReadMMCBlock(buf, block_num, 1, 100) != HAL_OK)
				return 0;
			memcpy(buf, &src[size-bytes_left], bytes_left);
			if(WriteMMCBlock(buf, block_num + whole_blocks, 1, 100) != HAL_OK){
				return 0;
			}
		}
	}
	return 1;
}


void Init_Error_Handler(void)
{
	while(1){
	}
}

void HAL_Delay(uint32_t Delay)
{
  for(int i=0;i<Delay*580;i++);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
