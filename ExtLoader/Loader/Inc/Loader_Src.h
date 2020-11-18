/**
  ******************************************************************************
  * @file    Loader_Src.h
  * @author  MCD Tools Team
  * @date    October-2015
  * @brief   Loader Header file.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOADER_SRC_H
#define __LOADER_SRC_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/


#if defined(__CC_ARM)
extern uint32_t Load$$QSPI$$Base;
extern uint32_t Load$$QSPI$$Length;
#elif defined(__ICCARM__)
#pragma section =".qspi"
#pragma section =".qspi_init"
#elif defined(__GNUC__)
extern uint32_t _qspi_init_base;
extern uint32_t _qspi_init_length;
#endif


#ifdef __ICCARM__                //IAR
#define KeepInCompilation __root 
#elif __CC_ARM                   //MDK-ARM
#define KeepInCompilation __attribute__((used))
#else //TASKING               //TrueStudio
#define KeepInCompilation __attribute__((used))
#endif


#define StartRamAddress          0x20000000
#define EndRamAddress            0x20080000


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

//All system initialisation
int Init ();

//QSPI operation functions
KeepInCompilation int Write(uint32_t Address, uint32_t Size, uint8_t* buffer);
KeepInCompilation int Read(uint32_t Address, uint32_t Size, uint16_t* buffer);
//KeepInCompilation uint64_t Verify (uint32_t MemoryAddr, uint32_t RAMBufferAddr, uint32_t Size, uint32_t InitVal);
KeepInCompilation uint64_t Verify (uint32_t MemoryAddr, uint32_t RAMBufferAddr, uint32_t Size);
uint32_t CheckSum(uint32_t StartAddress, uint32_t Size, uint32_t InitVal);

typedef enum{
	GREEN,
	RED,
}led_colour_t;

void dbg_led_on(led_colour_t led_colour);
void dbg_led_off(led_colour_t led_colour);


#endif /* __LOADER_SRC_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
