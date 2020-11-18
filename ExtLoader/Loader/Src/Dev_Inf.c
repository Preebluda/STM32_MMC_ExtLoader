/**
  ******************************************************************************
  * @file    Dev_Inf.c
  * @author  MCD Tools Team
  * @date    October-2015
  * @brief   This file defines the structure containing informations about the 
  *          external flash memory N25Q256QA used by ST-LINK Utility in 
  *          programming/erasing the device.
  ******************************************************************************
  */
#include "Dev_Inf.h"

/* This structure containes information used by ST-LINK Utility to program and erase the device */
#if defined (__ICCARM__)
__root struct StorageInfo const StorageInfo  =  {
#else
struct StorageInfo const StorageInfo  =  {
#endif
   "MMC_STM32F769I-DISCO", 					                        // Device Name + version number
   SRAM,                                                           // Device Type
   MMC_ASSIGNED_ADDRESS,                                                          // Device Start Address
   0x2000000,                 						// Device Size in Bytes (32MBytes)
   0x200,                 						// Programming Page Size 512Bytes
   0x00,                       						// Initial Content of Erased Memory
// Specify Size and Address of Sectors (view example below)
   0x10000, 0x000200,     				 		// Sector Num : 1 ,Sector Size: 8MBytes
   0x00000000, 0x00000000,
};


