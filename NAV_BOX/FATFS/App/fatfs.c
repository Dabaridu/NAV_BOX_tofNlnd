/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#include "fatfs.h"

uint8_t retUSER;    /* Return value for USER */
char USERPath[4];   /* USER logical drive path */
FATFS USERFatFS;    /* File system object for USER logical drive */
FIL USERFile;       /* File object for USER */

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the USER driver ###########################*/
  retUSER = FATFS_LinkDriver(&USER_Driver, USERPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  /* 
     Generates a FAT timestamp from system ticks.
     FAT timestamp format: [31:25] Year(0-127, 1980-2107) [24:21] Month(1-12) [20:16] Day(1-31)
                         [15:11] Hour(0-23) [10:5] Minute(0-59) [4:0] Second/2(0-29)
  */
  uint32_t ticks = HAL_GetTick();
  
  // Simple counter-based approach: increment date every ~24 hours of uptime
  uint32_t days = ticks / 86400000;  // milliseconds per day
  
  // Start from 2025-01-01 (FAT year offset from 1980)
  uint32_t year = 45;   // 2025 - 1980 = 45
  uint32_t month = 1;   // January
  uint32_t day = 1 + (days % 28);  // Limit to 28 days per month for simplicity
  if (day > 28) day = 28;
  
  // Extract time from ticks (hh:mm:ss)
  uint32_t remaining = ticks % 86400000;
  uint32_t hour = (remaining / 3600000) % 24;
  uint32_t minute = (remaining / 60000) % 60;
  uint32_t second = (remaining / 1000) % 60;
  
  // Build FAT timestamp
  DWORD fattime = ((year & 0x7F) << 25) |
                  ((month & 0x0F) << 21) |
                  ((day & 0x1F) << 16) |
                  ((hour & 0x1F) << 11) |
                  ((minute & 0x3F) << 5) |
                  ((second / 2) & 0x1F);
  
  return fattime;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */
