/**
  ******************************************************************************
  * @file    stm32746g_discovery_qspi.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32746g_discovery_qspi.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#pragma once

#include "stm32f7xx_hal.h"


/* QSPI Error codes */
#define QSPI_OK            ((uint8_t)0x00)
#define QSPI_ERROR         ((uint8_t)0x01)
#define QSPI_BUSY          ((uint8_t)0x02)
#define QSPI_NOT_SUPPORTED ((uint8_t)0x04)
#define QSPI_SUSPENDED     ((uint8_t)0x08)


/* QSPI Info */
typedef struct {
    uint32_t FlashSize;          /*!< Size of the flash */
    uint32_t EraseSectorSize;    /*!< Size of sectors for the erase operation */
    uint32_t EraseSectorsNumber; /*!< Number of sectors for the erase operation */
    uint32_t ProgPageSize;       /*!< Size of pages for the program operation */
    uint32_t ProgPagesNumber;    /*!< Number of pages for the program operation */
} QSPI_Info;

typedef struct {
    QSPI_HandleTypeDef *hqspi;
    DMA_HandleTypeDef *hdma_quadspi;

    QSPI_Info flash_info;
} QSPI_FLASH_t;


uint8_t QSPI_Init(QSPI_HandleTypeDef *hqspi, DMA_HandleTypeDef *hdma_quadspi);

uint8_t QSPI_ResetMemory(QSPI_HandleTypeDef *hqspi);

uint8_t QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi);

uint8_t QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi, uint32_t Timeout);

uint8_t QSPI_Read(QSPI_HandleTypeDef *hqspi, uint8_t *pData, uint32_t ReadAddr, uint32_t Size);

uint8_t QSPI_Write(QSPI_HandleTypeDef *hqspi, uint8_t *pData, uint32_t WriteAddr, uint32_t Size);

uint8_t QSPI_Erase_Block(QSPI_HandleTypeDef *hqspi, uint32_t BlockAddress);

uint8_t QSPI_Erase_Chip(QSPI_HandleTypeDef *hqspi);

uint8_t QSPI_GetStatus(QSPI_HandleTypeDef *hqspi);

uint8_t QSPI_GetInfo(QSPI_HandleTypeDef *hqspi, QSPI_Info *pInfo);

uint8_t QSPI_EnableMemoryMappedMode(QSPI_HandleTypeDef *hqspi);


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
