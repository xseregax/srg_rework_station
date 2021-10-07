/**
  ******************************************************************************
  * @author  MCD Application Team
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


// Error codes
#define FLASH_OK            ((uint8_t)0x00)
#define FLASH_ERROR         ((uint8_t)0x01)
#define FLASH_BUSY          ((uint8_t)0x02)
#define FLASH_NOT_SUPPORTED ((uint8_t)0x04)
#define FLASH_SUSPENDED     ((uint8_t)0x08)

typedef struct {
    uint32_t FlashSize;          /*!< Size of the flash */
    uint32_t EraseSectorSize;    /*!< Size of sectors for the erase operation */
    uint32_t EraseSectorsNumber; /*!< Number of sectors for the erase operation */
    uint32_t ProgPageSize;       /*!< Size of pages for the program operation */
    uint32_t ProgPagesNumber;    /*!< Number of pages for the program operation */
} FLASH_INFO_t;

typedef struct {
    QSPI_HandleTypeDef *hqspi;
    DMA_HandleTypeDef *hdma_quadspi;

    FLASH_INFO_t flash_info;
} FLASH_t;


uint8_t FLASH_Init(QSPI_HandleTypeDef *hqspi, DMA_HandleTypeDef *hdma_quadspi);

uint8_t FLASH_ResetMemory(void);

uint8_t FLASH_WriteEnable(void);

uint8_t FLASH_AutoPollingMemReady(uint32_t Timeout);

uint8_t FLASH_Read(uint8_t *pData, uint32_t ReadAddr, uint32_t Size);

uint8_t FLASH_Write(uint8_t *pData, uint32_t WriteAddr, uint32_t Size);

uint8_t FLASH_Erase_Block(uint32_t BlockAddress);

uint8_t FLASH_Erase_Chip(void);

uint8_t FLASH_GetStatus(void);

uint8_t FLASH_GetInfo(FLASH_INFO_t *pInfo);

uint8_t FLASH_EnableMemoryMappedMode(void);

