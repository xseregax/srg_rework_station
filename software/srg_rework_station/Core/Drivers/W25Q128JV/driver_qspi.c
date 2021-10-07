/**
  ******************************************************************************
  * @author  MCD Application Team
   (#) Initialization steps:
       (++) Initialize the QPSI external memory using the FLASH_Init() function. This
            function includes the MSP layer hardware resources initialization and the
            QSPI interface with the external memory.

   (#) QSPI memory operations
       (++) QSPI memory can be accessed with read/write operations once it is
            initialized.
            Read/write operation can be performed with AHB access using the functions
            FLASH_Read()/FLASH_Write().
       (++) The function FLASH_GetInfo() returns the configuration of the QSPI memory.
            (see the QSPI memory data sheet)
       (++) Perform erase block operation using the function FLASH_Erase_Block() and by
            specifying the block address. You can perform an erase operation of the whole
            chip by calling the function FLASH_Erase_Chip().
       (++) The function FLASH_GetStatus() returns the current status of the QSPI memory.
            (see the QSPI memory data sheet)
  ******************************************************************************
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

#include "driver_qspi.h"
#include "W25Q128JV.h"

FLASH_t flash = {0};

void FLASH_Init_Default_Command(QSPI_CommandTypeDef *s_command) {
    s_command->InstructionMode = QSPI_INSTRUCTION_1_LINE;
    s_command->AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    s_command->DdrMode = QSPI_DDR_MODE_DISABLE;
    s_command->DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    s_command->SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    s_command->DummyCycles = 0;

    s_command->AddressMode = QSPI_ADDRESS_NONE;
    s_command->DataMode = QSPI_DATA_NONE;
}

uint8_t FLASH_Init(QSPI_HandleTypeDef *hqspi, DMA_HandleTypeDef *hdma_quadspi) {
    flash.hqspi = hqspi;
    flash.hdma_quadspi = hdma_quadspi;

    /* QSPI memory reset */
    if (FLASH_ResetMemory() != FLASH_OK) {
        return FLASH_NOT_SUPPORTED;
    }

    FLASH_GetInfo(&flash.flash_info);

    //TODO need check correct flash info

    /* Configure the QSPI in memory-mapped mode */
    FLASH_EnableMemoryMappedMode();

    return FLASH_OK;
}

/**
  * @brief  This function reset the QSPI memory.
  * @param  hqspi: QSPI handle
  * @retval None
  */
uint8_t FLASH_ResetMemory(void) {
    QSPI_CommandTypeDef s_command = {0};
    FLASH_Init_Default_Command(&s_command);
    
    /* Initialize the reset enable command */
    s_command.Instruction = RESET_ENABLE_CMD;
    if (HAL_QSPI_Command(flash.hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return FLASH_ERROR;
    }

    /* Send the reset memory command */
    s_command.Instruction = RESET_MEMORY_CMD;
    if (HAL_QSPI_Command(flash.hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return FLASH_ERROR;
    }

    /* Configure automatic polling mode to wait the memory is ready */
    if (FLASH_AutoPollingMemReady(HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != FLASH_OK) {
        return FLASH_ERROR;
    }

    return FLASH_OK;
}

/**
  * @brief  This function send a Write Enable and wait it is effective.
  * @param  hqspi: QSPI handle
  * @retval None
  */
uint8_t FLASH_WriteEnable(void) {
    QSPI_CommandTypeDef s_command = {0};
    FLASH_Init_Default_Command(&s_command);

    QSPI_AutoPollingTypeDef s_config = {0};

    /* Enable write operations */
    s_command.Instruction = WRITE_ENABLE_CMD;
    if (HAL_QSPI_Command(flash.hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return FLASH_ERROR;
    }

    /* Configure automatic polling mode to wait for write enabling */
    s_config.Match = W25Q128JV_FSR1_WEL;
    s_config.Mask = W25Q128JV_FSR1_WEL;
    s_config.MatchMode = QSPI_MATCH_MODE_AND;
    s_config.StatusBytesSize = 1;
    s_config.Interval = 0x10;
    s_config.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

    s_command.Instruction = WRITE_STATUS_REG1_CMD;
    s_command.DataMode = QSPI_DATA_1_LINE;
    if (HAL_QSPI_AutoPolling(flash.hqspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return FLASH_ERROR;
    }

    return FLASH_OK;
}

/**
  * @brief  This function read the SR of the memory and wait the EOP.
  * @param  hqspi: QSPI handle
  * @param  Timeout
  * @retval None
  */
uint8_t FLASH_AutoPollingMemReady(uint32_t Timeout) {
    QSPI_CommandTypeDef s_command = {0};
    FLASH_Init_Default_Command(&s_command);

    QSPI_AutoPollingTypeDef s_config = {0};

    /* Configure automatic polling mode to wait for memory ready */
    s_command.Instruction = READ_STATUS_REG1_CMD;
    s_command.DataMode = QSPI_DATA_1_LINE;

    s_config.Match = 0;
    s_config.Mask = W25Q128JV_FSR1_BUSY;
    s_config.MatchMode = QSPI_MATCH_MODE_AND;
    s_config.StatusBytesSize = 1;
    s_config.Interval = 0x10;
    s_config.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

    if (HAL_QSPI_AutoPolling(flash.hqspi, &s_command, &s_config, Timeout) != HAL_OK) {
        return FLASH_ERROR;
    }

    return FLASH_OK;
}


/**
  * @brief  Reads an amount of data from the QSPI memory.
  * @param  pData: Pointer to data to be read
  * @param  ReadAddr: Read start address
  * @param  Size: Size of data to read
  * @retval QSPI memory status
  */
uint8_t FLASH_Read(uint8_t *pData, uint32_t ReadAddr, uint32_t Size) {
    QSPI_CommandTypeDef s_command = {0};
    FLASH_Init_Default_Command(&s_command);

    /* Initialize the read command */
    s_command.Instruction = FAST_READ_DUAL_OUT_CMD;

    s_command.AddressMode = QSPI_ADDRESS_2_LINES;
    s_command.AddressSize = QSPI_ADDRESS_24_BITS;
    s_command.Address = ReadAddr;
    
    s_command.DataMode = QSPI_DATA_2_LINES;
    s_command.NbData = Size;

    s_command.DummyCycles = W25Q128JV_DUMMY_CYCLES_FAST_READ;

    /* Configure the command */
    if (HAL_QSPI_Command(flash.hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return FLASH_ERROR;
    }

    /* Set S# timing for Read command */
    MODIFY_REG(flash.hqspi->Instance->DCR, QUADSPI_DCR_CSHT, QSPI_CS_HIGH_TIME_3_CYCLE);

    /* Reception of the data */
//    if (HAL_QSPI_Receive_DMA(flash.hqspi, pData) != HAL_OK) {
//        return FLASH_ERROR;
//    }

    if (HAL_QSPI_Receive(flash.hqspi, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return FLASH_ERROR;
    }

    /* Restore S# timing for nonRead commands */
    MODIFY_REG(flash.hqspi->Instance->DCR, QUADSPI_DCR_CSHT, QSPI_CS_HIGH_TIME_6_CYCLE);

    return FLASH_OK;
}

/**
  * @brief  Writes an amount of data to the QSPI memory.
  * @param  pData: Pointer to data to be written
  * @param  WriteAddr: Write start address
  * @param  Size: Size of data to write
  * @retval QSPI memory status
  */
uint8_t FLASH_Write(uint8_t *pData, uint32_t WriteAddr, uint32_t Size) {
    QSPI_CommandTypeDef s_command = {0};
    FLASH_Init_Default_Command(&s_command);

    uint32_t end_addr, current_size, current_addr;

    /* Calculation of the size between the write address and the end of the page */
    current_size = W25Q128JV_PAGE_SIZE - (WriteAddr % W25Q128JV_PAGE_SIZE);

    /* Check if the size of the data is less than the remaining place in the page */
    if (current_size > Size) {
        current_size = Size;
    }

    /* Initialize the adress variables */
    current_addr = WriteAddr;
    end_addr = WriteAddr + Size;

    /* Initialize the program command */
    
    s_command.Instruction = PAGE_PROG_CMD;

    s_command.AddressMode = QSPI_ADDRESS_1_LINE;
    s_command.AddressSize = QSPI_ADDRESS_24_BITS;
    
    s_command.DataMode = QSPI_DATA_1_LINE;

    /* Perform the write page by page */
    do {
        s_command.Address = current_addr;
        s_command.NbData = current_size;

        /* Enable write operations */
        if (FLASH_WriteEnable() != FLASH_OK) {
            return FLASH_ERROR;
        }

        /* Configure the command */
        if (HAL_QSPI_Command(flash.hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            return FLASH_ERROR;
        }

        /* Transmission of the data */
        if (HAL_QSPI_Transmit_DMA(flash.hqspi, pData) != HAL_OK) {
            return FLASH_ERROR;
        }

//        if (HAL_QSPI_Transmit(flash.hqspi, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
//            return FLASH_ERROR;
//        }

        /* Configure automatic polling mode to wait for end of program */
        if (FLASH_AutoPollingMemReady(HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != FLASH_OK) {
            return FLASH_ERROR;
        }

        /* Update the address and size variables for next page programming */
        current_addr += current_size;
        pData += current_size;
        current_size = ((current_addr + W25Q128JV_PAGE_SIZE) > end_addr) ? (end_addr - current_addr)
                                                                        : W25Q128JV_PAGE_SIZE;
    } while (current_addr < end_addr);

    return FLASH_OK;
}

/**
  * @brief  Erases the specified block of the QSPI memory.
  * @param  BlockAddress: Block address to erase
  * @retval QSPI memory status
  */
uint8_t FLASH_Erase_Block(uint32_t BlockAddress) {
    QSPI_CommandTypeDef s_command = {0};
    FLASH_Init_Default_Command(&s_command);

    /* Initialize the erase command */
    
    s_command.Instruction = SECTOR_ERASE_4KB_CMD;

    s_command.AddressMode = QSPI_ADDRESS_1_LINE;
    s_command.AddressSize = QSPI_ADDRESS_24_BITS;

    s_command.Address = BlockAddress;

    /* Enable write operations */
    if (FLASH_WriteEnable() != FLASH_OK) {
        return FLASH_ERROR;
    }

    /* Send the command */
    if (HAL_QSPI_Command(flash.hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return FLASH_ERROR;
    }

    /* Configure automatic polling mode to wait for end of erase */
    if (FLASH_AutoPollingMemReady(W25Q128JV_SUBSECTOR_ERASE_4KB_MAX_TIME) != FLASH_OK) {
        return FLASH_ERROR;
    }

    return FLASH_OK;
}

/**
  * @brief  Erases the entire QSPI memory.
  * @retval QSPI memory status
  */
uint8_t FLASH_Erase_Chip(void) {
    QSPI_CommandTypeDef s_command = {0};
    FLASH_Init_Default_Command(&s_command);

    /* Initialize the erase command */
    s_command.Instruction = CHIP_ERASE_CMD;

    /* Enable write operations */
    if (FLASH_WriteEnable() != FLASH_OK) {
        return FLASH_ERROR;
    }

    /* Send the command */
    if (HAL_QSPI_Command(flash.hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return FLASH_ERROR;
    }

    /* Configure automatic polling mode to wait for end of erase */
    if (FLASH_AutoPollingMemReady(W25Q128JV_CHIP_ERASE_MAX_TIME) != FLASH_OK) {
        return FLASH_ERROR;
    }

    return FLASH_OK;
}

/**
  * @brief  Reads current status of the QSPI memory.
  * @retval QSPI memory status
  */
uint8_t FLASH_GetStatus(void) {
    QSPI_CommandTypeDef s_command = {0};
    FLASH_Init_Default_Command(&s_command);

    uint8_t reg;
    /* Initialize the read flag status register command */
    s_command.Instruction = READ_STATUS_REG1_CMD;

    s_command.DataMode = QSPI_DATA_1_LINE;
    s_command.NbData = 1;

    /* Configure the command */
    if (HAL_QSPI_Command(flash.hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return FLASH_ERROR;
    }

    /* Reception of the data */
    if (HAL_QSPI_Receive(flash.hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return FLASH_ERROR;
    }

    /* Check the value of the register */
    if ((reg & W25Q128JV_FSR1_BUSY) != 0) {
        return FLASH_BUSY;
    } else {
        return FLASH_OK;
    }
}

/**
  * @brief  Return the configuration of the QSPI memory.
  * @param  pInfo: pointer on the configuration structure
  * @retval QSPI memory status
  */
uint8_t FLASH_GetInfo(FLASH_INFO_t *pInfo) {
    /* Configure the structure with the memory configuration */
    pInfo->FlashSize = W25Q128JV_FLASH_SIZE;
    pInfo->EraseSectorSize = W25Q128JV_SUBSECTOR_SIZE;
    pInfo->EraseSectorsNumber = (W25Q128JV_FLASH_SIZE / W25Q128JV_SUBSECTOR_SIZE);
    pInfo->ProgPageSize = W25Q128JV_PAGE_SIZE;
    pInfo->ProgPagesNumber = (W25Q128JV_FLASH_SIZE / W25Q128JV_PAGE_SIZE);

    return FLASH_OK;
}

/**
  * @brief  Configure the QSPI in memory-mapped mode
  * @retval QSPI memory status
  */
uint8_t FLASH_EnableMemoryMappedMode(void) {
    QSPI_CommandTypeDef s_command = {0};
    FLASH_Init_Default_Command(&s_command);

    QSPI_MemoryMappedTypeDef s_mem_mapped_cfg = {0};

    /* Configure the command for the read instruction */
    s_command.Instruction = FAST_READ_DUAL_INOUT_CMD;

    s_command.AddressMode = QSPI_ADDRESS_2_LINES;
    s_command.AddressSize = QSPI_ADDRESS_24_BITS;
    
    s_command.DataMode = QSPI_DATA_2_LINES;

    s_command.DummyCycles = W25Q128JV_DUMMY_CYCLES_FAST_READ;

    /* Configure the memory mapped mode */
    s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
    s_mem_mapped_cfg.TimeOutPeriod = 0;

    if (HAL_QSPI_MemoryMapped(flash.hqspi, &s_command, &s_mem_mapped_cfg) != HAL_OK) {
        return FLASH_ERROR;
    }

    return FLASH_OK;
}
