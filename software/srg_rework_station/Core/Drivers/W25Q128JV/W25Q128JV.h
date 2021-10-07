#pragma once

/*
Winbond W25Q128JV
3V 128M-BIT
SERIAL FLASH MEMORY WITH
DUAL/QUAD SPI

*/

/*
const uint16_t data_qspi[] __attribute__((section(".data_qspi")));
const uint16_t data_qspi[]= {  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF }

//    // Copy picture from QSPI to RAM
//    memset(buf, 0, 100*sizeof(uint16_t));
//    memcpy(buf, (void*) &data_qspi, 100*sizeof(uint16_t));

 QSPI clock = AHB clock / (ClockPrescaler + 1)
 ClockPrescaler set to 1, so QSPI clock = 216MHz / (1+1) = 108MHz
 ClockPrescaler set to 2, so QSPI clock = 216MHz / (2+1) = 72MHz

 /CS Active Hold Time relative to CLK tCHSH 3 ns
 ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE or QSPI_CS_HIGH_TIME_2_CYCLE ???

 Sample shifting half cycle disabled – may be needed with bad PCB design
 SampleShifting = QSPI_SAMPLE_SHIFTING_NONE or QSPI_SAMPLE_SHIFTING_HALFCYCLE ???

 */

#define W25Q128JV_FLASH_SIZE        0x1000000 // 128 MBits => 16MBytes
#define W25Q128JV_FLASH_SIZE_BIT    23 // 128 MBits => 16MBytes => 16777216bytes => 2 ^(23 + 1), so FlashSize = 23
#define W25Q128JV_SECTOR_SIZE       0x10000 // 256 sectors of 64KBytes
#define W25Q128JV_SUBSECTOR_SIZE    0x1000  // 4096 subsectors of 4kBytes
#define W25Q128JV_PAGE_SIZE         0x100   // 65536 pages of 256 bytes

#define W25Q128JV_DUMMY_CYCLES_FAST_READ 8

#define W25Q128JV_SECTOR_ERASE_MAX_TIME             3
#define W25Q128JV_SUBSECTOR_ERASE_4KB_MAX_TIME      400
#define W25Q128JV_SUBSECTOR_ERASE_32KB_MAX_TIME     1600
#define W25Q128JV_SUBSECTOR_ERASE_64KB_MAX_TIME     2000
#define W25Q128JV_CHIP_ERASE_MAX_TIME               200000



// Reset Operations
#define POWER_DOWN_CMD              0xb9
#define RESET_ENABLE_CMD            0x66
#define RESET_MEMORY_CMD            0x99

// Identification Operations
#define READ_UNIQUE_ID              0x4B
#define READ_ID_CMD                 0x90
#define DUAL_READ_ID_CMD            0x92
#define QUAD_READ_ID_CMD            0x94
#define READ_JEDEC_ID_CMD           0x9F

// Read Operations
#define READ_CMD                    0x03 //  only supported in Standard SPI mode
#define FAST_READ_CMD               0x0B
#define FAST_READ_DUAL_OUT_CMD      0x3B
#define FAST_READ_QUAD_OUT_CMD      0x6B

#define FAST_READ_DUAL_INOUT_CMD    0xBB
#define FAST_READ_QUAD_INOUT_CMD    0xEB

#define BURST_WITH_WRAP_CMD         0x77

// Write Operations
#define WRITE_ENABLE_CMD            0x06
#define WRITE_DISABLE_CMD           0x04
#define WRITE_ENABLE_VOL_SR_CMD     0x50

// Register Operations
#define READ_STATUS_REG1_CMD        0x05
#define READ_STATUS_REG2_CMD        0x35
#define READ_STATUS_REG3_CMD        0x15

#define WRITE_STATUS_REG1_CMD       0x01
#define WRITE_STATUS_REG2_CMD       0x31
#define WRITE_STATUS_REG3_CMD       0x11


// Program Operations
#define PAGE_PROG_CMD               0x02
#define QUAD_INPUT_PAGE_PROG_CMD    0x32


// Erase Operations
#define SECTOR_ERASE_4KB_CMD        0x20
#define SECTOR_ERASE_32KB_CMD       0x52
#define SECTOR_ERASE_64KB_CMD       0xD8

#define SECTOR_ERASE_CMD            SECTOR_ERASE_4KB_CMD
#define CHIP_ERASE_CMD              0xC7

#define PROG_ERASE_RESUME_CMD       0x7A
#define PROG_ERASE_SUSPEND_CMD      0x75

#define GLOBAL_BLOCK_LOCK_CMD       0x7E
#define GLOBAL_BLOCK_UNLOCK_CMD     0x98
#define READ_BLOCK_LOCK_CMD         0x3d
#define INDIV_BLOCK_LOCK_CMD        0x36
#define INDIV_BLOCK_UNLOCK_CMD      0x39

// Security register Operations
#define ERASE_SECURITY_REG_CMD      0x44
#define PROG_SECURITY_REG_CMD       0x42
#define READ_SECURITY_REG_CMD       0x48

// Flag Status Register-1
#define W25Q128JV_FSR1_BUSY     ((uint8_t)0x01)     // busy, write in progress
#define W25Q128JV_FSR1_WEL      ((uint8_t)0x02)     // write enable latch
#define W25Q128JV_FSR1_BP0      ((uint8_t)0x04)     // block protect bit 0
#define W25Q128JV_FSR1_BP1      ((uint8_t)0x08)     // block protect bit 1
#define W25Q128JV_FSR1_BP2      ((uint8_t)0x10)     // block protect bit 2
#define W25Q128JV_FSR1_TB       ((uint8_t)0x20)     // top/bottom protect
#define W25Q128JV_FSR1_SEC      ((uint8_t)0x40)     // sector protect
#define W25Q128JV_FSR1_SRP      ((uint8_t)0x80)     // status register protect

// Flag Status Register-2
#define W25Q128JV_FSR2_SRL      ((uint8_t)0x01)     // status register lock
#define W25Q128JV_FSR2_QE       ((uint8_t)0x02)     // quad enable
//#define W25Q128JV_FSR2_R      ((uint8_t)0x04)     // reserved
#define W25Q128JV_FSR2_LB1      ((uint8_t)0x08)     // secure register lock bit 0
#define W25Q128JV_FSR2_LB2      ((uint8_t)0x10)     // secure register lock bit 1
#define W25Q128JV_FSR2_LB3      ((uint8_t)0x20)     // secure register lock bit 2
#define W25Q128JV_FSR2_CMP      ((uint8_t)0x40)     // complement protect
#define W25Q128JV_FSR2_SUS      ((uint8_t)0x80)     // suspend status

// Flag Status Register-3
//#define W25Q128JV_FSR3_R      ((uint8_t)0x01)     // reserved
//#define W25Q128JV_FSR3_R      ((uint8_t)0x02)     // reserved
#define W25Q128JV_FSR3_WPS      ((uint8_t)0x04)     // write protect selection
//#define W25Q128JV_FSR3_R      ((uint8_t)0x08)     // reserved
//#define W25Q128JV_FSR3_R      ((uint8_t)0x10)     // reserved
#define W25Q128JV_FSR3_DRV0     ((uint8_t)0x20)     // output driver strength bit 0
#define W25Q128JV_FSR3_DRV1     ((uint8_t)0x40)     // output driver strength bit 1
#define W25Q128JV_FSR3_RST      ((uint8_t)0x80)     // reset function


/*****************************************************************************************************/





