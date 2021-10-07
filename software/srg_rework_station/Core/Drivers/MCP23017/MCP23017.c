#include "MCP23017.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "semphr.h"

MCP23017 mcp = {0};

const osMutexAttr_t myMutexI2cInputs_attributes = {
        .name = "myMutexI2cInputs"
};

void MCP23017_HW_Init(void);

HAL_StatusTypeDef MCP23017_Read(uint16_t reg, uint8_t *data);

HAL_StatusTypeDef MCP23017_Write(uint16_t reg, uint8_t data);


HAL_StatusTypeDef MCP23017_Init(I2C_HandleTypeDef *i2cHandle) {

    mcp.i2cHandle = i2cHandle;
    mcp.myMutexI2cInputsHandle = osMutexNew(&myMutexI2cInputs_attributes);

    MCP23017_HW_Init();

    return HAL_OK;
}

void MCP23017_HW_Init(void)
{
    xSemaphoreTake(mcp.myMutexI2cInputsHandle, portMAX_DELAY);

    // IntA - Active-low
    MCP23017_Write(MCP23017_REG_IOCON,
                   MCP23017_BIT_IOCON_SEQOP // Sequential operation disabled
    );
    MCP23017_Write(MCP23017_REG_IODIRA, MCP23017_IODIR_ALL_INPUT); // PORTA all input
    MCP23017_Write(MCP23017_REG_IODIRB, MCP23017_IODIR_ALL_OUTPUT); // PORTB all output

    MCP23017_Write(MCP23017_REG_IPOLA, MCP23017_IPOL_ALL_NORMAL); // PORTA default

    MCP23017_Write(MCP23017_REG_GPINTENA, MCP23017_GPINTEN_ALL_INT); // PORTA enable interrupt on change
    MCP23017_Write(MCP23017_REG_INTCONA,
                   MCP23017_INTCON_ALL_PREVVAL); // PORTA, 0 = Pin value is compared against the previous pin value.

    MCP23017_Write(MCP23017_REG_GPPUA, MCP23017_GPPU_ALL_ENABLED); // PORTA all pull up

    MCP23017_Write(MCP23017_REG_GPIOB,
                   MCP23017_PORT_IO0 // pin 0 high, other low
    ); // PORTB

    xSemaphoreGive(mcp.myMutexI2cInputsHandle);
}

HAL_StatusTypeDef MCP23017_Read(uint16_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Read(mcp.i2cHandle, MCP23017_ADDRESS_20, reg, 1, data, 1, MCP23017_I2C_TIMEOUT);
}

HAL_StatusTypeDef MCP23017_Write(uint16_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(mcp.i2cHandle, MCP23017_ADDRESS_20, reg, 1, &data, 1, MCP23017_I2C_TIMEOUT);
}

