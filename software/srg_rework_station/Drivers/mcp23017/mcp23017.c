#include "mcp23017.h"

HAL_StatusTypeDef mcp23017_init(void)
{
  // IntA - Active-low
  mcp23017_write(MCP23017_REG_IOCON,
      MCP23017_BIT_IOCON_SEQOP // Sequential operation disabled
  );
  mcp23017_write(MCP23017_REG_IODIRA, MCP23017_IODIR_ALL_INPUT); // PORTA all input
  mcp23017_write(MCP23017_REG_IODIRB, MCP23017_IODIR_ALL_OUTPUT); // PORTB all output

  mcp23017_write(MCP23017_REG_IPOLA, MCP23017_IPOL_ALL_NORMAL); // PORTA default

  mcp23017_write(MCP23017_REG_GPINTENA, MCP23017_GPINTEN_ALL_INT); // PORTA enable interrupt on change
  mcp23017_write(MCP23017_REG_INTCONA, MCP23017_INTCON_ALL_PREVVAL); // PORTA, 0 = Pin value is compared against the previous pin value.

  mcp23017_write(MCP23017_REG_GPPUA, MCP23017_GPPU_ALL_ENABLED); // PORTA all pull up

  mcp23017_write(MCP23017_REG_GPIOB,
      MCP23017_PORT_IO0 // pin 0 high, other low
  ); // PORTB

  return HAL_OK;
}

HAL_StatusTypeDef mcp23017_read(uint16_t reg, uint8_t *data)
{
  return HAL_I2C_Mem_Read(MCP23017_I2C_PORT, MCP23017_I2C_ADDR, reg, 1, data, 1, MCP23017_I2C_TIMEOUT);
}

HAL_StatusTypeDef mcp23017_write(uint16_t reg, uint8_t data)
{
  return HAL_I2C_Mem_Write(MCP23017_I2C_PORT, MCP23017_I2C_ADDR, reg, 1, &data, 1, MCP23017_I2C_TIMEOUT);
}

