
#ifndef __MCP23017_H
#define __MCP23017_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include "i2c.h"
#include "main.h"
// I2C
#define MCP23017_I2C_PORT &hi2c2
// I2C address
#define MCP23017_I2C_ADDR MCP23017_ADDRESS_20
// I2C timeout
#define MCP23017_I2C_TIMEOUT 10

/***********************/

// Ports
#define MCP23017_PORTA        0x00
#define MCP23017_PORTB        0x01

// Address (A0-A2)
#define MCP23017_ADDRESS_20     0x20
#define MCP23017_ADDRESS_21     0x21
#define MCP23017_ADDRESS_22     0x22
#define MCP23017_ADDRESS_23     0x23
#define MCP23017_ADDRESS_24     0x24
#define MCP23017_ADDRESS_25     0x25
#define MCP23017_ADDRESS_26     0x26
#define MCP23017_ADDRESS_27     0x27

// Registers for IOCON.BANK = 0
#define MCP23017_REG_IODIRA   0x00 // I/O DIRECTION REGISTER
#define MCP23017_REG_IODIRB   0x01 // I/O DIRECTION REGISTER
#define MCP23017_REG_IPOLA    0x02 // INPUT POLARITY PORT REGISTER
#define MCP23017_REG_IPOLB    0x03 // INPUT POLARITY PORT REGISTER
#define MCP23017_REG_GPINTENA 0x04 // INTERRUPT-ON-CHANGE PINS
#define MCP23017_REG_GPINTENB 0x05 // INTERRUPT-ON-CHANGE PINS
#define MCP23017_REG_DEFVALA  0x06 // DEFAULT VALUE REGISTER
#define MCP23017_REG_DEFVALB  0x07 // DEFAULT VALUE REGISTER
#define MCP23017_REG_INTCONA  0x08 // INTERRUPT-ON-CHANGE CONTROL REGISTER
#define MCP23017_REG_INTCONB  0x09 // INTERRUPT-ON-CHANGE CONTROL REGISTER
#define MCP23017_REG_IOCON    0x0A // CONFIGURATION REGISTER
#define MCP23017_REG_GPPUA    0x0C // GPIO PULL-UP RESISTOR REGISTER
#define MCP23017_REG_GPPUB    0x0D // GPIO PULL-UP RESISTOR REGISTER
#define MCP23017_REG_INTFA    0x0E // INTERRUPT FLAG REGISTER read-only
#define MCP23017_REG_INTFB    0x0F // INTERRUPT FLAG REGISTER read-only
#define MCP23017_REG_INTCAPA  0x10 // INTERRUPT CAPTURE REGISTER
#define MCP23017_REG_INTCAPB  0x11 // INTERRUPT CAPTURE REGISTER
#define MCP23017_REG_GPIOA    0x12 // PORT REGISTER
#define MCP23017_REG_GPIOB    0x13 // PORT REGISTER
#define MCP23017_REG_OLATA    0x14 // OUTPUT LATCH REGISTER
#define MCP23017_REG_OLATB    0x15 // OUTPUT LATCH REGISTER

/*
INTPOL: This bit sets the polarity of the INT output pin.This bit is functional only when the ODR bit is cleared
1 = Active-high.
0 = Active-low.
 */
#define MCP23017_BIT_IOCON_INTPOL 1
/*
ODR: This bit configures the INT pin as an open-drain output.
1 = Open-drain output (overrides the INTPOL bit).
0 = Active driver output (INTPOL bit sets the polarity).
 */
#define MCP23017_BIT_IOCON_ODR    2
/*
HAEN: Hardware Address Enable bit (MCP23S17 only).
Address pins are always enabled on MCP23017.
1 = Enables the MCP23S17 address pins.
0 = Disables the MCP23S17 address pins.
 */
#define MCP23017_BIT_IOCON_HAEN   3
/*
DISSLW: Slew Rate control bit for SDA output.
1 = Slew rate disabled .
0 = Slew rate enabled
 */
#define MCP23017_BIT_IOCON_DISSLW 4
/*
SEQOP: Sequential Operation mode bit.
1 = Sequential operation disabled, address pointer does not increment.
0 = Sequential operation enabled, address pointer increments.
 */
#define MCP23017_BIT_IOCON_SEQOP  5
/*
MIRROR: INT Pins Mirror bit
1 = The INT pins are internally connected
0 = The INT pins are not connected. INTA is associated with PortA and INTB is associated with PortB
 */
#define MCP23017_BIT_IOCON_MIRROR 6
/*
BANK: Controls how the registers are addressed
1 = The registers associated with each port are separated into different banks PORTA:00h-0Ah, PORTB:10h-1Ah
0 = The registers are in the same bank (addresses are sequential) 00h-15h, IODIRA:0h, IODIRB:1h..
 */
#define MCP23017_BIT_IOCON_BANK   7


// Port mask
#define MCP23017_PORT_IO0    0x01
#define MCP23017_PORT_IO1    0x02
#define MCP23017_PORT_IO2    0x04
#define MCP23017_PORT_IO3    0x08
#define MCP23017_PORT_IO4    0x10
#define MCP23017_PORT_IO5    0x20
#define MCP23017_PORT_IO6    0x40
#define MCP23017_PORT_IO7    0x80


// I/O Direction
// Default state: MCP23017_IODIR_ALL_INPUT
#define MCP23017_IODIR_ALL_INPUT    0xFF
#define MCP23017_IODIR_ALL_OUTPUT   0x00

// Input Polarity
// Default state: MCP23017_IPOL_ALL_NORMAL
#define MCP23017_IPOL_ALL_INVERTED  0xFF
#define MCP23017_IPOL_ALL_NORMAL    0x00



// Pull-Up Resistor
// Default state: MCP23017_GPPU_ALL_DISABLED
/*
PU7:PU0: These bits control the weak pull-up resistors on each pin (when configured as an input)
<7:0>.
1 = Pull-up enabled.
0 = Pull-up disabled.
 */
#define MCP23017_GPPU_ALL_ENABLED   0xFF
#define MCP23017_GPPU_ALL_DISABLED  0x00

/*
GPINTEN  INTERRUPT FLAG REGISTER
INT7:INT0: These bits reflect the interrupt condition on the port. Will reflect the change only if interrupts
are enabled (GPINTEN) <7:0>.
1 = Pin caused interrupt.
0 = Interrupt not pending.
 */
#define MCP23017_GPINTEN_ALL_INT  0xFF
#define MCP23017_GPINTEN_ALL_NOINT  0x00

/*
INTCON INTERRUPT CONTROL REGISTER
IOC7:IOC0: These bits control how the associated pin value is compared for interrupt-on-change
<7:0>
1 = the corresponding I/O pin is compared against the associated bit in the DEFVAL register.
0 = Pin value is compared against the previous pin value.
 */
#define MCP23017_INTCON_ALL_DEFVAL  0xFF
#define MCP23017_INTCON_ALL_PREVVAL  0x00

/*
GPIO PORT REGISTER
GP7:GP0: These bits reflect the logic level on the pins <7:0>
1 = Logic-high.
0 = Logic-low.
 */
#define MCP23017_GPIO_ALL_HIGH 0xFF
#define MCP23017_GPIO_ALL_LOW 0x00

/*
INTCAP  INTERRUPT CAPTURE REGISTER
ICP7:ICP0: These bits reflect the logic level on the port pins at the time of interrupt due to pin change
<7:0>
1 = Logic-high.
0 = Logic-low.
 */


/*
OLAT OUTPUT LATCH REGISTER
OL7:OL0: These bits reflect the logic level on the output latch <7:0>
1 = Logic-high.
0 = Logic-low.
 */

/*
INTF – INTERRUPT FLAG REGISTER  read-only
INT7:INT0: These bits reflect the interrupt condition on the port. Will reflect the change only if interrupts
are enabled (GPINTEN) <7:0>.
1 = Pin caused interrupt.
0 = Interrupt not pending.

 */


HAL_StatusTypeDef mcp23017_init(void);


#ifdef __cplusplus
}
#endif

#endif /* __MCP23017_H */
