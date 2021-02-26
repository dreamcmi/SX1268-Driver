/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: SX126x driver specific target board functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/

#include "stm32l1xx_hal.h"
#include "SPI.h"
#include "radio.h"
#include "sx126x.h"
#include "sx126x-board.h"
void SX126xReset(void)
{
}

void SX126xWaitOnBusy(void)
{
}

void SX126xWakeup(void)
{
}

void SX126xWriteCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size)
{
}

void SX126xReadCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size)
{
}

void SX126xWriteRegisters(uint16_t address, uint8_t *buffer, uint16_t size)
{
}

void SX126xWriteRegister(uint16_t address, uint8_t value)
{
    SX126xWriteRegisters(address, &value, 1);
}

void SX126xReadRegisters(uint16_t address, uint8_t *buffer, uint16_t size)
{
}

uint8_t SX126xReadRegister(uint16_t address)
{
}

void SX126xWriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size)
{
}

void SX126xReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size)
{
}

void SX126xSetRfTxPower(int8_t power)
{
}

uint8_t SX126xGetPaSelect(uint32_t channel)
{
    //    if( GpioRead( &DeviceSel ) == 1 )
    //    {
    //        return SX1261;
    //    }
    //    else
    //    {
    //        return SX1262;
    //    }

    return SX1262;
}

void SX126xAntSwOn(void)
{
    //GpioInit( &AntPow, ANT_SWITCH_POWER, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
}

void SX126xAntSwOff(void)
{
    // GpioInit( &AntPow, ANT_SWITCH_POWER, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

bool SX126xCheckRfFrequency(uint32_t frequency)
{
    // Implement check. Currently all frequencies are supported
    return true;
}
