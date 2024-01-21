/*
  An Arduino Library which allows you to communicate seamlessly with u-blox GNSS modules using the Configuration Interface

  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/15136
  https://www.sparkfun.com/products/16481
  https://www.sparkfun.com/products/16344
  https://www.sparkfun.com/products/18037
  https://www.sparkfun.com/products/18719
  https://www.sparkfun.com/products/18774
  https://www.sparkfun.com/products/19663
  https://www.sparkfun.com/products/17722

  Original version by Nathan Seidle @ SparkFun Electronics, September 6th, 2018
  v2.0 rework by Paul Clark @ SparkFun Electronics, December 31st, 2020
  v3.0 rework by Paul Clark @ SparkFun Electronics, December 8th, 2022

  https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3

  This library is an updated version of the popular SparkFun u-blox GNSS Arduino Library.
  v3 uses the u-blox Configuration Interface (VALSET and VALGET) to:
  detect the module (during begin); configure message intervals; configure the base location; etc..

  This version of the library will not work with older GNSS modules.
  It is specifically written for newer modules like the ZED-F9P, ZED-F9R and MAX-M10S.
  For older modules, please use v2 of the library: https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.19

  SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
  The MIT License (MIT)
  Copyright (c) 2018 SparkFun Electronics
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
  associated documentation files (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the Software is furnished to
  do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial
  portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
  NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

// sfe_bus.cpp

#include "stm32h7xx_hal_i2c.h"
#include "stm32h7xx_hal_spi.h"
#include "stm32h7xx_hal_uart.h"
#include "stm32h7xx_hal.h"
#include "sfe_bus.h"

//////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor
//

namespace SparkFun_UBLOX_GNSS
{

  SfeI2C::SfeI2C(void) : _i2cPort{nullptr}, _address{0}
  {
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // I2C init()
  //
  // Methods to init/setup this device.
  // The caller can provide a Wire Port, or this class will use the default.
  // Always update the address in case the user has changed the I2C address - see Example9
  bool SfeI2C::init(I2C_HandleTypeDef &hi2c, uint8_t address)
  {
 
    _i2cPort = &hi2c;
    _address = address;

    return true;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // I2C init() - Unused
  //
  // Methods to init/setup this device.
  // The caller can provide a Wire Port, or this class will use the default.
  /*
  bool SfeI2C::init(uint8_t address)
  {
    return init(Wire, address);
  }*/

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // ping()
  //
  // Is a device connected?
  
/*  MODIFIED  */
  bool SfeI2C::ping() 
  {
    if (_i2cPort == nullptr) {return false;}

    if (HAL_I2C_IsDeviceReady(_i2cPort, _address, 2, 100) == HAL_OK) {return true;} // Device responded
    else                                                             {return false;} // No response from the device
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////
  // available()
  //
  // Checks how many bytes are waiting in the GNSS's I2C buffer
  // It does this by reading registers 0xFD and 0xFE
  //
  // From the u-blox integration manual:
  // "There are two forms of DDC read transfer. The "random access" form includes a peripheral register
  //  address and thus allows any register to be read. The second "current address" form omits the
  //  register address. If this second form is used, then an address pointer in the receiver is used to
  //  determine which register to read. This address pointer will increment after each read unless it
  //  is already pointing at register 0xFF, the highest addressable register, in which case it remains
  //  unaltered."

/*  MODIFIED  */
  uint16_t SfeI2C::available()
  {
    if (_i2cPort == nullptr)
    {
      return 0;
    }

    uint8_t reg = 0xFD;
    uint16_t bytesAvailable = 0;
    uint8_t buffer[2] = {0, 0};

    // Transmit the register address we want to read from
    if (HAL_I2C_Master_Transmit(_i2cPort, _address, &reg, 1, 100) != HAL_OK)
    {
      return 0;
    } // Transmission error

    // Receive 2 bytes of data
    if (HAL_I2C_Master_Receive(_i2cPort, _address, buffer, 2, 100) != HAL_OK)
    {
      return 0;
    } // Receive error

    bytesAvailable = (uint16_t)(buffer[0] << 8 | buffer[1]);
    return bytesAvailable;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // writeBytes()

/*  MODIFIED  */
  uint8_t SfeI2C::writeBytes(uint8_t *dataToWrite, uint8_t length) 
  {
    if (_i2cPort == nullptr || length == 0)
      return 0;

    HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(_i2cPort, _address, dataToWrite, length, 100);

    if (result == HAL_OK) {return length;}
    else                  {return 0;} // Transmission failed
  }


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // readBytes()

/*  MODIFIED  */
  uint8_t SfeI2C::readBytes(uint8_t *data, uint8_t length) 
  {
    if (_i2cPort == nullptr || length == 0)
      return 0;

    HAL_StatusTypeDef result = HAL_I2C_Master_Receive(_i2cPort, _address, data, length, 100); // 100ms timeout

    if (result == HAL_OK) {return length;} // If HAL_OK, all requested bytes were successfully read
    else                  {return 0;} // If there was an error, return 0
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////
  // Constructor
  //

  SfeSPI::SfeSPI(void) : _spiPort{nullptr}{}

  ////////////////////////////////////////////////////////////////////////////////////////////////
  // SPI init()
  //

/*  MODIFIED  */
  bool SfeSPI::init(SPI_HandleTypeDef &spiPort, SPI_InitTypeDef spiSettings, GPIO_TypeDef &gpioPort, uint16_t cs, bool bInit)
  {
    if (!_spiPort)
    {
      _spiPort = &spiPort;
      _sfeSPISettings = spiSettings;

      if (bInit)
      {
        // Apply the settings provided by spiSettings
        _spiPort->Init = spiSettings;

        // Initialize the SPI peripheral
        if (HAL_SPI_Init(_spiPort) != HAL_OK)
        {
          // Initialization failed
          return false;
        }
      }
    }

    _cs = cs; // Chip Select pin
    _gpioPort = &gpioPort;
    return true;
  }

/*  MODIFIED  */
  /*bool SfeSPI::init(SPI_HandleTypeDef &spiPort, uint8_t cs) 
  {
    // Define default SPI settings
    uint32_t spiSpeed = SPI_BAUDRATEPRESCALER_16; // Example prescaler value, adjust as needed

    // Call the full init function with the default SPI port and settings
    return init(spiPort, spiSpeed, cs, true);
  }*/


  ////////////////////////////////////////////////////////////////////////////////////////////////
  // SPI init()
  //
  // Methods to init/setup this device. The caller can provide a SPI Port, or this class
  // will use the default
  
/*  MODIFIED  */
/*
  bool SfeSPI::init(SPI_HandleTypeDef &spiPort, uint32_t spiSpeed, uint8_t cs, bool bInit)
  {

    // If the transaction settings are not provided by the user they are built here.
    SPISettings spiSettings = SPISettings(spiSpeed, MSBFIRST, SPI_MODE0);

    // In addition of the port is not provided by the user, it defaults to SPI here.
    return init(spiPort, spiSettings, cs, bInit);
  }*/

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // available()
  //
  // available isn't applicable for SPI

  uint16_t SfeSPI::available() {return (0);}

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // writeBytes()

/*  MODIFIED  */
  uint8_t SfeSPI::writeBytes(uint8_t *data, uint8_t length)
  {
    if (!_spiPort || length == 0)
      return 0;

    // Signal communication start
    HAL_GPIO_WritePin(_gpioPort, _cs, GPIO_PIN_RESET);

    // Transmit data
    if (HAL_SPI_Transmit(_spiPort, data, length, HAL_MAX_DELAY) == HAL_OK)
    {
      // Signal end of communication
      HAL_GPIO_WritePin(_gpioPort, _cs, GPIO_PIN_SET);
      return length;
    }

    HAL_GPIO_WritePin(_gpioPort, _cs, GPIO_PIN_SET);
    return 0;
  }

/*  MODIFIED  */
  uint8_t SfeSPI::readBytes(uint8_t *data, uint8_t length)
  {
    if (!_spiPort || length == 0)
      return 0;

    uint8_t dummyData = 0xFF;
    uint8_t readData[length];

    // Signal communication start
    HAL_GPIO_WritePin(_gpioPort, _cs, GPIO_PIN_RESET);

    // Receive data
    if (HAL_SPI_TransmitReceive(_spiPort, &dummyData, readData, length, HAL_MAX_DELAY) == HAL_OK)
    {
      memcpy(data, readData, length);
      // Signal end of communication
      HAL_GPIO_WritePin(_gpioPort, _cs, GPIO_PIN_SET);
      return length;
    }

    HAL_GPIO_WritePin(_gpioPort, _cs, GPIO_PIN_SET);
    return 0;
  }

/*  MODIFIED  */
  uint8_t SfeSPI::writeReadBytes(const uint8_t *data, uint8_t *readData, uint8_t length)
  {
    if (!_spiPort || length == 0)
      return 0;

    // Signal communication start
    HAL_GPIO_WritePin(_gpioPort, _cs, GPIO_PIN_RESET);

    // Transmit and receive data
    if (HAL_SPI_TransmitReceive(_spiPort, (uint8_t *)data, readData, length, HAL_MAX_DELAY) == HAL_OK)
    {
      // Signal end of communication
      HAL_GPIO_WritePin(_gpioPort, _cs, GPIO_PIN_SET);
      return length;
    }

    HAL_GPIO_WritePin(_gpioPort, _cs, GPIO_PIN_SET);
    return 0;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // writeReadBytes()

/*  MODIFIED  */
void SfeSPI::startWriteReadByte() // beginTransaction
{
  if (!_spiPort)
    return;

    // No need to apply settings as HAL_SPI_Init should have already been called

    // Signal communication start by pulling CS low
  HAL_GPIO_WritePin(_gpioPort, _cs, GPIO_PIN_RESET);
}

void SfeSPI::writeReadByte(const uint8_t *data, uint8_t *readData) 
{
  if (!_spiPort)
    return;

  HAL_SPI_TransmitReceive(_spiPort, (uint8_t *)data, readData, 1, HAL_MAX_DELAY);
}

void SfeSPI::writeReadByte(const uint8_t data, uint8_t *readData) 
{
  if (!_spiPort)
    return;

  HAL_SPI_TransmitReceive(_spiPort, (uint8_t *)&data, readData, 1, HAL_MAX_DELAY);
}

void SfeSPI::endWriteReadByte() 
{
    // Raise CS to end the transaction
  HAL_GPIO_WritePin(_gpioPort, _cs, GPIO_PIN_SET);
}


//UART Functionalites Left out


  //////////////////////////////////////////////////////////////////////////////////////////////////
  // Constructor
  //

  SfeSerial::SfeSerial(void) : _serialPort{nullptr}
  {
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // Serial init()
  //
  // Methods to init/setup this device

  bool SfeSerial::init(UART_HandleTypeDef &serialPort)
  {
    // if we don't have a port already
    if (!_serialPort)
    {
      _serialPort = &serialPort;
    }

    // Get rid of any stale serial data already in the processor's RX buffer
    while (_serialPort->available())
      _serialPort->read();

    return true;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // available()

  uint16_t SfeSerial::available()
  {

    if (!_serialPort)
      return 0;

    return (_serialPort->available());
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // writeBytes()

  uint8_t SfeSerial::writeBytes(uint8_t *dataToWrite, uint8_t length)
  {
    if (!_serialPort)
      return 0;

    if (length == 0)
      return 0;

    return _serialPort->write(dataToWrite, length);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // readBytes()

  uint8_t SfeSerial::readBytes(uint8_t *data, uint8_t length)
  {
    if (!_serialPort)
      return 0;

    if (length == 0)
      return 0;

    return _serialPort->readBytes(data, length);
  }
}
