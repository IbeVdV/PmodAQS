/***************************************************************************//**
 * @file main_zg_hg.c
 * @brief This project demonstrates both master and slave configurations of the
 * EFM32 I2C peripheral. Two EFM32 I2C modules are connected and set up to both
 * transmit (master mode) and receive (slave mode) between each other using a
 * common I2C bus.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *******************************************************************************
 * # Evaluation Quality
 * This code has been minimally tested to ensure that it builds and is suitable 
 * as a demonstration for evaluation purposes only. This code will be maintained
 * at the sole discretion of Silicon Labs.
 ******************************************************************************/
 
#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_i2c.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "bsp.h"
#include "i2cspm.h"


// Defines
#define CORE_FREQUENCY                100000
#define RTC_MIN_TIMEOUT                32000
#define I2C_ADDRESS                     0xB6
#define I2C_RXBUFFER_SIZE                 1


#define CCS811_OK                            0x0000   /**< No errors                            */
#define CCS811_ERROR_APPLICATION_NOT_PRESENT 0x0001   /**< Application firmware is not present  */
#define CCS811_ERROR_NOT_IN_APPLICATION_MODE 0x0002   /**< The part is not in application mode  */
#define CCS811_ERROR_DRIVER_NOT_INITIALIZED  0x0003   /**< The driver is not initialized        */
#define CCS811_ERROR_I2C_TRANSACTION_FAILED  0x0004   /**< I2C transaction failed               */
#define CCS811_ERROR_INIT_FAILED             0x0005   /**< The initialization failed            */
#define CCS811_ERROR_FIRMWARE_UPDATE_FAILED  0x0006   /**< The firmware update was unsuccessful */
/**@}*/

/**************************************************************************//**
* @name Register Addresses
* @{
******************************************************************************/
#define CCS811_ADDR_STATUS                   0x00 /**< Status register                                                                           */
#define CCS811_ADDR_MEASURE_MODE             0x01 /**< Measurement mode and conditions register                                                  */
#define CCS811_ADDR_ALG_RESULT_DATA          0x02 /**< Algorithm result                                                                          */
#define CCS811_ADDR_RAW_DATA                 0x03 /**< Raw ADC data values for resistance and current source used                                */
#define CCS811_ADDR_ENV_DATA                 0x05 /**< Temperature and Humidity data can be written to enable compensation                       */
#define CCS811_ADDR_NTC                      0x06 /**< Provides the voltage across the reference resistor and the voltage across the NTC resistor */
#define CCS811_ADDR_THRESHOLDS               0x10 /**< Thresholds for operation when interrupts are only generated when eCO2 ppm crosses a threshold */
#define CCS811_ADDR_HW_ID                    0x20 /**< Hardware ID                                                                               */
#define CCS811_ADDR_HW_VERSION               0x21 /**< Hardware Version                                                                          */
#define CCS811_ADDR_FW_BOOT_VERSION          0x23 /**< Firmware Boot Version                                                                     */
#define CCS811_ADDR_FW_APP_VERSION           0x24 /**< Firmware Application Version                                                              */
#define CCS811_ADDR_ERR_ID                   0xE0 /**< Error ID                                                                                  */
#define CCS811_ADDR_FW_ERASE                 0xF1 /**< Firmware erase                                                                            */
#define CCS811_ADDR_FW_PROGRAM               0xF2 /**< Firmware programming                                                                      */
#define CCS811_ADDR_FW_VERIFY                0xF3 /**< Firmware verification                                                                     */
#define CCS811_ADDR_APP_START                0xF4 /**< Application start                                                                         */
#define CCS811_ADDR_SW_RESET                 0xFF /**< Software reset                                                                            */
/**@}*/

/**************************************************************************//**
* @name Measure mode value definitions
* @{
******************************************************************************/
#define CCS811_MEASURE_MODE_DRIVE_MODE_SHIFT 4     /**< DRIVE_MODE field bit shift value                                                           */
#define CCS811_MEASURE_MODE_DRIVE_MODE_IDLE  0x00  /**< Idle mode, measurements are disabled                                                       */
#define CCS811_MEASURE_MODE_DRIVE_MODE_1SEC  0x10  /**< IAQ Mode 1, a measurement is performed every second                                        */
#define CCS811_MEASURE_MODE_DRIVE_MODE_10SEC 0x20  /**< IAQ Mode 2, a measurement is performed every 10 seconds                                    */
#define CCS811_MEASURE_MODE_DRIVE_MODE_60SEC 0x30  /**< IAQ Mode 3, a measurement is performed every 60 seconds                                    */
#define CCS811_MEASURE_MODE_DRIVE_MODE_RAW   0x40  /**< IAQ Mode 4, Raw Data Mode, a measurement is performed every 250ms for external algorithms  */
#define CCS811_MEASURE_MODE_INTERRUPT        0x08  /**< Interrupt generation enable                                                                */
#define CCS811_MEASURE_MODE_THRESH           0x04  /**< Enable interrupt when eCO2 level exceeds threshold
/**@}*/



bool dataAvailable = false;
uint8_t treshData[] = {0x03,0x84, 0x05, 0xDC}; //low to med / med to high

static volatile uint32_t msTicks; /* counts 1ms timeTicks */

/**************************************************************************//**
 * @brief  Starting oscillators and enabling clocks
 *****************************************************************************/
void initCMU(void)
{
  // Enabling clock to the I2C, GPIO, LE
  CMU_ClockEnable(cmuClock_I2C0, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_HFLE, true);

  // Starting LFXO and waiting until it is stable
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
}

/**************************************************************************//**
 * @brief GPIO initialization
 *****************************************************************************/
void initGPIO(void)
{
  GPIO_PinModeSet(gpioPortC, 10, gpioModeInput, 0);
  GPIO_IntConfig(gpioPortC, 10, false, true, true);
}


/**************************************************************************//**
 * @brief  disables Sensor interrupts
 *****************************************************************************/
void disableSensorInterrupts(void)
{
	  NVIC_DisableIRQ(GPIO_EVEN_IRQn);
	  NVIC_DisableIRQ(GPIO_ODD_IRQn);
}


/**************************************************************************//**
 * @brief  enables Sensor slave interrupts
 *****************************************************************************/
void enableSensorInterrupts(void)
{
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

/**************************************************************************//**
 * @brief  Setup I2C
 *****************************************************************************/
void initI2C(void)
{
  // Using default settings
  I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
  i2cInit.freq = I2C_FREQ_FAST_MAX;

  // Using PD6 (SDA) and PD7 (SCL)
  GPIO_PinModeSet(gpioPortD, 6, gpioModeWiredAndPullUpFilter, 1);
  GPIO_PinModeSet(gpioPortD, 7, gpioModeWiredAndPullUpFilter, 1);

  // Enable pins at location 1 as specified in datasheet
  I2C0->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN;
  I2C0->ROUTE = (I2C0->ROUTE & (~_I2C_ROUTE_LOCATION_MASK)) | I2C_ROUTE_LOCATION_LOC1;

  // Initializing the I2C
  I2C_Init(I2C0, &i2cInit);
}

/**************************************************************************//**
 * @brief  Transmitting I2C data. Will busy-wait until the transfer is complete.
 *****************************************************************************/
uint8_t writeRegister(uint8_t id, uint8_t data)
{
  // Transfer structure
  I2C_TransferSeq_TypeDef i2cTransfer;
  I2C_TransferReturn_TypeDef result;

  uint8_t i2c_write_data[1];
  i2c_write_data[0] = id;

  // Initializing I2C transfer
  i2cTransfer.addr          = I2C_ADDRESS;
  i2cTransfer.flags         = I2C_FLAG_WRITE_WRITE;
  i2cTransfer.buf[0].data   = i2c_write_data;
  i2cTransfer.buf[0].len    = 1;
  i2cTransfer.buf[1].data   = &data;
  i2cTransfer.buf[1].len    = 1;

  result = I2CSPM_Transfer(I2C0, &i2cTransfer);

  // Sending data
  if(result != i2cTransferDone)
   {
	  return 1;
   }
  return 0;
}


uint8_t writeMailbox(uint8_t id, uint8_t* data)
{
  // Transfer structure
  I2C_TransferSeq_TypeDef i2cTransfer;
  I2C_TransferReturn_TypeDef result;

  uint8_t i2c_write_data[1];
  i2c_write_data[0] = id;

  // Initializing I2C transfer
  i2cTransfer.addr          = I2C_ADDRESS;
  i2cTransfer.flags         = I2C_FLAG_WRITE_WRITE;
  i2cTransfer.buf[0].data   = i2c_write_data;
  i2cTransfer.buf[0].len    = 1;
  i2cTransfer.buf[1].data   = data;
  i2cTransfer.buf[1].len    = sizeof(data);

  result = I2CSPM_Transfer(I2C0, &i2cTransfer);

  // Sending data
  if(result != i2cTransferDone)
   {
	  return 1;
   }
  return 0;
}


void writeNoData(uint8_t id)
{
  // Transfer structure
  I2C_TransferSeq_TypeDef i2cTransfer;
  I2C_TransferReturn_TypeDef result;

  uint8_t i2c_write_data[1];
  i2c_write_data[0] = id;

  // Initializing I2C transfer
  i2cTransfer.addr          = I2C_ADDRESS;
  i2cTransfer.flags         = I2C_FLAG_WRITE;
  i2cTransfer.buf[0].data   = i2c_write_data;
  i2cTransfer.buf[0].len    = 1;
  result = I2C_TransferInit(I2C0, &i2cTransfer);

  // Sending data
  while (result == i2cTransferInProgress)
  {
    result = I2C_Transfer(I2C0);
  }
}



uint8_t readMailbox(uint8_t id, uint8_t length, uint8_t *data)
{
  // Transfer structure
  I2C_TransferSeq_TypeDef i2cTransfer;
  I2C_TransferReturn_TypeDef result;

  uint8_t i2c_write_data[1];
  i2c_write_data[0] = id;

  // Initializing I2C transfer
  i2cTransfer.addr          = I2C_ADDRESS;
  i2cTransfer.flags         = I2C_FLAG_WRITE_READ;
  i2cTransfer.buf[0].data   = i2c_write_data;
  i2cTransfer.buf[0].len    = 1;
  i2cTransfer.buf[1].data   = data;
  i2cTransfer.buf[1].len    = length;

  result = I2CSPM_Transfer(I2C0, &i2cTransfer);

    // Sending data
    if(result != i2cTransferDone)
     {
    	return 1;
     }
    return 0;
}

/***************************************************************************//**
 * @brief GPIO Interrupt handler
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
	  uint32_t interruptMask = GPIO_IntGet();
	  GPIO_IntClear(interruptMask);
	  dataAvailable = true;
	}

void GPIO_ODD_IRQHandler(void)
{
	  uint32_t interruptMask = GPIO_IntGet();
	  GPIO_IntClear(interruptMask);
	  dataAvailable = true;
	}

void SysTick_Handler(void)
{
  msTicks++;       /* increment counter necessary in Delay()*/
}

static void Delay(uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) ;
}


int main(void)
{
  // Chip errata
  CHIP_Init();

  // Configuring clocks in the Clock Management Unit (CMU)
  initCMU();
  
  // Initializations
  initGPIO();

  // Setting up i2c
  initI2C();

  BSP_LedsInit();

  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) {
     while (1) ;
   }


   Delay(1000);
   uint8_t data[1];
   Delay(10);
   readMailbox(CCS811_ADDR_HW_ID,1,data);
   Delay(10);
   readMailbox(CCS811_ADDR_STATUS,1,data);
   Delay(10);
   writeNoData(CCS811_ADDR_FW_VERIFY);
   Delay(100);
   writeNoData(CCS811_ADDR_APP_START);
   Delay(1000);
   //writeMailbox(CCS811_ADDR_THRESHOLDS,treshData);
   //Delay(10);
   writeRegister(CCS811_ADDR_MEASURE_MODE,CCS811_MEASURE_MODE_DRIVE_MODE_1SEC + CCS811_MEASURE_MODE_INTERRUPT);
   Delay(10);
   readMailbox(CCS811_ADDR_ERR_ID,1,data);
   Delay(10);
   readMailbox(CCS811_ADDR_MEASURE_MODE,1,data);
   enableSensorInterrupts();
   EMU_EnterEM3(false);
   uint8_t algresultData[2];

  while (1)
  {
	disableSensorInterrupts();
    if(dataAvailable)
    {

    	readMailbox(CCS811_ADDR_ALG_RESULT_DATA,2,algresultData);
    	uint32_t concatData = (algresultData[0] << 8) | algresultData[1];
    	dataAvailable = false;
    	if(concatData > 1200){
    		BSP_LedsSet(3);

    	}
    	else if(concatData > 900){
    		BSP_LedsSet(1);
    	}
    	else{
    		BSP_LedsSet(0);
    	}

    }
    enableSensorInterrupts();
    EMU_EnterEM3(false);
  }

}
