/*
 * Amazon FreeRTOS CELLULAR Preview Release
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/* The config header is always included first. */
#include "iot_config.h"

/* Standard includes. */
#include <string.h>

#include "platform/iot_threads.h"
#include "event_groups.h"

/* Cellular includes. */
#include "aws_cellular_config.h"
#include "cellular_config_defaults.h"
#include "cellular_comm_interface.h"

#include "stm32f4xx.h"

/* Configure logs for the functions in this file. */
#ifdef IOT_LOG_LEVEL_GLOBAL
    #define LIBRARY_LOG_LEVEL    IOT_LOG_LEVEL_GLOBAL
#else
    #define LIBRARY_LOG_LEVEL    IOT_LOG_ERROR
#endif

#define LIBRARY_LOG_NAME         ( "COMM_IF_UBLOX" )
#include "iot_logging_setup.h"

/*-----------------------------------------------------------*/

/**
 * @brief Control pin definitions.
 */
#define CELLULAR_PWR_EN_GPIO_Port                GPIOE
#define CELLULAR_PWR_EN_Pin                      GPIO_PIN_14
#define CELLULAR_RST_GPIO_Port                   GPIOB
#define CELLULAR_RST_Pin                         GPIO_PIN_5
#define CELLULAR_RTS_GPIO_Port                   GPIOA
#define CELLULAR_RTS_Pin                         GPIO_PIN_12

/**
 * @brief UART pin configuration.
 */
#define CELLULAR_UART_MAIN_TX_GPIO_Port          GPIOA
#define CELLULAR_UART_MAIN_TX_Pin                GPIO_PIN_9
#define CELLULAR_UART_MAIN_RX_GPIO_Port          GPIOA
#define CELLULAR_UART_MAIN_RX_Pin                GPIO_PIN_10
#define CELLULAR_UART_MAIN_RTS_GPIO_Port         GPIOA
#define CELLULAR_UART_MAIN_RTS_Pin               GPIO_PIN_12
#define CELLULAR_UART_MAIN_CTS_GPIO_Port         GPIOA
#define CELLULAR_UART_MAIN_CTS_Pin               GPIO_PIN_11

#define CELLULAR_UART_PINS_MAX                   ( 2U )

/**
 * @brief Device GPIO timing parameters.
 */
#define DEVICE_OFF_PULSE_TIME		                 ( 1500 )
#define DEVICE_ON_PULSE_TIME		               	 ( 1000 )
#define DEVICE_ON_OFF_WAIT_TIME			             ( 5000 )

/*-----------------------------------------------------------*/

/**
 * @brief CELLULAR General Purpose I/O and GPIO Init structure definition.
 */
typedef struct CellularGpioInitStruct
{
    GPIO_TypeDef * Port;
    GPIO_InitTypeDef Init;
} CellularGpioInitStruct_t;

/**
 * @brief Number of supported communication interfaces
 */
enum
{
    CELLULAR_INTERFACE_MAIN_UART = 0, /* The main UART for Cellular interface is UART4. */
    CELLULAR_INTERFACE_MAX
};

/*-----------------------------------------------------------*/

/**
 * @brief Initialize UART pins.
 */
static void prvUartPinInit( void );

/**
 * @brief Deinitialize UART pins.
 */
static void prvUartPinDeInit( void );

/**
 * @brief Initialize device control pins.
 */
static void prvDevicePinInit( void );

/*-----------------------------------------------------------*/

/**
 * @brief Control pin configuration.
 */
/* This structure is defined as configuration. Doesn't belong to specific function. */
/* coverity[misra_c_2012_rule_8_9_violation].*/
static const CellularGpioInitStruct_t _cellularGpioInitStruct[] =
{
    /* Port     Pin                   Mode                  Pull            Speed                   Alternate */
    { .Port = CELLULAR_RST_GPIO_Port,
      .Init ={ CELLULAR_RST_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0 } },

    { .Port = CELLULAR_PWR_EN_GPIO_Port,
      .Init ={ CELLULAR_PWR_EN_Pin, GPIO_MODE_OUTPUT_OD, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW, 0 } },
		
		{ .Port = CELLULAR_RTS_GPIO_Port,
      .Init ={ CELLULAR_RTS_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0 } },

    { NULL }
};

/**
 * @brief UART pin configuration.
 */
static const CellularGpioInitStruct_t _cellularUartInitStruct[ CELLULAR_INTERFACE_MAX ][ CELLULAR_UART_PINS_MAX ] =
{
    /*   Port     Pin                   	Mode                Pull            Speed                     Alternate */
    { 
			{ .Port = CELLULAR_UART_MAIN_TX_GPIO_Port,
        .Init = { CELLULAR_UART_MAIN_TX_Pin, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF7_USART1 } },
      { .Port = CELLULAR_UART_MAIN_RX_GPIO_Port,
        .Init = { CELLULAR_UART_MAIN_RX_Pin, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF7_USART1 } } 
		},
};

/*-----------------------------------------------------------*/

static void prvUartPinInit( void )
{
    const CellularGpioInitStruct_t * pCellularGpioInitStruct = &_cellularUartInitStruct[ CELLULAR_INTERFACE_MAIN_UART ][ 0 ];
    uint8_t count = 0;

    /* UART pin init. */
    for( count = 0; count < CELLULAR_UART_PINS_MAX; count++ )
    {
        if( pCellularGpioInitStruct->Port != NULL )
        {
            HAL_GPIO_Init( ( GPIO_TypeDef * ) pCellularGpioInitStruct->Port,
                           ( GPIO_InitTypeDef * ) &pCellularGpioInitStruct->Init );
        }

        pCellularGpioInitStruct++;
    }
}

/*-----------------------------------------------------------*/

static void prvUartPinDeInit( void )
{
    const CellularGpioInitStruct_t * pCellularGpioInitStruct = &_cellularUartInitStruct[ CELLULAR_INTERFACE_MAIN_UART ][ 0 ];
    uint8_t count = 0;

    /* UART pin deinit. */
    for( count = 0; count < CELLULAR_UART_PINS_MAX; count++ )
    {
        if( pCellularGpioInitStruct->Port != NULL )
        {
            HAL_GPIO_DeInit( ( GPIO_TypeDef * ) pCellularGpioInitStruct->Port,
                             pCellularGpioInitStruct->Init.Pin );
        }

        pCellularGpioInitStruct++;
    }
}

/*-----------------------------------------------------------*/

static void prvDevicePinInit( void )
{
    const CellularGpioInitStruct_t * pCellularGpioInitStruct = _cellularGpioInitStruct;
	
		/* Enable GPIO Ports Clock. */
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		__HAL_RCC_GPIOE_CLK_ENABLE();

		while( pCellularGpioInitStruct->Port != NULL )
		{
				HAL_GPIO_Init( ( GPIO_TypeDef * ) pCellularGpioInitStruct->Port,
											 ( GPIO_InitTypeDef * ) &pCellularGpioInitStruct->Init );
				pCellularGpioInitStruct++;
		}

		//Bring us out of reset
		HAL_GPIO_WritePin( CELLULAR_RST_GPIO_Port, CELLULAR_RST_Pin, GPIO_PIN_SET );
		HAL_GPIO_WritePin( CELLULAR_RTS_GPIO_Port, CELLULAR_RTS_Pin, GPIO_PIN_RESET );
		
		//Power off initially so we start with known state
		HAL_GPIO_WritePin( CELLULAR_PWR_EN_GPIO_Port, CELLULAR_PWR_EN_Pin, GPIO_PIN_RESET );
		HAL_Delay(DEVICE_OFF_PULSE_TIME);
		HAL_GPIO_WritePin( CELLULAR_PWR_EN_GPIO_Port, CELLULAR_PWR_EN_Pin, GPIO_PIN_SET );
		HAL_Delay(DEVICE_ON_OFF_WAIT_TIME);
}

/*-----------------------------------------------------------*/

/**
 * @brief Power on the cellular device.
 *
 * @return IOT_COMM_INTERFACE_SUCCESS if the operation is successful, otherwise an error
 * code indicating the cause of the error.
 */
CellularCommInterfaceError_t CellularDevice_PowerOn( void )
{
    CellularCommInterfaceError_t ret = IOT_COMM_INTERFACE_SUCCESS;
    uint8_t count = 0, countLimit = 0;
    GPIO_PinState pinStatus = GPIO_PIN_SET;

    IotLogDebug( "CellularDevice_PowerOn +" );

    /* Device pin init. */
    prvDevicePinInit();

    /* UART pin init. */
    prvUartPinInit();

		//power on the modem
		HAL_GPIO_WritePin( CELLULAR_PWR_EN_GPIO_Port, CELLULAR_PWR_EN_Pin, GPIO_PIN_RESET );
		HAL_Delay(DEVICE_ON_PULSE_TIME);
		HAL_GPIO_WritePin( CELLULAR_PWR_EN_GPIO_Port, CELLULAR_PWR_EN_Pin, GPIO_PIN_SET );
		HAL_Delay(DEVICE_ON_OFF_WAIT_TIME);	

    //status pin is not available on SARA-R4 so assume modem powered up.
    IotLogDebug( "CellularDevice_PowerOn -" );

    return ret;
}

/*-----------------------------------------------------------*/

/**
 * @brief Power off the cellular device.
 *
 * @return IOT_COMM_INTERFACE_SUCCESS if the operation is successful, otherwise an error
 * code indicating the cause of the error.
 */
void CellularDevice_PowerOff( void )
{
    uint8_t count = 0, countLimit = 0;
    GPIO_PinState pinStatus = GPIO_PIN_SET;

    IotLogDebug( "CellularDevice_PowerOff +" );

		HAL_GPIO_WritePin( CELLULAR_PWR_EN_GPIO_Port, CELLULAR_PWR_EN_Pin, GPIO_PIN_RESET );
		HAL_Delay(DEVICE_OFF_PULSE_TIME);
		HAL_GPIO_WritePin( CELLULAR_PWR_EN_GPIO_Port, CELLULAR_PWR_EN_Pin, GPIO_PIN_SET );
		HAL_Delay(DEVICE_ON_OFF_WAIT_TIME);

    /* UART pin deinit. */
    prvUartPinDeInit();

    IotLogDebug( "CellularDevice_PowerOff -" );
}

/*-----------------------------------------------------------*/
