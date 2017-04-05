/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       Board.h
 *
 *  @brief      CC26XXEM_7ID Board Specific header file.
 *              The project options should point to this file if this is the
 *              CC26XXEM you are developing code for.
 *
 *  The CC26XX header file should be included in an application as follows:
 *  @code
 *  #include <Board.h>
 *  @endcode
 *
 *  ============================================================================
 */
#ifndef __CC26XXEM_7ID_H__
#define __CC26XXEM_7ID_H__

#ifdef __cplusplus
extern "C" {
#endif

/** ============================================================================
 *  Symbol by generic Board.c to include the correct kit specific Board.c
 *  ==========================================================================*/
#define CC2650EM_7ID


/** ============================================================================
 *  Includes
 *  ==========================================================================*/
#include <ti/drivers/PIN.h>
#include <driverlib/ioc.h>

/** ============================================================================
 *  Externs
 *  ==========================================================================*/
extern PIN_Config BoardGpioInitTable[];

/** ============================================================================
 *  Defines
 *  ==========================================================================*/

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>                <pin mapping>
 */
#define LED_ON          (1)
#define LED_OFF         (0)

#define BOARD_LED1      (IOID_3)
#define BOARD_LED2      (IOID_4)
#define LIS3DH_INT      (IOID_7)

#define SPI_CS          (IOID_8)
#define SPI_SCK         (IOID_9)
#define SPI_MOSI        (IOID_10)
#define SPI_MISO        (IOID_11)

#define BOARD_UART_TX   (IOID_0)
#define BOARD_UART_RX   (IOID_1)

#define BUZZER_PWM      (IOID_2)
#define BUZZER_SHDN_N   (IOID_12)

#define MOTIONSNS_SPI   (CC26XX_SPI1)
/** ============================================================================
 *  Number of peripherals and their names
 *  ==========================================================================*/

/*!
 *  @def    CC26XX_I2CName
 *  @brief  Enum of I2C names on the CC26XX dev board
 */
typedef enum CC26XX_I2CName {
    CC26XX_I2C0 = 0,
    CC26XX_I2CCOUNT
} CC26XX_I2CName;

/*!
 *  @def    CC26XX_CryptoName
 *  @brief  Enum of Crypto names on the CC26XX dev board
 */
typedef enum CC26XX_CryptoName {
    CC26XX_CRYPTO0 = 0,
    CC26XX_CRYPTOCOUNT
} CC26XX_CryptoName;

/*!
 *  @def    CC26XX_SPIName
 *  @brief  Enum of SPI names on the CC26XX dev board
 */
typedef enum CC26XX_SPIName {
    CC26XX_SPI0 = 0,
    CC26XX_SPI1,
    CC26XX_SPICOUNT
} CC26XX_SPIName;

/*!
 *  @def    CC26XX_UARTName
 *  @brief  Enum of UARTs on the CC26XX dev board
 */
typedef enum CC26XX_UARTName {
    CC26XX_UART0 = 0,
    CC26XX_UARTCOUNT
} CC26XX_UARTName;

/*!
 *  @def    CC26XX_UdmaName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC26XX_UdmaName {
    CC26XX_UDMA0 = 0,
    CC26XX_UDMACOUNT
} CC26XX_UdmaName;

#ifdef __cplusplus
}
#endif

#endif /* __CC26XXEM_H__ */
