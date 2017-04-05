/*
 * LIS3DH_Driver.c
 *
 *  Created on: Jul 28, 2016
 *      Author: Ross Dehmoobed
 */

#include <stdint.h>
#include <stddef.h>

#include <LIS3DH_Driver.h>
#include <Board.h>
#include <SPICC26XXDMA.h>

static SPI_Handle handle;

inline uint8_t BUILD_LIS3DH_COMMAND(bool RW, bool MS, uint8_t uAddr)
{
    return (RW << 7) | ( MS << 6) | (uAddr & 0x3F);
}


bool LIS3DH_Initialize(void)
{
    SPI_Params params;
    // Init SPI and specify non-default parameters
    SPI_Params_init(&params);
    params.bitRate     = 1000000;
    params.frameFormat = SPI_POL1_PHA1;
    params.mode        = SPI_MASTER;

    // Open the SPI and perform the transfer
    handle = SPI_open(MOTIONSNS_SPI, &params);

    return LIS3DH_VerifyCommunication();
}

bool LIS3DH_StartTransfer(void)
{
    SPI_Transaction transaction;
    uint8_t txBuf[] = "Hello World";    // Transmit buffer
    uint8_t rxBuf[11];                  // Receive buffer

    // Configure the transaction
    transaction.count = sizeof(txBuf);
    transaction.txBuf = txBuf;
    transaction.rxBuf = rxBuf;

    return SPI_transfer(handle, &transaction);
}

bool LIS3DH_VerifyCommunication(void)
{
    bool bResult = false;

    uint8_t uCommand = BUILD_LIS3DH_COMMAND(1,0, LIS3DH_REG_WHO_AM_I);
    //todo send the command
    //todo verify the response

    return bResult;
}
