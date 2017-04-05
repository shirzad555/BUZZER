/*
 * LIS3DH_Driver.h
 *
 *  Created on: Jul 28, 2016
 *      Author: Ross Dehmoobed
 */

#ifndef APPLICATION_LIS3DH_DRIVER_H_
#define APPLICATION_LIS3DH_DRIVER_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// this file is to interface with LIS3DH chip using SPI1
enum LIS3DH_REGISTERS
{
    LIS3DH_REG_STATUS_AUX   = 0x07,
    LIS3DH_REG_OUT_1_L      = 0x08, // OUT_1_L (08h),
    LIS3DH_REG_OUT_1_H      = 0x09, // OUT_1_H (09h)
    LIS3DH_REG_OUT_2_L      = 0x0A, // OUT_2_L (0Ah),
    LIS3DH_REG_OUT_2_H      = 0x0B, // OUT_2_H (0Bh)
    LIS3DH_REG_OUT_3_L      = 0x0C, // OUT_3_L (0Ch),
    LIS3DH_REG_OUT_3_H      = 0x0D, // OUT_3_H (0Dh)
    LIS3DH_REG_INT_COUNTER  = 0x0E, // INT_COUNTER (0Eh)
    LIS3DH_REG_WHO_AM_I     = 0x0F, // WHO_AM_I (0Fh)

    LIS3DH_REG_TEMP_CFG_REG = 0x1F, // TEMP_CFG_REG (1Fh)
    LIS3DH_REG_CTRL_REG1    = 0x20, // CTRL_REG1 (20h)
    LIS3DH_REG_CTRL_REG2    = 0x21, // CTRL_REG4 (21h)
    LIS3DH_REG_CTRL_REG3    = 0x22, // CTRL_REG3 (22h)
    LIS3DH_REG_CTRL_REG4    = 0x23, // CTRL_REG4 (23h)
    LIS3DH_REG_CTRL_REG5    = 0x24, // CTRL_REG5 (24h)
    LIS3DH_REG_CTRL_REG6    = 0x25, // CTRL_REG6 (25h)

    LIS3DH_REG_REFERENCE    = 0x26, // REFERENCE/DATACAPTURE (26h)
    LIS3DH_REG_STATUS       = 0x27, // STATUS_REG (27h)
    LIS3DH_REG_OUT_X_L      = 0x28, // OUT_X_L (28h),
    LIS3DH_REG_OUT_X_H      = 0x29, // OUT_X_H (29h)
    LIS3DH_REG_OUT_Y_L      = 0x2A, // OUT_Y_L (2Ah),
    LIS3DH_REG_OUT_Y_H      = 0x2B, // OUT_Y_H (2Bh)
    LIS3DH_REG_OUT_Z_L      = 0x2C, // OUT_Z_L (2Ch),
    LIS3DH_REG_OUT_Z_H      = 0x2D, // OUT_Z_H (2Dh)
    LIS3DH_REG_FIFO_CTRL    = 0x2E, // FIFO_CTRL_REG (2Eh)
    LIS3DH_REG_FIFO_SRC     = 0x2F, // FIFO_SRC_REG (2Fh)

    LIS3DH_REG_INT1_CFG     = 0x30, // INT1_CFG (30h)
    LIS3DH_REG_INT1_SRC     = 0x31, // INT1_SRC (31h)
    LIS3DH_REG_INT1_THS     = 0x32, // INT1_THS (32h)
    LIS3DH_REG_INT1_DURATION= 0x33, // INT1_DURATION (33h)

    LIS3DH_REG_CLICK_CFG    = 0x38, // CLICK_CFG (38h)
    LIS3DH_REG_CLICK_SRC    = 0x39, // CLICK_SRC (39h)
    LIS3DH_REG_CLICK_THS    = 0x3A, // CLICK_THS (3Ah)
    LIS3DH_REG_TIME_LIMIT   = 0x3B, // TIME_LIMIT (3Bh)
    LIS3DH_REG_TIME_LATENCY = 0x3C, // TIME_LATENCY (3Ch)
    LIS3DH_REG_TIME_WINDOW  = 0x3D, // TIME WINDOW (3Dh)

};
// initialize SPI and talk to the LIS3DH to ensure connectivitiy
// returns true if success - false otherwise
bool LIS3DH_Initialize(void);

bool LIS3DH_StartTransfer(void);

bool LIS3DH_VerifyCommunication(void);


#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_LIS3DH_DRIVER_H_ */
