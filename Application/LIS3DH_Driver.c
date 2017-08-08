

#include <stdint.h>
#include <stddef.h>

#include "PINCC26XX.h"
#include <LIS3DH_Driver.h>
#include <Board.h>
//#include <SPICC26XXDMA.h>

//#include "SPI.h"
#include <ti/drivers/SPI.h>

//#include <ti/sysbios/family/arm/cc26xx/Power.h>
//#include <ti/sysbios/knl/Task.h>
//#include <ti/drivers/spi/SPICC26XXDMA.h>
//#include <ti/drivers/dma/UDMACC26XX.h>

//#include <driverlib/ssi.h>

void sbp_spiCallback(SPI_Handle handle, SPI_Transaction * transaction);

static SPI_Handle spiHandle;

inline uint8_t BUILD_LIS3DH_COMMAND(bool RW, bool MS, uint8_t uAddr)
{
    return (RW << 7) | ( MS << 6) | (uAddr & 0x3F);
}


bool LIS3DH_Initialize(void)
{
    SPI_Params spiParams;
    SPI_init();
    // Init SPI and specify non-default parameters
    SPI_Params_init(&spiParams);

    spiParams.mode                     = SPI_MASTER;
    spiParams.transferMode             = SPI_MODE_BLOCKING; //SPI_MODE_CALLBACK; //SPI_MODE_CALLBACK; // SPI_MODE_BLOCKING
    spiParams.transferCallbackFxn      = sbp_spiCallback;
    spiParams.bitRate                  = 800000;
    spiParams.frameFormat              = SPI_POL1_PHA1;// Clock to be normally high, write on the falling edge and capture data on the rising edge //SPI_POL1_PHA1; (clock is normally high) //SPI_POL0_PHA0; (clock is normally low)
//    spiParams.dataSize				   = 16;
    spiHandle = SPI_open(MOTIONSNS_SPI, &spiParams);
/*    params.bitRate     = 1000000;
    params.frameFormat = SPI_POL1_PHA1;
    params.mode        = SPI_MASTER;
*/
    // Open the SPI and perform the transfer
//    handle = SPI_open(MOTIONSNS_SPI, &params);

	PINCC26XX_setOutputEnable(SPI_CS, 1);
	PINCC26XX_setOutputValue(SPI_CS, 1);

	PINCC26XX_setOutputEnable(SPI_MISO, 0);

    return LIS3DH_VerifyCommunication();

}

bool LIS3DH_StartTransfer(uint8_t len, uint8_t * txBuf, uint8_t * rxBuf, uint8_t * args)
{
	bool bResult = false;
    SPI_Transaction transaction;
    //uint8_t txBuf[] = "Hello World";    // Transmit buffer
    //uint8_t rxBuf[16];                  // Receive buffer
    uint8_t test = 0xFA;


    // Configure the transaction
    transaction.count = len;
    transaction.txBuf = txBuf;
    transaction.rxBuf = rxBuf;
    transaction.arg   = (void *) &test;

    PINCC26XX_setOutputValue(SPI_CS, 0);
    bResult = SPI_transfer(spiHandle, &transaction);
    PINCC26XX_setOutputValue(SPI_CS, 1);

    return bResult; //SPI_transfer(spiHandle, &transaction);
}

bool LIS3DH_VerifyCommunication(void)
{
    bool bResult = false;
    uint8_t arg = 0xAB;
    uint8_t uCommand = BUILD_LIS3DH_COMMAND(1,0, LIS3DH_REG_WHO_AM_I);
    uint8_t rxBuf[2];                  // Receive buffer

    bResult = LIS3DH_StartTransfer(2, &uCommand, rxBuf, &arg);
    if (!bResult) return false;
    if (rxBuf[0] != 0x33) return false; // 0x33 is the expected respond from LIS3DH for WHO_AM_I

    return true;
}

void sbp_spiCallback(SPI_Handle handle, SPI_Transaction * transaction)
{
    uint8_t * args = (uint8_t *)transaction->arg;
    uint8_t key;

    // may want to disable the interrupt first
    key = Hwi_disable();
    PINCC26XX_setOutputValue(SPI_CS, 1);
    if (*args == 0xAB ) PINCC26XX_setOutputValue(BOARD_LED2, 1);
    if(transaction->status == SPI_TRANSFER_COMPLETED){
        // do something here for successful transaction...
    }
    Hwi_restore(key);
}

bool LIS3DH_SetDeviceMode(LIS3DH_OperatingMode_t operatingMode, LIS3DH_DataRate_t dataRate, LIS3DH_FullScaleSelection_t fullScaleSelection)
{
	bool bResult = false;
	uint8_t rxBuf[2];
	uint8_t txBuf[2];
	uint8_t arg = NULL;
	txBuf[1]= 0;

	txBuf[0] = BUILD_LIS3DH_COMMAND(0,0, LIS3DH_REG_CTRL_REG1); // ODR3 ODR2 ODR1 ODR0 LPen Zen Yen Xen
	txBuf[1]= ((uint8_t)dataRate << 4) | ((uint8_t)operatingMode << 3) | (0x07); // 0x3F;
	bResult = LIS3DH_StartTransfer(2, txBuf, rxBuf, &arg);
	if (!bResult) return bResult;

	//uCommand = BUILD_LIS3DH_COMMAND(0,0, LIS3DH_REG_CTRL_REG4); // 	BDU BLE FS1 FS0 HR ST1 ST0 SIM
	txBuf[0] = BUILD_LIS3DH_COMMAND(0,0, LIS3DH_REG_CTRL_REG4); // 	BDU=1 BLE FS1 FS0 HR ST1 ST0 SIM
	txBuf[1]= 0;
	if (operatingMode == LIS3DH_MODE_HIGH_RES_POWER) txBuf[1] = 0x88; //((uint8_t)fullScaleSelection << 3) | (0x88); /// txBuf[1]= 0x88;
	else txBuf[1] = 0x80;
	//rxBuf[0]= (fullScaleSelection << 5);
	bResult = LIS3DH_StartTransfer(2, txBuf, rxBuf, &arg);

	return bResult;
}
/*
For 8-bit, and range of +/-2g the output will be from 7F(2g)->0(0g)->FF(-1mg)->C0(-1g)->80(-2g),  when just sitting we only have 1g so it will be half the values as
3f(1g)->0(0g)->FF(-1g)->C0(-1g), note the device in 8-bit mode can be off by i think 16mg
I have see 0x100, 0x107 and ... when i expected to see 0 which i think is a bug, so in 8 bit mode we need to mask the high MSB and change the 2s complement to negative
*/
bool LIS3DH_ReadDeviceValue(uint16_t * xValue, uint16_t * yValue, uint16_t * zValue)
{
	bool bResult = false;
	uint8_t rxBuf[7];
	uint8_t arg = NULL;

	uint8_t uCommand = BUILD_LIS3DH_COMMAND(1,1, LIS3DH_REG_OUT_X_L); // Read all XYZ values together
	bResult = LIS3DH_StartTransfer(7, &uCommand, rxBuf, &arg);

	*xValue = (8 << (uint16_t)rxBuf[1]) +rxBuf[2];
	*yValue = (8 << (uint16_t)rxBuf[3]) +rxBuf[4];
	*zValue = (8 << (uint16_t)rxBuf[5]) +rxBuf[6];

	return bResult;
}

bool LIS3DH_SetFilter(LIS3DH_Filter filter_Parms)
{
	bool bResult = false;
	uint8_t rxBuf[2];
	uint8_t txBuf[2];
	uint8_t arg = NULL;
	txBuf[1]= 0;

	txBuf[0] = BUILD_LIS3DH_COMMAND(0,0, LIS3DH_REG_CTRL_REG2); // HPM1 HPM0 HPCF2 HPCF1 FDS HPCLICK HP_IA2 HP_IA1
	txBuf[1] = ((uint8_t)filter_Parms.highPassFilterMode << 6) | ((uint8_t)filter_Parms.highPassFilterCutOffFreq << 4) | ((uint8_t)filter_Parms.highPassFilterDataSel << 3) | ((uint8_t)filter_Parms.highPassFilterIntEnable);
	bResult = LIS3DH_StartTransfer(2, txBuf, rxBuf, &arg);
	return bResult;
}

bool LIS3DH_InterruptCtrl(void)
{
	bool bResult = false;
	uint8_t rxBuf[2];
	uint8_t txBuf[2];
	uint8_t arg = NULL;

	txBuf[0] = BUILD_LIS3DH_COMMAND(0,0, LIS3DH_REG_CTRL_REG3); // I1_CLICK I1_IA1 I1_IA2 I1_ZYXDA I1_321DA I1_WTM I1_OVERRUN --
	txBuf[1]= 0x40;  // Interrupt activity 1 driven to INT1 pad
	bResult = LIS3DH_StartTransfer(2, txBuf, rxBuf, &arg);

	txBuf[0] = BUILD_LIS3DH_COMMAND(0,0, LIS3DH_REG_CTRL_REG5); // BOOT FIFO_EN -- -- LIR_INT1 D4D_INT1 LIR_INT2 D4D_INT2
	txBuf[1]= 0x08;  // Interrupt 1 pin latched
	bResult = LIS3DH_StartTransfer(2, txBuf, rxBuf, &arg);

	return bResult;
}

bool LIS3DH_Interrupt1Threshold(uint8_t threshold)
{
	bool bResult = false;
	uint8_t rxBuf[2];
	uint8_t txBuf[2];
	uint8_t arg = NULL;

	txBuf[0] = BUILD_LIS3DH_COMMAND(0,0, LIS3DH_REG_INT1_THS); // 0 THS6 THS5 THS4 THS3 THS2 THS1 THS0
	txBuf[1]= threshold;  // Interrupt activity 1 driven to INT1 pad
	bResult = LIS3DH_StartTransfer(2, txBuf, rxBuf, &arg);

	return bResult;
}

bool LIS3DH_ReadRefrence(uint8_t * reference)
{
	bool bResult = false;
	uint8_t rxBuf[2];
	uint8_t txBuf[2];
	uint8_t arg = NULL;

	txBuf[0] = BUILD_LIS3DH_COMMAND(1,0, LIS3DH_REG_REFERENCE); // 0 THS6 THS5 THS4 THS3 THS2 THS1 THS0
	bResult = LIS3DH_StartTransfer(2, txBuf, rxBuf, &arg);
	*reference = rxBuf[1];

	return bResult;
}

bool LIS3DH_Interrupt1Config(uint8_t config)
{
	bool bResult = false;
	uint8_t rxBuf[2];
	uint8_t txBuf[2];
	uint8_t arg = NULL;

	// e.g. INTx_CFG = 0x4Ah for 6D movement and INT1_CFG = 0xCAh for 6D position, INT1_CFG = 0x2A for interrupt on XYZ high with no 6D
	txBuf[0] = BUILD_LIS3DH_COMMAND(0,0, LIS3DH_REG_INT1_CFG); // AOI 6D ZHIE ZLIE YHIE YLIE XHIE XLIE
	txBuf[1]= config;
	bResult = LIS3DH_StartTransfer(2, txBuf, rxBuf, &arg);

	return bResult;
}

bool LIS3DH_ReadINT1Source(uint8_t * int1Source)
{
	bool bResult = false;
	uint8_t rxBuf[2];
	uint8_t txBuf[2];
	uint8_t arg = NULL;

	txBuf[0] = BUILD_LIS3DH_COMMAND(1,0, LIS3DH_REG_INT1_SRC); // 0 IA ZH ZL YH YL XH XL
	bResult = LIS3DH_StartTransfer(2, txBuf, rxBuf, &arg);
	*int1Source = rxBuf[1];

	return bResult;
}

/*Duration time is measured in N/ODR (ms), where N is the content of the duration register (@10Hz the ODR is 0x2, @25Hz is 0x3, ...)*/
bool LIS3DH_Interrupt1Duration(uint8_t duration)
{
	bool bResult = false;
	uint8_t rxBuf[2];
	uint8_t txBuf[2];
	uint8_t arg = NULL;

	txBuf[0] = BUILD_LIS3DH_COMMAND(0,0, LIS3DH_REG_INT1_DURATION); // 0 D6 D5 D4 D3 D2 D1 D0
	txBuf[1]= duration;
	bResult = LIS3DH_StartTransfer(2, txBuf, rxBuf, &arg);

	return bResult;
}
