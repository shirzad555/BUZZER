

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
    LIS3DH_REG_STATUS_AUX   	= 0x07,
    LIS3DH_REG_ADC_OUT_1_L      = 0x08, // OUT_1_L (08h),
    LIS3DH_REG_ADC_OUT_1_H      = 0x09, // OUT_1_H (09h)
    LIS3DH_REG_ADC_OUT_2_L      = 0x0A, // OUT_2_L (0Ah),
    LIS3DH_REG_ADC_OUT_2_H      = 0x0B, // OUT_2_H (0Bh)
    LIS3DH_REG_ADC_OUT_3_L      = 0x0C, // OUT_3_L (0Ch),
    LIS3DH_REG_ADC_OUT_3_H      = 0x0D, // OUT_3_H (0Dh)
    LIS3DH_REG_INT_COUNTER  	= 0x0E, // INT_COUNTER (0Eh)
    LIS3DH_REG_WHO_AM_I     	= 0x0F, // WHO_AM_I (0Fh)

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

//Table 29. CTRL_REG1 register
//ODR3 ODR2 ODR1 ODR0 LPen Zen Yen Xen

typedef enum //LIS3DH_OPERATING_MODE
{
    LIS3DH_MODE_LOW_POWER		  = 1, 	// data resolution is 8 bit in this mode
	LIS3DH_MODE_MORMAL_POWER	  = 0,		// data resolution is 10 bit in this mode
	LIS3DH_MODE_HIGH_RES_POWER	  = 0,		// data resolution is 12 bit in this mode
	LIS3DH_MODE_POWER_DOWN = 0, // we enter this mode by choosing the LIS3DH_DR_POWER_DWON as data rate in this code
}LIS3DH_OperatingMode_t;

typedef enum //LIS3DH_DATA_RATE
{
    LIS3DH_ODR_POWER_DWON		  = 0,
	LIS3DH_ODR_1_HZ, 				// good for all 3 power modes
	LIS3DH_ODR_10_HZ, 				// good for all 3 power modes
	LIS3DH_ODR_25_HZ, 				// good for all 3 power modes
	LIS3DH_ODR_50_HZ, 				// good for all 3 power modes
	LIS3DH_ODR_100_HZ,				// good for all 3 power modes
	LIS3DH_ODR_200_HZ,				// good for all 3 power modes
	LIS3DH_ODR_400_HZ,				// good for all 3 power modes
	LIS3DH_ODR_LOW_POWER_1600_HZ,	// low power mode only
	LIS3DH_ODR_LOW_POWER_5376_HZ,	// in normal mode the rate is 1344 Hz
}LIS3DH_DataRate_t;

typedef enum //LIS3DH_FULL_SCALE_SELECTION
{
    LIS3DH_FULL_SCALE_SELECTION_2G		  = 0,
	LIS3DH_FULL_SCALE_SELECTION_4G,
	LIS3DH_FULL_SCALE_SELECTION_8G,
	LIS3DH_FULL_SCALE_SELECTION_16G,
}LIS3DH_FullScaleSelection_t;

typedef enum //LIS3DH_HF_FILTER_MODE
{
	LIS3DH_HF_FILTER_MODE_NORMAL_RESET		  = 0, 	//  Normal mode (reset by reading REFERENCE (26h))
	LIS3DH_HF_FILTER_MODE_REF_SIG, 					// Reference signal for filtering
	LIS3DH_HF_FILTER_MODE_NORMAL, 					// Normal mode
	LIS3DH_HF_FILTER_MODE_AUTORESET, 				//  Autoreset on interrupt event
}LIS3DH_HP_FilterMode_t;

typedef enum //LIS3DH_HF_FILTER_CUTOFF_FREQ
{
	LIS3DH_HF_FILTER_CUTOFF_FREQ_0		  = 0,
	LIS3DH_HF_FILTER_CUTOFF_FREQ_1,
	LIS3DH_HF_FILTER_CUTOFF_FREQ_2,
	LIS3DH_HF_FILTER_CUTOFF_FREQ_3,
}LIS3DH_HP_FilterCutOff_t;

typedef enum //LIS3DH_HF_FILTER_DATA_SEL
{
	LIS3DH_HF_FILTER_DATA_SEL_BYPASS		  = 0,  // (0: internal filter bypassed;
	LIS3DH_HF_FILTER_DATA_SEL_OUT,					// data from internal filter sent to output register and FIFO)
}LIS3DH_HP_FilterDataSel_t;

typedef enum //LIS3DH_HF_FILTER_INT_SEL
{
	LIS3DH_HF_FILTER_INT_NONE		  = 0,
	LIS3DH_HF_FILTER_INT_AI1,
	LIS3DH_HF_FILTER_INT_AI2,
	LIS3DH_HF_FILTER_INT_AI1_AI2,
}LIS3DH_HP_FilterIntEnable_t;

typedef struct LIS3DH_Filter {
	LIS3DH_HP_FilterMode_t    		highPassFilterMode;       	/*!< High Pass Filter Mode */
	LIS3DH_HP_FilterCutOff_t  		highPassFilterCutOffFreq;   /*!< Cut off frequency based on power mode, see the datasheet */
	LIS3DH_HP_FilterDataSel_t   	highPassFilterDataSel;		/*!< Filtered data selection. */
	LIS3DH_HP_FilterIntEnable_t		highPassFilterIntEnable;    /*!< High-pass filter enabled for AOI function on interrupt 1/2 */
} LIS3DH_Filter;

enum LIS3DH_FIFOS_MODE
{
    LIS3DH_FIFOS_MODE_BYPASS		  = 0,
	LIS3DH_FIFOS_MODE_FIFO,
	LIS3DH_FIFOS_MODE_STREAM,
	LIS3DH_FIFOS_MODE_STREAM_TO_FIFO,
};

// initialize SPI and talk to the LIS3DH to ensure connectivitiy
// returns true if success - false otherwise
bool LIS3DH_Initialize(void);

bool LIS3DH_StartTransfer(uint8_t len, uint8_t * txBuf, uint8_t * rxBuf, uint8_t * args);

bool LIS3DH_VerifyCommunication(void);
bool LIS3DH_SetDeviceMode(LIS3DH_OperatingMode_t operatingMode, LIS3DH_DataRate_t dataRate, LIS3DH_FullScaleSelection_t fullScaleSelection);
bool LIS3DH_ReadDeviceValue(uint16_t * xValue, uint16_t * yValue, uint16_t * zValue);
bool LIS3DH_SetFilter(LIS3DH_Filter filter_Parms);
bool LIS3DH_InterruptCtrl(void);
bool LIS3DH_Interrupt1Threshold(uint8_t threshold);
bool LIS3DH_ReadRefrence(uint8_t * reference);
bool LIS3DH_Interrupt1Config(uint8_t config);
bool LIS3DH_ReadINT1Source(uint8_t * int1Source);
bool LIS3DH_Interrupt1Duration(uint8_t duration);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_LIS3DH_DRIVER_H_ */
