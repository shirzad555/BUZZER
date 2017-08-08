/*******************************************************************************
  Filename:       CC26XX_BLEPeripheral.c
  Revised:        $Date: 2016-01-07 16:59:59 -0800 (Thu, 07 Jan 2016) $
  Revision:       $Revision: 44594 $

  Description:    This file contains the Simple BLE Peripheral sample
                  application for use with the CC2650 Bluetooth Low Energy
                  Protocol Stack.

  Copyright 2013 - 2015 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
*******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <movedetectorGATTprofile.h>
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>



#include "hci_tl.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"

#if defined(SENSORTAG_HW)
#include "bsp_spi.h"
#endif // SENSORTAG_HW

#if defined(FEATURE_OAD) || defined(IMAGE_INVALIDATE)
#include "oad_target.h"
#include "oad.h"
#endif //FEATURE_OAD || IMAGE_INVALIDATE

#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "ICallBleAPIMSG.h"

#include "util.h"
//#include "board_lcd.h"
#include "board_key.h"
#include "Board.h"

#include "PINCC26XX.h"

#include <movedetectorSensor.h>

#include <ti/drivers/lcd/LCDDogm1286.h>

#include "led.h"
#include "alarm.h"
#include <LIS3DH_Driver.h>


/*********************************************************************
 * CONSTANTS
 */
// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

#ifndef FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800
#else
// Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8
#endif // FEATURE_OAD

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// How often to perform periodic event (in msec)
#define MDP_PERIODIC_EVT_PERIOD               500  // 5000 Original
#define MDP_LED_BLINK_EVT_PERIOD			  250
#define MDP_SENSOR_MOVE_EVT_PERIOD			  40

#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

// Task configuration
#define MDP_TASK_PRIORITY                     1

#ifndef MDP_TASK_STACK_SIZE
#define MDP_TASK_STACK_SIZE                   644
#endif

// Internal Events for RTOS application
#define MDP_STATE_CHANGE_EVT                  0x0001
#define MDP_CHAR_CHANGE_EVT                   0x0002
#define MDP_PERIODIC_EVT                      0x0004
#define MDP_CONN_EVT_END_EVT                  0x0008

#define MDP_KEY_CHANGE_EVT                    0x0010
#define MDP_LED_BLINK_EVT					  0x0020
#define MDP_SENSOR_MOVE_EVT					  0x0040

#define SENSOR_MOVE_COUNT 						10
#define ALARM_MOVEMENT_THRESHOLD				10
/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
} sbpEvt_t;

/*********************************************************************
 * LOCAL VARIABLES
 */

uint8_t valueForTest = 0;

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;
static Clock_Struct ledBlinkClock;
static Clock_Struct sensorMovementClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

#if defined(FEATURE_OAD)
// Event data from OAD profile.
static Queue_Struct oadQ;
static Queue_Handle hOadQ;
#endif //FEATURE_OAD

// events flag for internal application events.
static uint16_t events;

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[MDP_TASK_STACK_SIZE];

// Profile state and parameters
//static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  17,   // length of this data // 14
  GAP_ADTYPE_LOCAL_NAME_COMPLETE, //CC2650 SensorTag
  'C',
  'C',
  '2',
  '6',
  '5',
  '0',
  ' ',
  'S',
  'e',
  'n',
  's',
  'o',
  'r',
  'T',
  'a',
  'g',

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x11, // Shirzad Original: 0x03,   // length of this data
// Shirzad Original:  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
#ifdef FEATURE_OAD
  LO_UINT16(OAD_SERVICE_UUID),
  HI_UINT16(OAD_SERVICE_UUID)
#else
  GAP_ADTYPE_128BIT_MORE, // Shirzad Original: LO_UINT16(SIMPLEPROFILE_SERV_UUID),
  TI_BASE_UUID_128(MOVEDETECTOR_SERV_UUID), //(SIMPLEPROFILE_SERV_UUID), // Shirzad Original: HI_UINT16(SIMPLEPROFILE_SERV_UUID),
#endif //!FEATURE_OAD
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Project Angela";//"CC2650 SensorTag"; // "Simple BLE Peripheral";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

uint8_t ledBlinkCount = 0;
uint8_t sensorCheckCount = 0;

static uint16_t static_xyzValue[3][SENSOR_MOVE_COUNT];
//static uint16_t static_yValue[SENSOR_MOVE_COUNT];
//static uint16_t static_zValue[SENSOR_MOVE_COUNT];
/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void Movedetector_init( void );
static void Movedetector_taskFxn(UArg a0, UArg a1);

static uint8_t Movedetector_processStackMsg(ICall_Hdr *pMsg);
static uint8_t Movedetector_processGATTMsg(gattMsgEvent_t *pMsg);
static void Movedetector_processAppMsg(sbpEvt_t *pMsg);
static void Movedetector_processStateChangeEvt(
                                                     gaprole_States_t newState);
static void Movedetector_processCharValueChangeEvt(uint8_t paramID);
static void Movedetector_performPeriodicTask(void);

static void Movedetector_sendAttRsp(void);
static void Movedetector_freeAttRsp(uint8_t status);

static void Movedetector_stateChangeCB(gaprole_States_t newState);
#ifndef FEATURE_OAD
static void Movedetector_charValueChangeCB(uint8_t paramID);
#endif //!FEATURE_OAD
static void MovedetectorSensor_enqueueMsg(uint8_t event, uint8_t state);

#ifdef FEATURE_OAD
void Movedetector_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData);
#endif //FEATURE_OAD

static void Movedetector_clockHandler(UArg arg);

void MovedetectorSensor_keyChangeHandler(uint8 keysPressed);
static void MovedetectorSensor_handleKeys(uint8_t shift, uint8_t keys);
static void ReadSensorValue(void);
static void CheckForAlarm(void);
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t Movedetector_gapRoleCBs =
{
  Movedetector_stateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t Movedetector_BondMgrCBs =
{
  NULL, // Passcode callback (not used by application)
  NULL  // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
#ifndef FEATURE_OAD
static movedetectorCBs_t MovedetectorCBs =
{
  Movedetector_charValueChangeCB // Characteristic value change callback
};
#endif //!FEATURE_OAD

#ifdef FEATURE_OAD
static oadTargetCBs_t Movedetector_oadCBs =
{
  Movedetector_processOadWriteCB // Write Callback.
};
#endif //FEATURE_OAD

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Movedetector_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void Movedetector_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = MDP_TASK_STACK_SIZE;
  taskParams.priority = MDP_TASK_PRIORITY;

  Task_construct(&sbpTask, Movedetector_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      Movedetector_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void Movedetector_init(void)
{
	LIS3DH_Filter filter_Parms;
	uint8_t temp;
	filter_Parms.highPassFilterIntEnable = LIS3DH_HF_FILTER_INT_AI1; //LIS3DH_HF_FILTER_INT_NONE; //LIS3DH_HF_FILTER_INT_AI1;
	filter_Parms.highPassFilterDataSel = LIS3DH_HF_FILTER_DATA_SEL_OUT;
	filter_Parms.highPassFilterMode = LIS3DH_HF_FILTER_MODE_NORMAL_RESET;
	filter_Parms.highPassFilterCutOffFreq = LIS3DH_HF_FILTER_CUTOFF_FREQ_3;
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Hard code the BD Address till CC2650 board gets its own IEEE address
  //uint8 bdAddress[B_ADDR_LEN] = { 0xAD, 0xD0, 0x0A, 0xAD, 0xD0, 0x0A };
  //HCI_EXT_SetBDADDRCmd(bdAddress);

  // Set device's Sleep Clock Accuracy
  //HCI_EXT_SetSCACmd(500);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicClock, Movedetector_clockHandler,
                      MDP_PERIODIC_EVT_PERIOD, 0, false, MDP_PERIODIC_EVT); //Original! every time it's fired it's reset again in the app so it becomes periodic
//  Util_constructClock(&periodicClock, Movedetector_clockHandler,
//                      MDP_PERIODIC_EVT_PERIOD, 500, false, MDP_PERIODIC_EVT);
  // Create one-shot clocks for led_blinking events.
  Util_constructClock(&ledBlinkClock, Movedetector_clockHandler,
                      MDP_LED_BLINK_EVT_PERIOD, 0, false, MDP_LED_BLINK_EVT);

  // Create one-shot clocks for sensor events.
  Util_constructClock(&sensorMovementClock, Movedetector_clockHandler,
                      MDP_SENSOR_MOVE_EVT_PERIOD, 0, false, MDP_SENSOR_MOVE_EVT);

#ifndef SENSORTAG_HW
//  Board_openLCD();
#endif //SENSORTAG_HW

#if SENSORTAG_HW
  // Setup SPI bus for serial flash and Devpack interface
  bspSpiOpen();
#endif //SENSORTAG_HW

  // Initialize keys
  Board_initKeys(MovedetectorSensor_keyChangeHandler);

  // Init LED
  Led_init();
  //PINCC26XX_setOutputValue(SPI_CS, 1);
  //PINCC26XX_setOutputEnable(SPI_MISO, 1);
  //PINCC26XX_setOutputValue(SPI_MISO, 0);
  LIS3DH_Initialize();
  LIS3DH_SetDeviceMode(LIS3DH_MODE_LOW_POWER, LIS3DH_ODR_25_HZ, LIS3DH_FULL_SCALE_SELECTION_4G);
  /////////////////////// THIS IS TO SETUP THE HIGH PASS FILTER INTERRUPT /////////////////////////////////////////////////////////////////////////////////
  LIS3DH_SetFilter(filter_Parms);
  LIS3DH_InterruptCtrl();
  LIS3DH_Interrupt1Threshold(0x05); // Best way to adjust sensitivity
  LIS3DH_Interrupt1Duration(2);// you can add duration here too
  LIS3DH_ReadRefrence(&temp); // Dummy read to force the HP filter to current acceleration value  (i.e. set reference acceleration/tilt value)
  LIS3DH_Interrupt1Config(0x2A); // Configure desired wake-up event (AOI 6D ZHIE ZLIE YHIE YLIE XHIE XLIE)
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////// THIS IS TO BYPASS THE HIGH PASS FILTER AND INTERRUPT ON ABSOLUTE VALUES /////////////////////////////////////////////////////////////////////////////////
/*  LIS3DH_SetFilter(filter_Parms);
  LIS3DH_InterruptCtrl();
  LIS3DH_Interrupt1Threshold(0x15); // Best way to adjust sensitivity
  // you can add duration here too
  LIS3DH_ReadRefrence(&temp); // Dummy read to force the HP filter to current acceleration value  (i.e. set reference acceleration/tilt value)
  LIS3DH_Interrupt1Config(0x2A); // Configure desired wake-up event (AOI 6D ZHIE ZLIE YHIE YLIE XHIE XLIE)
*/  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Setup the GAP
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }

  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = 0; // passkey "000000"
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

   // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service

#ifndef FEATURE_OAD
  Movedetector_AddService(GATT_ALL_SERVICES); // Simple GATT Profile
#endif //!FEATURE_OAD

#ifdef FEATURE_OAD
  VOID OAD_addService();                 // OAD Profile
  OAD_register((oadTargetCBs_t *)&Movedetector_oadCBs);
  hOadQ = Util_constructQueue(&oadQ);
#endif

#ifdef IMAGE_INVALIDATE
  Reset_addService();
#endif //IMAGE_INVALIDATE
  
  
#ifndef FEATURE_OAD
  // Setup the SimpleProfile Characteristic Values
  {
    uint8_t charValue1 = 0;
    uint8_t charValue2 = 23;
    uint16_t charValue3 = 0;
//    uint8_t charValue4 = 4;
//    uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };

    Movedetector_SetParameter(MOVEDETECTOR_CHAR1, sizeof(uint8_t),
                               &charValue1);
    Movedetector_SetParameter(MOVEDETECTOR_CHAR2, sizeof(uint8_t),
                               &charValue2);
    Movedetector_SetParameter(MOVEDETECTOR_CHAR3, sizeof(uint16_t),
                               &charValue3);
/*    Movedetector_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                               &charValue4);
    Movedetector_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
                               charValue5);
                               */
  }

  // Register callback with SimpleGATTprofile
  Movedetector_RegisterAppCBs(&MovedetectorCBs); //
#endif //!FEATURE_OAD
  
  // Start the Device
  VOID GAPRole_StartDevice(&Movedetector_gapRoleCBs);
  
  // Start Bond Manager
  VOID GAPBondMgr_Register(&Movedetector_BondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);
  
  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);
  
#if defined FEATURE_OAD
#if defined (HAL_IMAGE_A)
//  LCD_WRITE_STRING("BLE Peripheral A", LCD_PAGE0);
#else
//  LCD_WRITE_STRING("BLE Peripheral B", LCD_PAGE0);
#endif // HAL_IMAGE_A
#else
//  LCD_WRITE_STRING("BLE Peripheral", LCD_PAGE0);
#endif // FEATURE_OAD
}

/*********************************************************************
 * @fn      Movedetector_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void Movedetector_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  Movedetector_init();


  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
//    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);
    ICall_Errno errno = ICall_wait(500);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Event *pEvt = (ICall_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & MDP_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              Movedetector_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = Movedetector_processStackMsg(
                                                             (ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          Movedetector_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
    }
    else if(errno == ICALL_ERRNO_TIMEOUT)
    {
//        Log_info0("Hello World!!");
//        Log_print0(Diags_USER1, "Hello World!!");
    }

    if (events & MDP_PERIODIC_EVT)
    {
      events &= ~MDP_PERIODIC_EVT;

      Util_startClock(&periodicClock);

      // Perform periodic application task
      Movedetector_performPeriodicTask();
    }
    if (events & MDP_LED_BLINK_EVT && ledBlinkCount > 0)
    {
    	events &= ~MDP_LED_BLINK_EVT;
    	ledBlinkCount--;
    	Util_startClock(&ledBlinkClock);
    	Log_print0(Diags_USER1, "Toggling led\r\n");
    	toggle_led();
    	valueForTest++;
    	Movedetector_SetParameter(MOVEDETECTOR_CHAR2, sizeof(uint8_t), &valueForTest);
    }
    if (events & MDP_SENSOR_MOVE_EVT)
    {
    	events &= ~MDP_SENSOR_MOVE_EVT;
    	if(sensorCheckCount > 0)
    	{
    		sensorCheckCount--;
    		Util_startClock(&sensorMovementClock);
    		ReadSensorValue();
    	}
    	else
    	{
    		//sensorCheckCount = SENSOR_MOVE_COUNT;
    		CheckForAlarm();
    	}
    }
    
#ifdef FEATURE_OAD
    while (!Queue_empty(hOadQ))
    {
      oadTargetWrite_t *oadWriteEvt = Queue_dequeue(hOadQ);

      // Identify new image.
      if (oadWriteEvt->event == OAD_WRITE_IDENTIFY_REQ)
      {
        OAD_imgIdentifyWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }
      // Write a next block request.
      else if (oadWriteEvt->event == OAD_WRITE_BLOCK_REQ)
      {
        OAD_imgBlockWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }

      // Free buffer.
      ICall_free(oadWriteEvt);
    }
#endif //FEATURE_OAD
  }
}

/*********************************************************************
 * @fn      Movedetector_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t Movedetector_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;
    
  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = Movedetector_processGATTMsg(
                                                        (gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            break;

          default:
            break;
        }
      }
      break;

    default:
      // do nothing
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      Movedetector_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t Movedetector_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   MDP_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      Movedetector_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.
    
    // Display the opcode of the message that caused the violation.
 //   LCD_WRITE_STRING_VALUE("FC Violated:", pMsg->msg.flowCtrlEvt.opcode,
 //                          10, LCD_PAGE5);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
//    LCD_WRITE_STRING_VALUE("MTU Size:", pMsg->msg.mtuEvt.MTU, 10, LCD_PAGE5);
  }
  else if (pMsg->method == ATT_HANDLE_VALUE_NOTI)// || pMsg->method == ATT_HANDLE_VALUE_IND || pMsg->method == ATT_HANDLE_VALUE_CFM)
  {
	  Log_print0(Diags_USER1, "ATT_HANDLE_VALUE_NOTI\r\n");
    // MTU size updated
//    LCD_WRITE_STRING_VALUE("MTU Size:", pMsg->msg.mtuEvt.MTU, 10, LCD_PAGE5);
  }
  
  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      Movedetector_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void Movedetector_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method,
                          &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);
      
      // We're done with the response message
      Movedetector_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
//      LCD_WRITE_STRING_VALUE("Rsp send retry:", rspTxRetry, 10, LCD_PAGE5);
    }
  }
}

/*********************************************************************
 * @fn      Movedetector_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void Movedetector_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
//      LCD_WRITE_STRING_VALUE("Rsp sent, retry:", rspTxRetry, 10, LCD_PAGE5);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

//      LCD_WRITE_STRING_VALUE("Rsp retry failed:", rspTxRetry, 10, LCD_PAGE5);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      Movedetector_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void Movedetector_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case MDP_STATE_CHANGE_EVT:
      Movedetector_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      break;

    case MDP_CHAR_CHANGE_EVT:
      Movedetector_processCharValueChangeEvt(pMsg->hdr.state);
      break;

    case MDP_KEY_CHANGE_EVT:
    	 MovedetectorSensor_handleKeys(0, pMsg->hdr.state);
      break;

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      Movedetector_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void Movedetector_stateChangeCB(gaprole_States_t newState)
{
	MovedetectorSensor_enqueueMsg(MDP_STATE_CHANGE_EVT, newState);
}

/*********************************************************************
 * @fn      Movedetector_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void Movedetector_processStateChangeEvt(gaprole_States_t newState)
{
#ifdef PLUS_BROADCASTER
  static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN,
                             systemId);

        // Display device address
//        LCD_WRITE_STRING(Util_convertBdAddr2Str(ownAddress), LCD_PAGE1);
//        LCD_WRITE_STRING("Initialized", LCD_PAGE2);
        Log_print0(Diags_USER1, "Initialized\r\n");
      }
      break;

    case GAPROLE_ADVERTISING:
//      LCD_WRITE_STRING("Advertising", LCD_PAGE2);
    	Log_print0(Diags_USER1, "Advertising\r\n");
      break;

#ifdef PLUS_BROADCASTER
    /* After a connection is dropped a device in PLUS_BROADCASTER will continue
     * sending non-connectable advertisements and shall sending this change of
     * state to the application.  These are then disabled here so that sending
     * connectable advertisements can resume.
     */
    case GAPROLE_ADVERTISING_NONCONN:
      {
        uint8_t advertEnabled = FALSE;

        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                           &advertEnabled);

        advertEnabled = TRUE;

        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);

        // Reset flag for next connection.
        firstConnFlag = false;

        Movedetector_freeAttRsp(bleNotConnected);
      }
      break;
#endif //PLUS_BROADCASTER

    case GAPROLE_CONNECTED:
      {
        uint8_t peerAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

        Util_startClock(&periodicClock);

//        LCD_WRITE_STRING("Connected", LCD_PAGE2);
//        LCD_WRITE_STRING(Util_convertBdAddr2Str(peerAddress), LCD_PAGE3);
        Log_print0(Diags_USER1, "Connected\r\n");

        #ifdef PLUS_BROADCASTER
          // Only turn advertising on for this state when we first connect
          // otherwise, when we go from connected_advertising back to this state
          // we will be turning advertising back on.
          if (firstConnFlag == false)
          {
            uint8_t advertEnabled = FALSE; // Turn on Advertising

            // Disable connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);

            // Set to true for non-connectabel advertising.
            advertEnabled = TRUE;

            // Enable non-connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            firstConnFlag = true;
          }
        #endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
//      LCD_WRITE_STRING("Connected Advertising", LCD_PAGE2);
    	Log_print0(Diags_USER1, "Connected Advertising\r\n");
      break;

    case GAPROLE_WAITING:
      Util_stopClock(&periodicClock);

      Movedetector_freeAttRsp(bleNotConnected);

 //     LCD_WRITE_STRING("Disconnected", LCD_PAGE2);

      // Clear remaining lines
 //     LCD_WRITE_STRING("", LCD_PAGE3);
 //     LCD_WRITE_STRING("", LCD_PAGE4);
 //     LCD_WRITE_STRING("", LCD_PAGE5);
      Log_print0(Diags_USER1, "Disconnected\r\n");
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      Movedetector_freeAttRsp(bleNotConnected);

//      LCD_WRITE_STRING("Timed Out", LCD_PAGE2);

      // Clear remaining lines
//      LCD_WRITE_STRING("", LCD_PAGE3);
//      LCD_WRITE_STRING("", LCD_PAGE4);
//      LCD_WRITE_STRING("", LCD_PAGE5);
      Log_print0(Diags_USER1, "Timed Out\r\n");

      #ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        firstConnFlag = false;
      #endif //#ifdef (PLUS_BROADCASTER)
      break;

    case GAPROLE_ERROR:
//      LCD_WRITE_STRING("Error", LCD_PAGE2);
    	Log_print0(Diags_USER1, "Error\r\n");
      break;

    default:
//      LCD_WRITE_STRING("", LCD_PAGE2);
      break;
  }

  // Update the state
  //gapProfileState = newState;
}

#ifndef FEATURE_OAD
/*********************************************************************
 * @fn      Movedetector_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void Movedetector_charValueChangeCB(uint8_t paramID)
{
  MovedetectorSensor_enqueueMsg(MDP_CHAR_CHANGE_EVT, paramID);
}
#endif //!FEATURE_OAD

/*********************************************************************
 * @fn      Movedetector_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void Movedetector_processCharValueChangeEvt(uint8_t paramID)
{
#ifndef FEATURE_OAD
  uint8_t newValue;
  //uint_t bVal;

  switch(paramID)
  {
    case MOVEDETECTOR_CHAR1: //SIMPLEPROFILE_CHAR1:
      Movedetector_GetParameter(MOVEDETECTOR_CHAR1, &newValue);
//      Log_print1(Diags_USER1, "CharValueChangeEvt 1 = %d\r\n", &newValue);

      switch(newValue)
      {
      case LED_STATE_OFF:
		PINCC26XX_setOutputValue(BOARD_LED2, LED_OFF);
		Log_print0(Diags_USER1, "LED is OFF\r\n");
    	break;

      case LED_STATE_ON:
		PINCC26XX_setOutputValue(BOARD_LED2, LED_ON);
		Log_print0(Diags_USER1, "LED is ON\r\n");
    	break;

      case LED_STATE_FLASH_1:
    	  ledBlinkCount = LED_BLINK_COUNT_1;
    	  PINCC26XX_setOutputValue(BOARD_LED2, LED_OFF);
    	  Util_startClock(&ledBlinkClock);
		  Log_print0(Diags_USER1, "LED is flashing in mode 1\r\n");
    	break;

      default:
    	PINCC26XX_setOutputValue(BOARD_LED2, LED_OFF); // Shirzad
    	Log_print0(Diags_USER1, "LED ERROR!!!\r\n");
    	break;

      }
//      LCD_WRITE_STRING_VALUE("Char 1:", (uint16_t)newValue, 10, LCD_PAGE4);
      break;

    case MOVEDETECTOR_CHAR2: //SIMPLEPROFILE_CHAR3:
      //SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &newValue);
      Movedetector_GetParameter(MOVEDETECTOR_CHAR2, &newValue);
//      Log_print1(Diags_USER1, "CharValueChangeEvt 2 = %d\r\n", &newValue);
      Log_print0(Diags_USER1, "CharValueChangeEvt 2\r\n");
//      LCD_WRITE_STRING_VALUE("Char 3:", (uint16_t)newValue, 10, LCD_PAGE4);
      break;

    case MOVEDETECTOR_CHAR3:
      Start_Alarm();
      Log_print0(Diags_USER1, "CharValueChangeEvt 3\r\n");
      break;

    default:
      // should not reach here!
      break;
  }
#endif //!FEATURE_OAD
}

/*********************************************************************
 * @fn      Movedetector_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (MDP_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void Movedetector_performPeriodicTask(void)
{
#ifndef FEATURE_OAD
  uint8_t valueToCopy;

  // Call to retrieve the value of the first characteristic in the service
//  if (ledFlashEnable)
//  {
//	  bVal = PINCC26XX_getOutputValue(BOARD_LED2); // Shirzad
//	  PINCC26XX_setOutputValue(BOARD_LED2, !bVal); // Shirzad
//  }
/*	uint16_t xValue;
	uint16_t yValue;
	uint16_t zValue;*/
//	LIS3DH_ReadDeviceValue(&xValue, &yValue, &zValue);

//	Log_print0(Diags_USER1, "Hello World!!");
/*	Log_print1(Diags_USER1, "X = %d ", xValue);
	Log_print1(Diags_USER1, "Y = %d ", yValue);
	Log_print1(Diags_USER1, "Z = %d\r\n", zValue);*/
	// math below will change the xyz values from 2s complement to 0x00 (-2g) to 0xFF (2g)
//	xValue = xValue & 0xFF; // in 8-bit mode sometimes i have seen 0x100 which is not correct, mask to remove the bug
//	if (xValue > 0x7F) xValue = xValue & 0x7F;
//	else xValue = xValue | 0x80;
/*
	yValue = yValue & 0xFF; // in 8-bit mode sometimes i have seen 0x100 which is not correct, mask to remove the bug
	if (yValue > 0x7F) yValue = yValue & 0x7F;
	else yValue = yValue | 0x80;

	zValue = zValue & 0xFF; // in 8-bit mode sometimes i have seen 0x100 which is not correct, mask to remove the bug
	if (zValue > 0x7F) zValue = zValue & 0x7F;
	else zValue = zValue | 0x80;
*///	Log_print3(Diags_USER1, "XYZ = %x, %d, %d", xValue, yValue, zValue);

//	LIS3DH_ReadINT1Source(&valueToCopy); // Return the event that has triggered the interrupt and clear interrupt
//	Log_print1(Diags_USER1, "INT1_SRC = %x", valueToCopy); // 0 IA ZH ZL YH YL XH XL
	PINCC26XX_setOutputValue(BOARD_LED2, LED_OFF);

  if (Movedetector_GetParameter(MOVEDETECTOR_CHAR1, &valueToCopy) == SUCCESS)
  {
	/*  if (valueToCopy == 0xAA)
	  {

	  }*/
	  //valueForTest++;
	  //valueToCopy = valueForTest;
	    // Call to set that value of the second characteristic in the service.
	    // Note that if notifications of the fourth characteristic have been
	    // enabled by a GATT client device, then a notification will be sent
	    // every time this function is called.
	  //Movedetector_SetParameter(MOVEDETECTOR_CHAR2, sizeof(uint8_t), &valueToCopy);
  }
#endif //!FEATURE_OAD
}


#if defined(FEATURE_OAD)
/*********************************************************************
 * @fn      Movedetector_processOadWriteCB
 *
 * @brief   Process a write request to the OAD profile.
 *
 * @param   event      - event type:
 *                       OAD_WRITE_IDENTIFY_REQ
 *                       OAD_WRITE_BLOCK_REQ
 * @param   connHandle - the connection Handle this request is from.
 * @param   pData      - pointer to data for processing and/or storing.
 *
 * @return  None.
 */
void Movedetector_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData)
{
  oadTargetWrite_t *oadWriteEvt = ICall_malloc( sizeof(oadTargetWrite_t) + \
                                             sizeof(uint8_t) * OAD_PACKET_SIZE);

  if ( oadWriteEvt != NULL )
  {
    oadWriteEvt->event = event;
    oadWriteEvt->connHandle = connHandle;

    oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
    memcpy(oadWriteEvt->pData, pData, OAD_PACKET_SIZE);

    Queue_enqueue(hOadQ, (Queue_Elem *)oadWriteEvt);

    // Post the application's semaphore.
    Semaphore_post(sem);
  }
  else
  {
    // Fail silently.
  }
}
#endif //FEATURE_OAD

/*********************************************************************
 * @fn      Movedetector_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void Movedetector_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      Movedetector_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static void MovedetectorSensor_enqueueMsg(uint8_t event, uint8_t state)
{
  sbpEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbpEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
}


/*********************************************************************
 * @fn      MovedetectorSensor_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void MovedetectorSensor_handleKeys(uint8_t shift, uint8_t keys)
{
  (void)shift;  // Intentionally unreferenced parameter
	LIS3DH_Filter filter_Parms;
	uint8_t temp;
if (sensorCheckCount == 0) // wait for the previous interrupt routine to finish
{
  PINCC26XX_setOutputValue(BOARD_LED2, LED_ON);
  Log_print0(Diags_USER1, "**\r\n");

	filter_Parms.highPassFilterIntEnable = LIS3DH_HF_FILTER_INT_NONE; //LIS3DH_HF_FILTER_INT_NONE; //LIS3DH_HF_FILTER_INT_AI1;
	filter_Parms.highPassFilterDataSel = LIS3DH_HF_FILTER_DATA_SEL_BYPASS;
	filter_Parms.highPassFilterMode = LIS3DH_HF_FILTER_MODE_NORMAL_RESET;
	filter_Parms.highPassFilterCutOffFreq = LIS3DH_HF_FILTER_CUTOFF_FREQ_3;

	/////////////////////// THIS IS TO SETUP THE HIGH PASS FILTER INTERRUPT /////////////////////////////////////////////////////////////////////////////////
	LIS3DH_SetFilter(filter_Parms);
//	LIS3DH_InterruptCtrl();
//	LIS3DH_Interrupt1Threshold(0x05); // Best way to adjust sensitivity
//	LIS3DH_Interrupt1Duration(2);// you can add duration here too
//	LIS3DH_ReadRefrence(&temp); // Dummy read to force the HP filter to current acceleration value  (i.e. set reference acceleration/tilt value)
	LIS3DH_Interrupt1Config(0x00); // Configure desired wake-up event (AOI 6D ZHIE ZLIE YHIE YLIE XHIE XLIE)

	LIS3DH_ReadINT1Source(&temp); // Return the event that has triggered the interrupt and clear interrupt
	sensorCheckCount = SENSOR_MOVE_COUNT;
  Util_startClock(&sensorMovementClock);
}
/*  uint8_t moveDetector;
  Movedetector_GetParameter(MOVEDETECTOR_CHAR1, &moveDetector);
  if (keys & KEY_UP)
  {
    if (moveDetector < 255)
    {
      moveDetector++;
      Movedetector_SetParameter(MOVEDETECTOR_CHAR1, sizeof(uint8_t),
                                   &moveDetector);
    }
  }

  if (keys & KEY_DOWN)
  {
    if (moveDetector > 0)
    {
    	moveDetector--;
    	Movedetector_SetParameter(MOVEDETECTOR_CHAR1, sizeof(uint8_t),
                                   &moveDetector);
    }
  }
  */
//  LCD_WRITE_STRING_VALUE("Sunlight:", (uint16_t)moveDetector, 10, LCD_PAGE5);
  return;
}

/*********************************************************************
 * @fn      MovedetectorSensor_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void MovedetectorSensor_keyChangeHandler(uint8 keysPressed)
{
  MovedetectorSensor_enqueueMsg(MDP_KEY_CHANGE_EVT, keysPressed);
}

/*********************************************************************
*********************************************************************/
/////// Read XYZ and see if the detected movement from HPF is real (not noise) //////////////
void ReadSensorValue(void)
{
	uint16_t xValue;
	uint16_t yValue;
	uint16_t zValue;

	LIS3DH_ReadDeviceValue(&xValue, &yValue, &zValue); // clear the interrupt

	// math below will change the xyz values from 2s complement to 0x00 (-2g) to 0xFF (2g)
	xValue = xValue & 0xFF; // in 8-bit mode sometimes i have seen 0x100 which is not correct, mask to remove the bug
	if (xValue > 0x7F) xValue = xValue & 0x7F;
	else xValue = xValue | 0x80;

	yValue = yValue & 0xFF; // in 8-bit mode sometimes i have seen 0x100 which is not correct, mask to remove the bug
	if (yValue > 0x7F) yValue = yValue & 0x7F;
	else yValue = yValue | 0x80;

	zValue = zValue & 0xFF; // in 8-bit mode sometimes i have seen 0x100 which is not correct, mask to remove the bug
	if (zValue > 0x7F) zValue = zValue & 0x7F;
	else zValue = zValue | 0x80;
//	Log_print3(Diags_USER1, "XYZ_1 = %x, %d, %d", xValue, yValue, zValue);

	static_xyzValue[0][sensorCheckCount] = xValue;
	static_xyzValue[1][sensorCheckCount] = yValue;
	static_xyzValue[2][sensorCheckCount] = zValue;
//	static_xValue[sensorCheckCount] = xValue;
//	static_yValue[sensorCheckCount] = yValue;
//	static_zValue[sensorCheckCount] = zValue;

	// create a periodic task to read XYZ, compare with the previous numbers and interrupt if more change then the threshold
}

static void CheckForAlarm(void)
{
	uint8_t i, j;
	uint8_t bVal;
	LIS3DH_Filter filter_Parms;
	uint8_t lastxyzValue[3] = {static_xyzValue[0][0], static_xyzValue[1][0], static_xyzValue[2][0]};
	uint8_t maxDiff = 0;

	for (i=1; i<SENSOR_MOVE_COUNT; i++)
	{
		for (j=0; j<3; j++)
		{
			if(static_xyzValue[j][i] > lastxyzValue[j]) bVal = static_xyzValue[j][i] - lastxyzValue[j];
			else bVal = lastxyzValue[j] - static_xyzValue[j][i];
			if (bVal > maxDiff)
			{
				maxDiff = bVal;
				lastxyzValue[j] = static_xyzValue[j][i];
				Log_print1(Diags_USER1, "maxDiff = %d", maxDiff);
			}
		}
		Log_print4(Diags_USER1, "s_XYZ_%d = %d, %d, %d", i, static_xyzValue[0][i], static_xyzValue[1][i], static_xyzValue[2][i]);
	}

	if (maxDiff > ALARM_MOVEMENT_THRESHOLD)
	{
		bVal = PINCC26XX_getOutputValue(BOARD_LED1);
		PINCC26XX_setOutputValue(BOARD_LED1, !bVal);
		//PINCC26XX_setOutputValue(BOARD_LED1, LED_ON);
	}

	filter_Parms.highPassFilterIntEnable = LIS3DH_HF_FILTER_INT_AI1; //LIS3DH_HF_FILTER_INT_NONE; //LIS3DH_HF_FILTER_INT_AI1;
	filter_Parms.highPassFilterDataSel = LIS3DH_HF_FILTER_DATA_SEL_OUT;
	filter_Parms.highPassFilterMode = LIS3DH_HF_FILTER_MODE_NORMAL_RESET;
	filter_Parms.highPassFilterCutOffFreq = LIS3DH_HF_FILTER_CUTOFF_FREQ_3;

  //LIS3DH_Initialize();
  //LIS3DH_SetDeviceMode(LIS3DH_MODE_LOW_POWER, LIS3DH_ODR_25_HZ, LIS3DH_FULL_SCALE_SELECTION_4G);
  /////////////////////// THIS IS TO SETUP THE HIGH PASS FILTER INTERRUPT /////////////////////////////////////////////////////////////////////////////////
  LIS3DH_SetFilter(filter_Parms);
  LIS3DH_InterruptCtrl();
  LIS3DH_Interrupt1Threshold(0x05); // Best way to adjust sensitivity
  LIS3DH_Interrupt1Duration(2);// you can add duration here too
  LIS3DH_ReadRefrence(&bVal); // Dummy read to force the HP filter to current acceleration value  (i.e. set reference acceleration/tilt value)
  LIS3DH_Interrupt1Config(0x2A); // Configure desired wake-up event (AOI 6D ZHIE ZLIE YHIE YLIE XHIE XLIE)
}
