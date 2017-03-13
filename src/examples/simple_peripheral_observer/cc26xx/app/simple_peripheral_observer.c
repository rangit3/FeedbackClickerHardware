/*
 * Filename: simple_peripheral_observer.c
 *
 * Description: This is the simple_central example modified to receive
 * data over BLE at a high throughput.
 *
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <limits.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "hci_tl.h"
#include "gatt.h"
#include "linkdb.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simple_gatt_profile.h"

#if defined(FEATURE_OAD) || defined(IMAGE_INVALIDATE)
#include "oad_target.h"
#include "oad.h"
#endif //FEATURE_OAD || IMAGE_INVALIDATE

#include "peripheral_observer.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

#include <ti/mw/display/Display.h>
#include "board_key.h"

#include "board.h"

#include "simple_peripheral_observer.h"

/*********************************************************************
 * CONSTANTS
 */

#define PLUS_OBSERVER 							1

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
#else //!FEATURE_OAD
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
#define SBP_PERIODIC_EVT_PERIOD               5000

//======================MY BLE DEFS========================
#ifdef PLUS_OBSERVER

#define MAX_GATEWAY_NAME					  25
// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8
// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  50//8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 5000

// Scan interval in ms
#define DEFAULT_SCAN_INTERVAL                 10

// Scan interval in ms
#define DEFAULT_SCAN_WINDOW                   5

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

#endif //#ifdef PLUS_OBSERVER

#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

// Task configuration
#define SBP_TASK_PRIORITY                     1

#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   644
#endif

// Internal Events for RTOS application
#define SBP_STATE_CHANGE_EVT                  0x0001
#define SBP_CHAR_CHANGE_EVT                   0x0002
#define SBP_PERIODIC_EVT                      0x0004
#define SBP_CONN_EVT_END_EVT                  0x0008

#ifdef PLUS_OBSERVER
#define SBP_KEY_CHANGE_EVT                    0x0010
#define SBP_OBSERVER_STATE_CHANGE_EVT         0x0020

//my events
#define BLEChangeAdvertiseName         0x0100
#define BLEShowDevices         0x0200
#define BLENewGateWayName         0x0300 //TODO CHANGE
#define BLEFindGateWay         0x0400 //TODO CHANGE
#define BLEDiscoverDevices         0x0500
#define BLEStartObserving         0x0600
#define BLEError         0x0700
#define UnHandeled         0x0800

//my defs
#define GREENLED 0
#define REDLED 1

#define FALSEANSWER 0
#define TRUEANSWER 1

#define CLICKERNOHANDLE 0
#define CLICKERWITHHANDLE 1

#endif

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct {
	appEvtHdr_t hdr;  // event header.
#ifdef PLUS_OBSERVER
	uint8 *pData; // event data pointer
#endif
} sbpEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;

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
Char sbpTaskStack[SBP_TASK_STACK_SIZE];

// Profile state and parameters
//static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] = {
		// complete name
		0x13,// length of this data
		GAP_ADTYPE_LOCAL_NAME_COMPLETE, 'P', 'e', 'r', 'i', 'p', 'h', 'e', 'r',
		'a', 'l', 'O', 'b', 's', 'e', 'r', 'v', 'e', 'r',
		// connection interval range
		0x05,// length of this data
		GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE, LO_UINT16(
				DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
		HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL), LO_UINT16(
				DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
		HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

		// Tx power level
		0x02,// length of this data
		GAP_ADTYPE_POWER_LEVEL, 0       // 0dBm
		};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] = {
// Flags; this sets the device to use limited discoverable
// mode (advertises for 30 seconds at a time) instead of general
// discoverable mode (advertises indefinitely)
		0x02,// length of this data
		GAP_ADTYPE_FLAGS,
		DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

		// service UUID, to notify central devices what services are included
		// in this peripheral
#if !defined(FEATURE_OAD) || defined(FEATURE_OAD_ONCHIP)
		0x03,   // length of this data
#else //OAD for external flash
		0x05,  // lenght of this data
#endif //FEATURE_OAD
		GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
#ifdef FEATURE_OAD
		LO_UINT16(OAD_SERVICE_UUID),
		HI_UINT16(OAD_SERVICE_UUID),
#endif //FEATURE_OAD
#ifndef FEATURE_OAD_ONCHIP
		LO_UINT16(SIMPLEPROFILE_SERV_UUID), HI_UINT16(SIMPLEPROFILE_SERV_UUID)
#endif //FEATURE_OAD_ONCHIP
		};

static uint8_t localData[];

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Peripheral Observer";
//try add 2 bits

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

// Screen row
enum {
	ROW_ZERO = 0,
	ROW_ONE = 1,
	ROW_TWO = 2,
	ROW_THREE = 3,
	ROW_FOUR = 4,
	ROW_FIVE = 5,
	ROW_SIX = 6,
	ROW_SEVEN = 7
};

#ifdef PLUS_OBSERVER
// Number of scan results and scan result index
static uint8_t scanRes;

typedef struct {
	char localName[MAX_GATEWAY_NAME];	 		 //!< Device's Name
	uint8_t addrType;            //!< Address Type: @ref ADDRTYPE_DEFINES
	uint8_t addr[B_ADDR_LEN];    //!< Device's Address
	uint8_t nameLength; 	 	 //!< Device name length
} devRecInfo_t;

// Scan result list
static devRecInfo_t devList[DEFAULT_MAX_SCAN_RES];
static bool scanningStarted = FALSE;
static uint8_t deviceInfoCnt = 0;
#endif

const char *AdvTypeStrings[] = { "Connectable undirected",
		"Connectable directed", "Scannable undirected",
		"Non-connectable undirected", "Scan response" };

//============MY Vars==================
static bool release = FALSE;
//static bool release = TRUE;

static bool isClicker = TRUE;
//static bool isClicker = FALSE;

static UInt32 lastTimestamp = 0;

static bool firstUsage = FALSE; //TODO true

static bool foundGateway = TRUE; //TODO false

static unsigned char appID;

static char* deviceID;

static char* lastGateWayName;

static char* secondLastGateWayName;

static UInt32 lastanswer = 2;

static bool waitingForAnswerPress = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimpleBLEPeripheral_init(void);
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg);
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState);
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID);
static void SimpleBLEPeripheral_performPeriodicTask(void);
static void SimpleBLEPeripheral_clockHandler(UArg arg);
void SimpleBLEPeripheral_keyChangeHandler(uint8 keysPressed);
static void SimpleBLEPeripheral_ObserverStateChangeCB(
		gapPeripheralObserverRoleEvent_t *pEvent);

static void SimpleBLEPeripheral_sendAttRsp(void);
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status);

static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState);
#ifndef FEATURE_OAD_ONCHIP
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID);
#endif //!FEATURE_OAD_ONCHIP
static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state,
		uint8_t *pData);

//===============MY BLE FUNCS====================
static void StartAdvertiseMode();
static void StartCentralMode();
static void StopCentralMode();

static void MyBLE_addDeviceInfo(uint8_t *pAddr, uint8_t addrType);
static bool MyBLE_findLocalName(uint8_t *pEvtData, uint8_t dataLen);
static void MyBLE_addDeviceName(uint8_t i, uint8_t *pEvtData, uint8_t dataLen);
static void MyBLE_showDevices();

//===================my funcs=====================
static void MyPrint(const char* str);
static void ChangeBLEName();
static UInt32 GetTime();
static char* GetDeviceID();
static bool isGateWay(uint8_t deviceNum);
static char* GetDeviceNameFromDevList(uint8_t deviceNum);
static bool IsNewGateWayName(uint8_t deviceNum);
static void GetMyHandle(uint8_t deviceNum);
static void GetMyHandleFromFlash();
static void SendAnswer(UInt32 answer);
static void HandleNewQuestion();
static void ChangeAdvertDataArr();
static void LocalDataToBase64(const char* data);
static void Base64ToLocalData(const char* data);

static void GenerateNewName(UInt32 state);
static void TurnOffLeds();
static void TurnOnLed(UInt32 led);
static void HandleError();
static bool IsNewQuestion();
static bool MyHandleHasProblem();

//==================end my funcs====================
#ifdef FEATURE_OAD
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
		uint8_t *pData);
#endif //FEATURE_OAD

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t SimpleBLEPeripheral_gapRoleCBs = {
		SimpleBLEPeripheral_stateChangeCB     // Profile State Change Callbacks
#ifdef PLUS_OBSERVER
		, SimpleBLEPeripheral_ObserverStateChangeCB
#endif

		};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs = {
NULL, // Passcode callback (not used by application)
		NULL  // Pairing / Bonding state Callback (not used by application)
		};

// Simple GATT Profile Callbacks
#ifndef FEATURE_OAD_ONCHIP
static simpleProfileCBs_t SimpleBLEPeripheral_simpleProfileCBs = {
		SimpleBLEPeripheral_charValueChangeCB // Characteristic value change callback
		};
#endif //!FEATURE_OAD_ONCHIP

#ifdef FEATURE_OAD
static oadTargetCBs_t simpleBLEPeripheral_oadCBs =
{
	SimpleBLEPeripheral_processOadWriteCB // Write Callback.
};
#endif //FEATURE_OAD

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_createTask(void) {
	Task_Params taskParams;

	// Configure task
	Task_Params_init(&taskParams);
	taskParams.stack = sbpTaskStack;
	taskParams.stackSize = SBP_TASK_STACK_SIZE;
	taskParams.priority = SBP_TASK_PRIORITY;

	Task_construct(&sbpTask, SimpleBLEPeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_init
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
static void SimpleBLEPeripheral_init(void) {
	// ******************************************************************
	// N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
	// ******************************************************************
	// Register the current thread as an ICall dispatcher application
	// so that the application can send and receive messages.
	ICall_registerApp(&selfEntity, &sem);

#ifdef USE_RCOSC
	RCOSC_enableCalibration();
#endif // USE_RCOSC

	// Create an RTOS queue for message from profile to be sent to app.
	appMsgQueue = Util_constructQueue(&appMsg);

	// Create one-shot clocks for internal periodic events.
	Util_constructClock(&periodicClock, SimpleBLEPeripheral_clockHandler,
	SBP_PERIODIC_EVT_PERIOD, 0, false, SBP_PERIODIC_EVT);

#ifdef PLUS_OBSERVER
	Board_initKeys(SimpleBLEPeripheral_keyChangeHandler);
#endif

	dispHandle = Display_open(Display_Type_UART, NULL); //ZH change to UART for LP UART support

#ifdef PLUS_OBSERVER
	//Setup GAP Observer params
	{
		uint8_t scanRes = DEFAULT_MAX_SCAN_RES;

		GAPRole_SetParameter(GAPROLE_MAX_SCAN_RES, sizeof(uint8_t), &scanRes);

		// Set the GAP Characteristics
		GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION); //how long to scan (in scan state)
		GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION);

		//Set scan interval
		GAP_SetParamValue(TGAP_GEN_DISC_SCAN_INT,
				(DEFAULT_SCAN_INTERVAL) / (0.625)); //period for one scan channel

		//Set scan window
		GAP_SetParamValue(TGAP_GEN_DISC_SCAN_WIND,
				(DEFAULT_SCAN_WINDOW) / (0.625)); //active scanning time within scan interval

	}
#endif

	// Setup the GAP
	GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL,
	DEFAULT_CONN_PAUSE_PERIPHERAL);

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
		GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData),
				advertData);

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
		GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t),
				&pairMode);
		GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t),
				&mitm);
		GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t),
				&ioCap);
		GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t),
				&bonding);
	}

	// Initialize GATT attributes
	GGS_AddService(GATT_ALL_SERVICES);           // GAP
	GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
	DevInfo_AddService();                        // Device Information Service

#ifndef FEATURE_OAD_ONCHIP
	SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile
#endif //!FEATURE_OAD_ONCHIP

#ifdef FEATURE_OAD
	VOID OAD_addService();                 // OAD Profile
	OAD_register((oadTargetCBs_t *)&simpleBLEPeripheral_oadCBs);
	hOadQ = Util_constructQueue(&oadQ);
#endif //FEATURE_OAD

#ifdef IMAGE_INVALIDATE
	Reset_addService();
#endif //IMAGE_INVALIDATE

#ifndef FEATURE_OAD_ONCHIP
	// Setup the SimpleProfile Characteristic Values
	{
		uint8_t charValue1 = 1;
		uint8_t charValue2 = 2;
		uint8_t charValue3 = 3;
		uint8_t charValue4 = 4;
		uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };

		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t),
				&charValue1);
		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8_t),
				&charValue2);
		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t),
				&charValue3);
		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
				&charValue4);
		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
				charValue5);
	}

	// Register callback with SimpleGATTprofile
	SimpleProfile_RegisterAppCBs(&SimpleBLEPeripheral_simpleProfileCBs);
#endif //!FEATURE_OAD_ONCHIP

	// Start the Device
	VOID GAPRole_StartDevice(&SimpleBLEPeripheral_gapRoleCBs);

	// Start Bond Manager
	VOID GAPBondMgr_Register(&simpleBLEPeripheral_BondMgrCBs);

	// Register with GAP for HCI/Host messages
	GAP_RegisterForMsgs(selfEntity);

	// Register for GATT local events and ATT Responses pending for transmission
	GATT_RegisterForMsgs(selfEntity);

	HCI_LE_ReadMaxDataLenCmd();

#if defined FEATURE_OAD
#if defined (HAL_IMAGE_A)
	Display_print0(dispHandle, 0, 0, "BLE Peripheral A");
#else
	Display_print0(dispHandle, 0, 0, "BLE Peripheral B");
#endif // HAL_IMAGE_A
#else
#ifdef PLUS_OBSERVER
	Display_print0(dispHandle, 0, 0, "BLE Peripheral Observer");
#else
	Display_print0(dispHandle, 0, 0, "BLE Peripheral");
#endif
#endif // FEATURE_OAD
}

// Lior's functions
static void gatewayHandleDeviceDiscovered(char* deviceName, int length);
static void handleNextHandles();
static void advertiseQuestion(char question, char* answers);
static void writeResultsForQuestion(char question);
static void clickerHandleDeviceDiscovered(char* deviceName, int length);
static void answerToQuestion(char handle, char counter, char question, char answer);
static void requestForHandle();
static char* readMyMac();

// end of: Lior's functions

/*********************************************************************
 * @fn      SimpleBLEPeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1) {
	// Initialize application
	SimpleBLEPeripheral_init();

	// Lior's Test
	char *myMac = readMyMac();

	requestForHandle();

	// test device discovery
	char testNew[17] = {'C','L','K',0xff};
	testNew[16] = '\0';
	strncpy(testNew+4, myMac,12);
	gatewayHandleDeviceDiscovered(testNew, 16);
	handleNextHandles();
	char gatewayResponse[18] = "GTWO";
	gatewayResponse[17] = '\0';
	gatewayResponse[16] = '0'; // handle
	strncpy(gatewayResponse+4, myMac,12);
	clickerHandleDeviceDiscovered(gatewayResponse,17);

	// ignore: mac already exist
	gatewayHandleDeviceDiscovered(testNew, 16);
	advertiseQuestion('1',"ASDXFGHJ");

	clickerHandleDeviceDiscovered("GTWQ1ASDXFGHJ",13);

	writeResultsForQuestion('1'); // before answer

	answerToQuestion('0','1','1','Y');

	char ansYes1[8] = {'C','L','K','0' /*handle*/,'1' /*count*/,'1' /*q*/,'Y' /*a*/,'\0'};
	gatewayHandleDeviceDiscovered(ansYes1, 7);

	writeResultsForQuestion('1'); // after answer

	char temp[8];
	temp[7] = '\0';

	// error: handle not given
	strncpy(temp, ansYes1, 7);
	temp[3] = '1';
	gatewayHandleDeviceDiscovered(temp, 7);

	// error: wrong counter
	strncpy(temp, ansYes1, 7);
	temp[4] = '0';
	gatewayHandleDeviceDiscovered(temp, 7);

	// ignore: same counter, although message is different - it's client fault
	strncpy(temp, ansYes1, 7);
	temp[6] = 'N';
	gatewayHandleDeviceDiscovered(temp, 7);

	// error: wrong answer
	strncpy(temp, ansYes1, 7);
	temp[4] = '2'; // next counter
	temp[6] = 'G';
	gatewayHandleDeviceDiscovered(temp, 7);

	writeResultsForQuestion('2'); // non answered

	// end of: Lior's Test


	// Application main loop
	for (;;) {
		// Waits for a signal to the semaphore associated with the calling thread.
		// Note that the semaphore associated with a thread is signaled when a
		// message is queued to the message receive queue of the thread or when
		// ICall_signal() function is called onto the semaphore.
		ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

		if (errno == ICALL_ERRNO_SUCCESS) {
			ICall_EntityID dest;
			ICall_ServiceEnum src;
			ICall_HciExtEvt *pMsg = NULL;

			if (ICall_fetchServiceMsg(&src, &dest,
					(void **) &pMsg) == ICALL_ERRNO_SUCCESS) {
				uint8 safeToDealloc = TRUE;

				if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity)) {
					ICall_Stack_Event *pEvt = (ICall_Stack_Event *) pMsg;

					// Check for BLE stack events first
					if (pEvt->signature == 0xffff) {
						if (pEvt->event_flag & SBP_CONN_EVT_END_EVT) {
							// Try to retransmit pending ATT Response (if any)
							SimpleBLEPeripheral_sendAttRsp();
						}
					} else {
						// Process inter-task message
						safeToDealloc = SimpleBLEPeripheral_processStackMsg(
								(ICall_Hdr *) pMsg);
					}
				}

				if (pMsg && safeToDealloc) {
					ICall_freeMsg(pMsg);
				}
			}

			// If RTOS queue is not empty, process app message.
			while (!Queue_empty(appMsgQueue)) {
				sbpEvt_t *pMsg = (sbpEvt_t *) Util_dequeueMsg(appMsgQueue);
				if (pMsg) {
					// Process message.
					SimpleBLEPeripheral_processAppMsg(pMsg);

					// Free the space from the message.
					ICall_free(pMsg);
				}
			}
		}

		if (events & SBP_PERIODIC_EVT) {
			events &= ~SBP_PERIODIC_EVT;

			Util_startClock(&periodicClock);

			// Perform periodic application task
			SimpleBLEPeripheral_performPeriodicTask();
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
 * @fn      Util_convertBytes2Str
 *
 * @brief   Convert bytes to string. Used to print advertising data. 
 *         
 *
 * @param   pData - data
 *
 * @return  Adv/Scan data as a string
 */
char *Util_convertBytes2Str(uint8_t *pData, uint8_t length) {
	uint8_t charCnt;
	char hex[] = "0123456789ABCDEF";
	static char str[(3 * 31) + 1];
	char *pStr = str;

	//*pStr++ = '0';
	//*pStr++ = 'x';

	for (charCnt = 0; charCnt < length; charCnt++) {
		*pStr++ = hex[*pData >> 4];
		*pStr++ = hex[*pData++ & 0x0F];
		*pStr++ = ':';
	}
	pStr = NULL;

	return str;
}

#ifdef PLUS_OBSERVER        
/*********************************************************************
 * @fn      SimpleBLECentral_processRoleEvent
 *
 * @brief   Central role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLEPeripheralObserver_processRoleEvent(
		gapPeripheralObserverRoleEvent_t *pEvent) {
	switch (pEvent->gap.opcode) {

	case GAP_DEVICE_INFO_EVENT: {
		//Print scan response data otherwise advertising data
		if (pEvent->deviceInfo.eventType == GAP_ADRPT_SCAN_RSP) {
			if (MyBLE_findLocalName(pEvent->deviceInfo.pEvtData,
					pEvent->deviceInfo.dataLen)) {
				MyBLE_addDeviceInfo(pEvent->deviceInfo.addr,
						pEvent->deviceInfo.addrType);
				MyBLE_addDeviceName(scanRes - 1, pEvent->deviceInfo.pEvtData,
						pEvent->deviceInfo.dataLen);
			}
			Display_print1(dispHandle, 5, 0, "name found is %s",
					devList[scanRes - 1].localName);
			Display_print1(dispHandle, 4, 0, "Scan Response Addr: %s",
					Util_convertBdAddr2Str(pEvent->deviceInfo.addr));
			Display_print1(dispHandle, 5, 0, "Scan Response Data: %s",
					Util_convertBytes2Str(pEvent->deviceInfo.pEvtData,
							pEvent->deviceInfo.dataLen));
		} else {
			deviceInfoCnt++;

			Display_print2(dispHandle, 6, 0,
					"Advertising Addr: %s Advertising Type: %s",
					Util_convertBdAddr2Str(pEvent->deviceInfo.addr),
					AdvTypeStrings[pEvent->deviceInfo.eventType]);
			Display_print1(dispHandle, 7, 0, "Advertising Data: %s",
					Util_convertBytes2Str(pEvent->deviceInfo.pEvtData,
							pEvent->deviceInfo.dataLen));
		}

		ICall_free(pEvent->deviceInfo.pEvtData);
		ICall_free(pEvent);
	}
		break;

	case GAP_DEVICE_DISCOVERY_EVENT: {
		// discovery complete
		scanningStarted = FALSE;
		deviceInfoCnt = 0;

		//Display_print0(dispHandle, 7, 0, "GAP_DEVICE_DISC_EVENT");
		Display_print1(dispHandle, 5, 0, "Devices discovered: %d",
				pEvent->discCmpl.numDevs);
		Display_print0(dispHandle, 4, 0, "Scanning Off");

		ICall_free(pEvent->discCmpl.pDevList);
		ICall_free(pEvent);

	}
		break;

	default:
		break;
	}
}
#endif

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg) {
	uint8_t safeToDealloc = TRUE;

	switch (pMsg->event) {
#ifdef PLUS_OBSERVER
	case GAP_MSG_EVENT:
		// Process GATT message
		SimpleBLEPeripheralObserver_processRoleEvent(
				(gapPeripheralObserverRoleEvent_t *) pMsg);
		break;
#endif
	case GATT_MSG_EVENT:
		// Process GATT message
		safeToDealloc = SimpleBLEPeripheral_processGATTMsg(
				(gattMsgEvent_t *) pMsg);
		break;

	case HCI_GAP_EVENT_EVENT: {
		// Process HCI message
		switch (pMsg->status) {
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
 * @fn      SimpleBLEPeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg) {
	// See if GATT server was unable to transmit an ATT response
	if (pMsg->hdr.status == blePending) {
		// No HCI buffer was available. Let's try to retransmit the response
		// on the next connection event.
		if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
		SBP_CONN_EVT_END_EVT) == SUCCESS) {
			// First free any pending response
			SimpleBLEPeripheral_freeAttRsp(FAILURE);

			// Hold on to the response message for retransmission
			pAttRsp = pMsg;

			// Don't free the response message yet
			return (FALSE);
		}
	} else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT) {
		// ATT request-response or indication-confirmation flow control is
		// violated. All subsequent ATT requests or indications will be dropped.
		// The app is informed in case it wants to drop the connection.

		// Display the opcode of the message that caused the violation.
		Display_print1(dispHandle, 5, 0, "FC Violated: %d",
				pMsg->msg.flowCtrlEvt.opcode);
	} else if (pMsg->method == ATT_MTU_UPDATED_EVENT) {
		// MTU size updated
		Display_print1(dispHandle, 5, 0, "MTU Size: $d", pMsg->msg.mtuEvt.MTU);
	}

	// Free message payload. Needed only for ATT Protocol messages
	GATT_bm_free(&pMsg->msg, pMsg->method);

	// It's safe to free the incoming message
	return (TRUE);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEPeripheral_sendAttRsp(void) {
	// See if there's a pending ATT Response to be transmitted
	if (pAttRsp != NULL) {
		uint8_t status;

		// Increment retransmission count
		rspTxRetry++;

		// Try to retransmit ATT response till either we're successful or
		// the ATT Client times out (after 30s) and drops the connection.
		status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method,
				&(pAttRsp->msg));
		if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL)) {
			// Disable connection event end notice
			HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);

			// We're done with the response message
			SimpleBLEPeripheral_freeAttRsp(status);
		} else {
			// Continue retrying
			Display_print1(dispHandle, 5, 0, "Rsp send retry: %d", rspTxRetry);
		}
	}
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status) {
	// See if there's a pending ATT response message
	if (pAttRsp != NULL) {
		// See if the response was sent out successfully
		if (status == SUCCESS) {
			Display_print1(dispHandle, 5, 0, "Rsp sent retry: %d", rspTxRetry);
		} else {
			// Free response payload
			GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

			Display_print1(dispHandle, 5, 0, "Rsp retry failed: %d",
					rspTxRetry);
		}

		// Free response message
		ICall_freeMsg(pAttRsp);

		// Reset our globals
		pAttRsp = NULL;
		rspTxRetry = 0;
	}
}

#ifdef PLUS_OBSERVER
/*********************************************************************
 * @fn      SimpleBLECentral_handleKeys
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
static void SimpleBLEPeripheral_handleKeys(uint8_t shift, uint8_t keys) {
	(void) shift;  // Intentionally unreferenced parameter
	if (release) {
	} else { //debug mode
		if (keys & KEY_RIGHT) {
//			if (scanningStarted == TRUE) {
//				StopCentralMode();
//			}
//
//			else {  // in advertise
			ChangeBLEName();
//			}
		}
		if (keys & KEY_LEFT) {
			if (scanningStarted == FALSE) {
				StartCentralMode();
			} else {
				MyBLE_showDevices();
			}
			return;
		}
		return;
	}
}
#endif //#ifdef PLUS_OBSERVER

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg) {
	switch (pMsg->hdr.event) {
	case SBP_STATE_CHANGE_EVT:
		SimpleBLEPeripheral_processStateChangeEvt(
				(gaprole_States_t) pMsg->hdr.state);
		break;

	case SBP_CHAR_CHANGE_EVT:
		SimpleBLEPeripheral_processCharValueChangeEvt(pMsg->hdr.state);
		break;

	case SBP_KEY_CHANGE_EVT:
		SimpleBLEPeripheral_handleKeys(0, pMsg->hdr.state);
		break;

	case SBP_OBSERVER_STATE_CHANGE_EVT:
		SimpleBLEPeripheral_processStackMsg((ICall_Hdr *) pMsg->pData);

		break;

	case BLEChangeAdvertiseName: {
		MyPrint("Process BLESearchAdvertise");
		ChangeBLEName();
	}
		break;

	case BLEShowDevices: {
		MyPrint("Process BLEShowDevices");
		MyBLE_showDevices();
	}
		break;

	case BLEStartObserving: {
		MyPrint("Process BLEStartObserving");
		StartCentralMode();
	}
		break;

	case BLEError: {
		MyPrint("Process BLEError");
		//turn red led
	}
		break;
	default:
		MyPrint("UnProcessed App Event");
		break;
	}

}

#ifdef PLUS_OBSERVER
/*********************************************************************
 * @fn      SimpleBLEPeripheral_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLEPeripheral_keyChangeHandler(uint8 keys) {
	SimpleBLEPeripheral_enqueueMsg(SBP_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ObserverStateChangeCB
 *
 * @brief   Peripheral observer event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static void SimpleBLEPeripheral_ObserverStateChangeCB(
		gapPeripheralObserverRoleEvent_t *pEvent) {

	sbpEvt_t *pMsg;

// Create dynamic pointer to message.
	if ((pMsg = ICall_malloc(sizeof(sbpEvt_t)))) {
		pMsg->hdr.event = SBP_OBSERVER_STATE_CHANGE_EVT;
		pMsg->hdr.state = SUCCESS;

		switch (pEvent->gap.opcode) {
		case GAP_DEVICE_INFO_EVENT: {
			gapDeviceInfoEvent_t *pDevInfoMsg;

			pDevInfoMsg = ICall_malloc(sizeof(gapDeviceInfoEvent_t));
			memcpy(pDevInfoMsg, pEvent, sizeof(gapDeviceInfoEvent_t));

			pDevInfoMsg->pEvtData = ICall_malloc(pEvent->deviceInfo.dataLen);
			memcpy(pDevInfoMsg->pEvtData, pEvent->deviceInfo.pEvtData,
					pEvent->deviceInfo.dataLen);

			pMsg->pData = (uint8 *) pDevInfoMsg;
		}
			break;

		case GAP_DEVICE_DISCOVERY_EVENT: {
			gapDevDiscEvent_t *pDevDiscMsg;

			pDevDiscMsg = ICall_malloc(sizeof(gapDevDiscEvent_t));
			memcpy(pDevDiscMsg, pEvent, sizeof(gapDevDiscEvent_t));

			pDevDiscMsg->pDevList = ICall_malloc(
					(pEvent->discCmpl.numDevs) * sizeof(gapDevRec_t));
			memcpy(pDevDiscMsg->pDevList, pEvent->discCmpl.pDevList,
					(pEvent->discCmpl.numDevs) * sizeof(gapDevRec_t));

			pMsg->pData = (uint8 *) pDevDiscMsg;
		}
			break;

		default:
			break;
		}

		// Enqueue the message.
		Util_enqueueMsg(appMsgQueue, sem, (uint8*) pMsg);
	}

// Free the stack message
	ICall_freeMsg(pEvent);
}

#endif

/*********************************************************************
 * @fn      SimpleBLEPeripheral_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState) {
	SimpleBLEPeripheral_enqueueMsg(SBP_STATE_CHANGE_EVT, newState, NULL);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState) {
#ifdef PLUS_BROADCASTER
	static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

	switch (newState) {
	case GAPROLE_STARTED: {
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
		Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(ownAddress));
		Display_print0(dispHandle, 2, 0, "Initialized");
	}
		break;

	case GAPROLE_ADVERTISING:
		Display_print0(dispHandle, 2, 0, "Advertising");
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

			SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
		}
		break;
#endif //PLUS_BROADCASTER

	case GAPROLE_CONNECTED: {
		linkDBInfo_t linkInfo;
		uint8_t numActive = 0;

		Util_startClock(&periodicClock);

		numActive = linkDB_NumActive();

		// Use numActive to determine the connection handle of the last
		// connection
		if (linkDB_GetInfo(numActive - 1, &linkInfo) == SUCCESS) {
			Display_print1(dispHandle, 2, 0, "Num Conns: %d",
					(uint16_t )numActive);
			Display_print0(dispHandle, 3, 0,
					Util_convertBdAddr2Str(linkInfo.addr));
		} else {
			uint8_t peerAddress[B_ADDR_LEN];

			GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

			Display_print0(dispHandle, 2, 0, "Connected");
			Display_print0(dispHandle, 3, 0,
					Util_convertBdAddr2Str(peerAddress));
		}

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
		Display_print0(dispHandle, 2, 0, "Connected Advertising");
		break;

	case GAPROLE_WAITING:
		Util_stopClock(&periodicClock);
		SimpleBLEPeripheral_freeAttRsp(bleNotConnected);

		Display_print0(dispHandle, 2, 0, "Disconnected");

		// Clear remaining lines
		Display_clearLines(dispHandle, 3, 5);
		break;

	case GAPROLE_WAITING_AFTER_TIMEOUT:
		SimpleBLEPeripheral_freeAttRsp(bleNotConnected);

		Display_print0(dispHandle, 2, 0, "Timed Out");

		// Clear remaining lines
		Display_clearLines(dispHandle, 3, 5);

#ifdef PLUS_BROADCASTER
		// Reset flag for next connection.
		firstConnFlag = false;
#endif //#ifdef (PLUS_BROADCASTER)
		break;

	case GAPROLE_ERROR:
		Display_print0(dispHandle, 2, 0, "Error");
		break;

	default:
		Display_clearLine(dispHandle, 2);
		break;
	}

// Update the state
//gapProfileState = newState;
}

#ifndef FEATURE_OAD_ONCHIP
/*********************************************************************
 * @fn      SimpleBLEPeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID) {
	SimpleBLEPeripheral_enqueueMsg(SBP_CHAR_CHANGE_EVT, paramID, NULL);
}
#endif //!FEATURE_OAD_ONCHIP

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID) {
#ifndef FEATURE_OAD_ONCHIP
	uint8_t newValue;

	switch (paramID) {
	case SIMPLEPROFILE_CHAR1:
		SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newValue);

		Display_print1(dispHandle, 4, 0, "Char 1: %d", (uint16_t )newValue);
		break;

	case SIMPLEPROFILE_CHAR3:
		SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &newValue);

		Display_print1(dispHandle, 4, 0, "Char 3: %d", (uint16_t )newValue);
		break;

	default:
		// should not reach here!
		break;
	}
#endif //!FEATURE_OAD_ONCHIP
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (SBP_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_performPeriodicTask(void) {
#ifndef FEATURE_OAD_ONCHIP
	uint8_t valueToCopy;

// Call to retrieve the value of the third characteristic in the profile
	if (SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &valueToCopy) == SUCCESS) {
		// Call to set that value of the fourth characteristic in the profile.
		// Note that if notifications of the fourth characteristic have been
		// enabled by a GATT client device, then a notification will be sent
		// every time this function is called.
		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
				&valueToCopy);
	}
#endif //!FEATURE_OAD_ONCHIP
}

#ifdef FEATURE_OAD
/*********************************************************************
 * @fn      SimpleBLEPeripheral_processOadWriteCB
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
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
		uint8_t *pData)
{
	oadTargetWrite_t *oadWriteEvt = ICall_malloc( sizeof(oadTargetWrite_t) +
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
 * @fn      SimpleBLEPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_clockHandler(UArg arg) {
// Store the event.
	events |= arg;

// Wake up the application.
	Semaphore_post(sem);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state,
		uint8_t *pData) {
	sbpEvt_t *pMsg;

// Create dynamic pointer to message.
	if ((pMsg = ICall_malloc(sizeof(sbpEvt_t)))) {
		pMsg->hdr.event = event;
		pMsg->hdr.state = state;

		// Enqueue the message.
		Util_enqueueMsg(appMsgQueue, sem, (uint8*) pMsg);
	}
}

//========base 64==============//
// Global variable.
// Note: To change the charset to a URL encoding, replace the '+' and '/' with '*' and '-'
unsigned char charset[]={"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"};

unsigned char revchar(char ch)
{
   if (ch >= 'A' && ch <= 'Z')
      ch -= 'A';
   else if (ch >= 'a' && ch <='z')
      ch = ch - 'a' + 26;
   else if (ch >= '0' && ch <='9')
      ch = ch - '0' + 52;
   else if (ch == '+')
      ch = 62;
   else if (ch == '/')
      ch = 63;
   return(ch);
}

int base64_encode(unsigned char in[], unsigned char out[], int len, int newline_flag)
{
   int idx,idx2,blks,left_over;
   // Since 3 input bytes = 4 output bytes, figure out how many even sets of 3 input bytes
   // there are and process those. Multiplying by the equivilent of 3/3 (int arithmetic)
   // will reduce a number to the lowest multiple of 3.
   blks = (len / 3) * 3;
   for (idx=0,idx2=0; idx < blks; idx += 3,idx2 += 4) {
      out[idx2] = charset[in[idx] >> 2];
      out[idx2+1] = charset[((in[idx] & 0x03) << 4) + (in[idx+1] >> 4)];
      out[idx2+2] = charset[((in[idx+1] & 0x0f) << 2) + (in[idx+2] >> 6)];
      out[idx2+3] = charset[in[idx+2] & 0x3F];
      // The offical standard requires insertion of a newline every 76 chars
      if (!(idx2 % 77) && newline_flag) {
         out[idx2+4] = '\n';
         idx2++;
      }
   }
   left_over = len % 3;
   if (left_over == 1) {
      out[idx2] = charset[in[idx] >> 2];
      out[idx2+1] = charset[(in[idx] & 0x03) << 4];
      out[idx2+2] = '=';
      out[idx2+3] = '=';
      idx2 += 4;
   }
   else if (left_over == 2) {
      out[idx2] = charset[in[idx] >> 2];
      out[idx2+1] = charset[((in[idx] & 0x03) << 4) + (in[idx+1] >> 4)];
      out[idx2+2] = charset[(in[idx+1] & 0x0F) << 2];
      out[idx2+3] = '=';
      idx2 += 4;
   }
   out[idx2] = '\0';
   return(idx2);
}

/*
ADD: Option to strip out newlines
*/
int base64_decode(unsigned char in[], unsigned char out[], int len)
{
   unsigned char ch;
   int idx,idx2,blks,left_over;

   if (in[len-1] == '=')
      len--;
   if (in[len-1] == '=')
      len--;

   blks = (len / 4) * 4;
   for (idx=0,idx2=0; idx2 < blks; idx += 3,idx2 += 4) {
      out[idx] = (revchar(in[idx2]) << 2) + ((revchar(in[idx2+1]) & 0x30) >> 4);
      out[idx+1] = (revchar(in[idx2+1]) << 4) + (revchar(in[idx2+2]) >> 2);
      out[idx+2] = (revchar(in[idx2+2]) << 6) + revchar(in[idx2+3]);
   }
   left_over = len % 4;
   if (left_over == 2) {
      out[idx] = (revchar(in[idx2]) << 2) + ((revchar(in[idx2+1]) & 0x30) >> 4);
      out[idx+1] = (revchar(in[idx2+1]) << 4);
      idx += 2;
   }
   else if (left_over == 3) {
      out[idx] = (revchar(in[idx2]) << 2) + ((revchar(in[idx2+1]) & 0x30) >> 4);
      out[idx+1] = (revchar(in[idx2+1]) << 4) + (revchar(in[idx2+2]) >> 2);
      out[idx+2] = revchar(in[idx2+2]) << 6;
      idx += 3;
   }
   out[idx] = '\0';
   return(idx);
}



//==============================MY BLE FUNCS================================

static void StartAdvertiseMode() {
	StopCentralMode();

	Display_print0(dispHandle, 4, 0, "Starting Advertising");

	uint8_t initialAdvertEnable = TRUE;
	uint8 status;

// Set the GAP Role Parameters
	GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
			&initialAdvertEnable);

	GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

// Start the stack in Peripheral mode.
	status = GAPRole_StartDevice(&SimpleBLEPeripheral_gapRoleCBs);

	if (status == SUCCESS) {
		scanningStarted = TRUE;
		Display_print0(dispHandle, 4, 0, "Advertising Mode is On");
	} else {
		Display_print1(dispHandle, 4, 0, "Advertising failed: %d", status);
	}
}
static void StartCentralMode() {
	uint8 status;

//Start scanning if not already scanning
	if ((scanningStarted == FALSE)) {
		status = GAPObserverRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
		DEFAULT_DISCOVERY_ACTIVE_SCAN,
		DEFAULT_DISCOVERY_WHITE_LIST);

		if (status == SUCCESS) {
			scanningStarted = TRUE;
			Display_print0(dispHandle, 4, 0, "Scanning On");
		} else {
			Display_print1(dispHandle, 4, 0, "Scanning failed: %d", status);
		}

	}
}

static void StopCentralMode() {
	uint8 status;

	if (scanningStarted == TRUE) {
		status = GAPObserverRole_CancelDiscovery();

		if (status == SUCCESS) {
			scanningStarted = FALSE;
			Display_print0(dispHandle, 4, 0, "Scanning Off");
		} else {
			Display_print0(dispHandle, 4, 0, "Scanning Off Fail");
		}
	}

}
void MyBLE_addDeviceInfo(uint8_t *pAddr, uint8_t addrType) {
	uint8_t i;

// If result count not at max
	if (scanRes < DEFAULT_MAX_SCAN_RES) {
		// Check if device is already in scan results
		for (i = 0; i < scanRes; i++) {
			if (memcmp(pAddr, devList[i].addr, B_ADDR_LEN) == 0) {
				return;
			}
		}

		// Add addr to scan result list
		memcpy(devList[scanRes].addr, pAddr, B_ADDR_LEN);
		devList[scanRes].addrType = addrType;

		// Increment scan result count
		scanRes++;
	}
}

static bool MyBLE_findLocalName(uint8_t *pEvtData, uint8_t dataLen) {
	uint8_t adLen;
	uint8_t adType;
	uint8_t *pEnd;

	pEnd = pEvtData + dataLen - 1;

// While end of data not reached
	while (pEvtData < pEnd) {
		// Get length of next data item
		adLen = *pEvtData++;
		if (adLen > 0) {
			adType = *pEvtData;

			// If AD type is for local name
			if ((adType == GAP_ADTYPE_LOCAL_NAME_SHORT)
					|| (adType == GAP_ADTYPE_LOCAL_NAME_COMPLETE)) {
				pEvtData++;
				adLen--;
				// For each local name in list
				if (adLen >= 2 && pEvtData < pEnd) {
					return TRUE;
				}

				// Handle possible erroneous extra byte in advertisement data
				if (adLen == 1) {
					pEvtData++;
				}
			} else {
				// Go to next item
				pEvtData += adLen;
			}
		}
	}
// No name found
	return FALSE;
}

void MyBLE_showDevices() {
	MyPrint("MyBLE_showDevices");
	uint8_t i;
	for (i = 0; i < scanRes; i++) {
		Display_print0(dispHandle, ROW_TWO, 0,
				Util_convertBdAddr2Str(devList[i].addr));
		Display_print1(dispHandle, ROW_THREE, 0, "%s", devList[i].localName);
	}
//Navigate through discovery results
//	if (!scanningStarted && scanRes > 0) {
//		if (scanIdx >= scanRes) {
//			//Display the scan option
////			state = BLE_STATE_BROWSING;
//			scanIdx = 0;
//		} else {
//			//Display next device
////			state = BLE_STATE_BROWSING;
//			scanIdx++;
//		}
//	}
}

static void MyBLE_addDeviceName(uint8_t i, uint8_t *pEvtData, uint8_t dataLen) {
	uint8_t scanRspLen;
	uint8_t scanRspType;
	uint8_t *pEnd;

	pEnd = pEvtData + dataLen - 1;

// While end of data not reached
	while (pEvtData < pEnd) {
		// Get length of next scan response item
		scanRspLen = *pEvtData++;
		if (scanRspLen > 0) {
			scanRspType = *pEvtData;

			// If scan response type is for local name
			if ((scanRspType == GAP_ADTYPE_LOCAL_NAME_SHORT)
					|| (scanRspType == GAP_ADTYPE_LOCAL_NAME_COMPLETE)) {
				//Set name length in the device struct.
				devList[i].nameLength = scanRspLen - 1;
				pEvtData++;
				uint8_t j = 0;

				//Copy device name from the scan response data
				while ((pEvtData < pEnd) && (j < scanRspLen - 1)) {
					devList[i].localName[j] = *pEvtData;
					pEvtData++;
					j++;
				}
			}
		} else {
			// Go to next scan response item
			pEvtData += scanRspLen;
		}
	}
}

//==============================MY FUNCS================================
static void MyPrint(const char* str) {
//	System_printf(str);
//	System_printf("\n");
//	System_flush();
}

void ChangeBLEName() {
	advertData[sizeof(advertData) - 2] = 'a';
// Initialize Advertisement data
	GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
//try this too:
//	attDeviceName[GAP_DEVICE_NAME_LEN-2] = 'a';
//GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);
}

UInt32 GetTime() {
//	lastTimestamp = Timestamp_get32();
//	return lastTimestamp;
	return 0;
}
char* GetDeviceID() {
	return 0;
}

char* GetDeviceNameFromDevList(uint8_t deviceNum) {
	return devList[deviceNum].localName;
}

bool isGateWay(uint8_t deviceNum) {
	return TRUE;
	//check if name contains g in start
}


bool IsNewGateWayName(uint8_t deviceNum) {
	char* name = GetDeviceNameFromDevList(deviceNum);
	//compare to lastGateWayName
	//if true change lastGateWayName to name and secondLastGateWayName to lastGateWayName and return true
	//else false
	return true;	//TODO change
}

static void GetMyHandle(uint8_t deviceNum) {
	char* name = GetDeviceNameFromDevList(deviceNum);
	//check if name contains my mac and if yes than update myhandle
	//write it to flash
}

static void GetMyHandleFromFlash() {
	//update handle if exists in flash
}

static void SendAnswer(UInt32 answer) {
	Display_print1(dispHandle, 2, 0, "Sending answer %d to gateway",(uint16_t )answer);
	lastanswer = answer;
	GenerateNewName(CLICKERWITHHANDLE);
	ChangeBLEName();
	TurnOnLed(GREENLED);
	waitingForAnswerPress = FALSE;
}

static void HandleNewQuestion() {
	if (isClicker) {
		StartCentralMode();
	}
}

static void GenerateNewName(UInt32 state) {
	if (isClicker) {
		if (state == CLICKERWITHHANDLE) {
			//change localData array name to:
			//time,handle,handle
			ChangeAdvertDataArr();
		} else {
			//change localData array name to:
			// 'n'[1],time[2-4],unique id[10-20]
			ChangeAdvertDataArr();
		}
	}

	else {

	}
}

static void ChangeAdvertDataArr() {
	//copy localdata to advertData in the right place
}

static void LocalDataToBase64(const char* data) {
	base64_encode(localData,data,MAX_GATEWAY_NAME,0);
}

static void Base64ToLocalData(const char* data) {
	base64_decode(data,localData,MAX_GATEWAY_NAME);
}

static void TurnOffLeds() {
//turn off all leds
}

static void TurnOnLed(UInt32 led) {
//turn on led for 2 seconds
}

static void HandleError() {
	Display_print0(dispHandle, 4, 0, "ERROR");
	TurnOnLed(REDLED);
}
static bool IsNewQuestion() {
	//check lastGateWayName and secondLastGateWayName and compare questions parts
	return TRUE;
}
static bool MyHandleHasProblem() {
	//check lastGateWayName and secondLastGateWayName and extract array and check handle place

	return FALSE;
}

/*********************************************************************
 *********************************************************************/

// Lior's additions
#define MAX_NUMBER_OF_CLICKERS 200
#define CLICKER "CLK"
static const char messageStart[] = CLICKER;
#define PREFIX_SIZE 4 // CLK (3) + handle (1)
#define MIN_MESSAGE 7 // PREFIX_SIZE +  counter (1) + question number (1) + answer (1)
#define HANDLE_INDEX 3
#define COUNTER_INDEX 4
#define QUESTION_INDEX 5
#define ANSWER_INDEX 6

#define MAC_ADDRESS_SIZE 12
#define UNASWERED 'U'
#define YES_ANS 'Y'
#define NO_ANS 'N'
#define MIN_COUNTER '1'
#define MAX_COUNTER '9'

static const char NO_HANDLE = CHAR_MAX;
static const char MIN_HANDLE_CHAR = '0'; // ==ascii 48 , still have enough for 200 clickers

static char messagesCounterByClicker[MAX_NUMBER_OF_CLICKERS] = {0}; // valid counter is from '1' to '9'
static char questionNumberByClicker[MAX_NUMBER_OF_CLICKERS] = {0};  // no obligation on question index
static char answersByClicker[MAX_NUMBER_OF_CLICKERS] = {UNASWERED};        // answers are 'U' for "Not Answered", 'Y' for Yes, 'N' for No
static char macAdrresses[MAX_NUMBER_OF_CLICKERS][MAC_ADDRESS_SIZE+1];

static int lastMacIndex = -1;
static int lastAssignedHandleIndex = -1;

static char tempMacAddress[MAC_ADDRESS_SIZE+1];

// prefix size is the same : 3+command
#define OFFER_HANDLE 'O'
#define GATEWAY_COMMAND 3
#define OFFER_MESSAGE_LENGTH (PREFIX_SIZE + MAC_ADDRESS_SIZE + 1/*handle*/)

// 2^8 > 200
#define NUMBER_OF_CHARS_FOR_ALL_CLICKERS 8
#define QUESTION 'Q'
#define QUESTION_MESSAGE_LENGTH (PREFIX_SIZE + 1 +NUMBER_OF_CHARS_FOR_ALL_CLICKERS)

static char tempDeviceNameForHandleOffering[OFFER_MESSAGE_LENGTH+1] = {'G','T','W','O'};
static char tempDeviceNameForQuestion[QUESTION_MESSAGE_LENGTH+1] = {'G','T','W','Q'};


// Gateway code
static void gatewayHandleDeviceDiscovered(char* deviceName, int length){ // length without null-terminate
	if(length < MIN_MESSAGE){
		return; // not relevant name
	}

	for(int i = 0 ; i < sizeof(messageStart) -1 /*null-terminate*/ ; i++, length--){
		if(messageStart[i] != deviceName[i]){
			return; // not relevant
		}
	}

	char handleAsChar = deviceName[HANDLE_INDEX];
	length--;

	if(handleAsChar == NO_HANDLE){ // new clicker - try add to mac addresses
		if(length != MAC_ADDRESS_SIZE){
			// ERROR !!!
			Display_print3(dispHandle, 5, 0, "ERROR found in device name: '%s' , the length after prefix (%d) is not equal to MAC address length (%d) !!! \n", deviceName, length, MAC_ADDRESS_SIZE);
			return;
		}

		strncpy(tempMacAddress , deviceName+PREFIX_SIZE, MAC_ADDRESS_SIZE);
		tempMacAddress[MAC_ADDRESS_SIZE] = '\0'; // add null-terminate
		// search all MACs already allocated

		for(int i = 0 ; i <= lastMacIndex; i++){
			if(strncmp(tempMacAddress , macAdrresses[i], MAC_ADDRESS_SIZE) == 0){
				Display_print1(dispHandle, 5, 0, "DEBUG: mac address '%s' already exist \n", tempMacAddress);
				return; // already in the list
			}
		}

		if(lastAssignedHandleIndex >= MAX_NUMBER_OF_CLICKERS - 1){
			// ERROR!!!
			Display_print1(dispHandle, 5, 0, "Reached max clickers index (%d), can't assign anymore", lastAssignedHandleIndex);
			return;
		}

		// not in list - add it and increment counter
		lastMacIndex++;
		strncpy(macAdrresses[lastMacIndex] , tempMacAddress, MAC_ADDRESS_SIZE);
		macAdrresses[lastMacIndex][MAC_ADDRESS_SIZE] = '\0'; // add null-terminate

		Display_print2(dispHandle, 5, 0, "MAC was added by device name: '%s' at index %d \n", deviceName, lastMacIndex);
	}
	// should be valid counter
	else{
		char lastHandleChar = MIN_HANDLE_CHAR+lastAssignedHandleIndex;
		if(handleAsChar < MIN_HANDLE_CHAR || handleAsChar > lastHandleChar) {
			// ERROR
			Display_print4(dispHandle, 5, 0, "ERROR found in device name: '%s' , the given handle ('%c') is not between range ('%c'-'%c') !!! \n", deviceName, handleAsChar, MIN_HANDLE_CHAR, lastHandleChar);
			return;
		}
		// find counter
		char counter = deviceName[COUNTER_INDEX];
		if(counter < MIN_COUNTER || counter > MAX_COUNTER) {
			// ERROR
			Display_print4(dispHandle, 5, 0, "ERROR found in device name: '%s' , the given counter ('%c') is not legal (not '%c'-'%c') !!! \n", deviceName, counter, MIN_COUNTER, MAX_COUNTER);
			return;
		}

		int handle = (int)(handleAsChar-MIN_HANDLE_CHAR); // make it integer

		if(messagesCounterByClicker[handle] == counter){ // is same ?
			Display_print2(dispHandle, 5, 0, "DEBUG: for device name: '%s' , the given counter ('%c') is already stored \n", deviceName, counter);
			return;
		}

		// new - update
		messagesCounterByClicker[handle] = counter;
		char question = deviceName[QUESTION_INDEX];
		questionNumberByClicker[handle] = question;

		char answer = deviceName[ANSWER_INDEX];
		if(answer != YES_ANS && answer != NO_ANS){
			answersByClicker[handle] = UNASWERED;
			// ERROR
			Display_print4(dispHandle, 5, 0, "ERROR found in device name: '%s' , the answer given ('%c') is not legal (not '%c' nor '%c') !!! \n", deviceName, answer, YES_ANS, NO_ANS);
			return;
		}

		answersByClicker[handle] = answer;
		Display_print4(dispHandle, 5, 0, "Answer was added by device name: '%s', handle number %d question '%c' answer '%c' \n", deviceName, handle, question, answer);
	}

}

static void handleNextHandles() {

	for(int i = lastAssignedHandleIndex+1 ; i <= lastMacIndex ;i++,lastAssignedHandleIndex++){
		strncpy(tempDeviceNameForHandleOffering+PREFIX_SIZE, macAdrresses[i],MAC_ADDRESS_SIZE);
		tempDeviceNameForHandleOffering[OFFER_MESSAGE_LENGTH-1] = i+MIN_HANDLE_CHAR;
		tempDeviceNameForHandleOffering[OFFER_MESSAGE_LENGTH] =  '\0';
		Display_print3(dispHandle, 5, 0, "Next handle %d should be assigned to mac %s , full name should be %s \n", i, macAdrresses[i],tempDeviceNameForHandleOffering);
		// do the procedure of name change
	}

}

static void advertiseQuestion(char question, char* answers){
	tempDeviceNameForQuestion[PREFIX_SIZE] = question;
	strncpy(tempDeviceNameForQuestion+PREFIX_SIZE+1, answers, NUMBER_OF_CHARS_FOR_ALL_CLICKERS);
	tempDeviceNameForQuestion[QUESTION_MESSAGE_LENGTH] = '\0';
	Display_print3(dispHandle, 5, 0, "Advertising question ('%c') and answers ('%s') , full name should be %s \n", question, answers, tempDeviceNameForQuestion);
	// do the procedure of name change
}


static void writeResultsForQuestion(char question) {
	int numYes=0;
	int numNo=0;
	for(int i = 0 ; i <= lastAssignedHandleIndex; i++){
		char questionForClicker = questionNumberByClicker[i];
		if(questionForClicker == question){
			char answer = answersByClicker[i];
			if(answer == YES_ANS){
				numYes++;
			}else if(answer == NO_ANS){
				numNo++;
			}
		}
	}

	int notAnswered = lastAssignedHandleIndex + 1 - numYes - numNo;

	Display_print4(dispHandle, 5, 0, "Results for question '%c': YES = %d , NO = %d , NOT ANSWERERD = %d \n", question, numYes, numNo, notAnswered);
}


static char lastQuestion = '\0';
static char lastAnswers[NUMBER_OF_CHARS_FOR_ALL_CLICKERS+1] = {0};
static char lastHandle = '\0';
static char myMac[MAC_ADDRESS_SIZE+1];

// clicker code
static void clickerHandleDeviceDiscovered(char* deviceName, int length){ // length without null-terminate
	// start is the same
	if(length < PREFIX_SIZE){
		return; // not relevant name
	}

	char command = deviceName[GATEWAY_COMMAND];
	if(command == OFFER_HANDLE){
		if(length != OFFER_MESSAGE_LENGTH){
			Display_print3(dispHandle, 5, 0, "DEBUG: in name: '%s' , the length for OFFER HANDLE (%d) is not equal to requested (%d) !!! \n", deviceName, length, OFFER_MESSAGE_LENGTH);
			return;
		}

		for(int i = 0 ; i < PREFIX_SIZE ; i++){
			if(tempDeviceNameForHandleOffering[i] != deviceName[i]){
				Display_print1(dispHandle, 5, 0, "DEBUG: in name: '%s' , prefix for OFFER HANDLE is not as requested ('%s') !!! \n", deviceName);
				return;
			}
		}

		strncpy(tempMacAddress , deviceName+PREFIX_SIZE, MAC_ADDRESS_SIZE);
		tempMacAddress[MAC_ADDRESS_SIZE] = '\0'; // add null-terminate
		if(strncmp(tempMacAddress, myMac, MAC_ADDRESS_SIZE)==0){
			// my mac - get handle
			lastHandle = deviceName[OFFER_MESSAGE_LENGTH-1];
			Display_print3(dispHandle, 5, 0, "Handle '%c' is assigned to mac '%s' by device name '%s'! \n",lastHandle, myMac, deviceName);
		}


	}
	else if(command == QUESTION){
		if(length != QUESTION_MESSAGE_LENGTH){
			Display_print3(dispHandle, 5, 0, "DEBUG: in name: '%s' , the length for QUESTION MESSAGE (%d) is not equal to requested (%d) !!! \n", deviceName, length, QUESTION_MESSAGE_LENGTH);
			return;
		}

		for(int i = 0 ; i < PREFIX_SIZE ; i++){
			if(tempDeviceNameForQuestion[i] != deviceName[i]){
				Display_print1(dispHandle, 5, 0, "DEBUG: in name: '%s' , prefix for QUESTION MESSAGE is not as requested ('%s') !!! \n", deviceName);
				return;
			}
		}

		// update question and numbers
		lastQuestion = deviceName[PREFIX_SIZE];
		strncpy(lastAnswers, deviceName+1+PREFIX_SIZE, NUMBER_OF_CHARS_FOR_ALL_CLICKERS);
		lastAnswers[NUMBER_OF_CHARS_FOR_ALL_CLICKERS] = '\0'; // add null-terminate
		Display_print3(dispHandle, 5, 0, "last question '%c' and answers '%s' by device name '%s'! \n",lastQuestion, lastAnswers, deviceName);



	}
	else{
		return; // not relevant
	}
}

static char tempAnswerForQuestion[MIN_MESSAGE+1]= {CLICKER};

static void answerToQuestion(char handle, char counter, char question, char answer){
	tempAnswerForQuestion[HANDLE_INDEX] = handle;
	tempAnswerForQuestion[COUNTER_INDEX] = counter;
	tempAnswerForQuestion[QUESTION_INDEX] = question;
	tempAnswerForQuestion[ANSWER_INDEX] = answer;
	tempAnswerForQuestion[MIN_MESSAGE] = '\0'; // add null-terminate
	Display_print5(dispHandle, 5, 0, "answer to question %c' with answer '%c' by handle '%c' and counter '%c'. device name is '%s'! \n",question, answer, handle,counter, tempAnswerForQuestion);

	// do the procedure of name change
}

static char tempRequestHandle[PREFIX_SIZE + MAC_ADDRESS_SIZE +1] = {CLICKER};

static void requestForHandle(){
	tempRequestHandle[HANDLE_INDEX] = NO_HANDLE;
	strncpy(tempRequestHandle+PREFIX_SIZE, myMac, MAC_ADDRESS_SIZE);
	tempRequestHandle[PREFIX_SIZE + MAC_ADDRESS_SIZE] = '\0'; // add null-terminate
	Display_print2(dispHandle, 5, 0, "Request for handle by mac '%s'. device name is '%s'! \n", myMac, tempRequestHandle);

	// do the procedure of name change
}


/*
 *  ======== empty.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
//#include <ti/sysbios/knl/Task.h>
//#include <ti/sysbios/knl/Semaphore.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/SPI.h>

/* Board Header files */
//#include "Board.h"

static PIN_Config BoardFlashPinTable[] = {
Board_SPI_FLASH_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL
        | PIN_DRVSTR_MIN, /* Ext. flash chip select */

PIN_TERMINATE };

static PIN_Handle hFlashPin = NULL;
static SPI_Handle spiHandle = NULL;

#define RDID_REMS_CODE 0x90 /* Manufacturer Device ID */

void taskFucntion(UArg arg0, UArg arg1) {
    System_printf("task started\n");
    System_flush();

    uint8_t wbuf[] = { RDID_REMS_CODE, 0x0, 0x0, 0x0 };

    // chip select
    PIN_setOutputValue(hFlashPin, Board_SPI_FLASH_CS, Board_FLASH_CS_ON);

    SPI_Transaction masterTransaction;

    masterTransaction.count = sizeof(wbuf);
    masterTransaction.txBuf = (void*) wbuf;
    masterTransaction.arg = NULL;
    masterTransaction.rxBuf = NULL;

    Bool success = SPI_transfer(spiHandle, &masterTransaction);

    if (!success) {
        // chip select off
        PIN_setOutputValue(hFlashPin, Board_SPI_FLASH_CS, Board_FLASH_CS_OFF);
        System_abort("failure in spi write\n");
    }

    UChar infoBuf[2];
    masterTransaction.count = sizeof(infoBuf);
    masterTransaction.txBuf = NULL;
    masterTransaction.arg = NULL;
    masterTransaction.rxBuf = (void*) infoBuf;

    success = SPI_transfer(spiHandle, &masterTransaction);

    // chip select off anyhow
    PIN_setOutputValue(hFlashPin, Board_SPI_FLASH_CS, Board_FLASH_CS_OFF);

    if (!success) {
        System_abort("failure in spi read\n");
    }

    /*
     print should be:

     .manfId = MF_MACRONIX = C2,  // Macronics
     .devId = 0x14,          // MX25R8035F
     .deviceSize = 0x100000  // 1 MByte (8 Mbit)
     */

    int i = 0;
    for (; i < 2; i++) {
        System_printf("read buffer @ index %d, value is 0x%x\n", i, infoBuf[i]);
        System_flush();
    }
}
/*
 *  ======== main ========
 */
int main2(void) {
    //Task_Params taskParams;

    /* Call board init functions */
    Board_initGeneral()
    ;
    Board_initSPI();

    Task_Handle task = Task_create(taskFucntion, NULL, NULL);
    if (task == NULL) {
        System_abort("Task create failed\n");
    }

    PIN_State pinState;
    hFlashPin = PIN_open(&pinState, BoardFlashPinTable);

    if (hFlashPin == NULL) {
        System_abort("Pin Open failed\n");
    }

    SPI_Params spiParams;
    SPI_Params_init(&spiParams);
    spiParams.mode = SPI_MASTER;
    spiParams.transferMode = SPI_MODE_BLOCKING;

    /* Attempt to open SPI. */
    spiHandle = SPI_open(Board_SPI0, &spiParams);
    if (spiHandle == NULL) {
        /* Error opening SPI */
        System_abort("Error initializing board spi\n");
    }

    //System_printf("Starting the example\nSystem provider is set to SysMin. "
    //        "Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    //System_flush();
    /* Start BIOS */
    BIOS_start();

    return (0);
}

// internal flash
//#include "driverlib/flash.h"
#include "inc/hw_fcfg1.h"
#include <stdio.h>

// mac is 6 bytes
static char* readMyMac(){
	uint32_t bleAddrlsb;

	uint32_t bleAddrmsb;

	bleAddrlsb = HWREG(FCFG1_BASE + FCFG1_O_MAC_BLE_0) ;
	bleAddrmsb  =HWREG(FCFG1_BASE + FCFG1_O_MAC_BLE_1);

	Display_print2(dispHandle, 5, 0, "my mac is '%x','%x'. \n", bleAddrmsb, bleAddrlsb);

	char tempAddress[9];
	sprintf(tempAddress,"%x", bleAddrmsb);
	strncpy(myMac, tempAddress+4, 4); // msb is only 2 bytes of data
	sprintf(tempAddress,"%x", bleAddrlsb);
	strncpy(myMac+4, tempAddress, 8);

	Display_print1(dispHandle, 5, 0, "my mac address as chars is '%s'\n", myMac);

	return myMac;
}

