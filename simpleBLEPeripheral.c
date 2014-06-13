/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "hal_i2c.h"
#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"
#include "thermometer.h"
#include "thermometerservice.h"
#include "simplekeys.h"
#include "humidityservice.h"
#include "linkdb.h"
#include "battservice.h"
#include "IBeacon.h"
#include "osal_snv.h"
#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"
#include "hal_humi.h"
#include "hal_sensor.h"
#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
//ghostyu bond
static uint8 gPairStatus=0;
static uint8 testx=0;
// How often to check battery voltage (in ms)
#define BATTERY_CHECK_PERIOD                  10000
// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                   5000

#define HID_HIGH_ADV_INT_MIN                  32
#define HID_HIGH_ADV_INT_MAX                  48
// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          800  //500ms¼s¼½¶¡¹j

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

//#if defined ( CC2540_MINIDK )
//#define// DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
//#else
//#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
//#endif  // defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
/*
// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL    800

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000
*/
// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6
//*********************³s½u¶¡¹j
//#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     400        // 1.25ms*200 = 500ms
//#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800    // 1.25ms*800 = 1000ms
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     1400        // 1.25ms*200 = 500ms
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     1500    // 1.25ms*800 = 1000ms
#define DEFAULT_DESIRED_SLAVE_LATENCY         0
#define DEFAULT_DESIRED_CONN_TIMEOUT          600           // 10ms*600 = 6s

#define GAPROLE_NO_ACTION                                   0 // Take no action upon unsuccessful parameter updates
#define GAPROLE_RESEND_PARAM_UPDATE          1 // Continue to resend request until successful update
#define GAPROLE_TERMINATE_LINK                         2 // Terminate link upon unsuccessful parameter updates
// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif
#define DEFAULT_DISCOVERY_DELAY               1000
// Common values for turning a sensor on and off + config/status
#define ST_CFG_SENSOR_DISABLE                 0x00
#define ST_CFG_SENSOR_ENABLE                  0x01
#define ST_CFG_CALIBRATE                      0x02
#define ST_CFG_ERROR                          0xFF

// System reset
#define ST_SYS_RESET_DELAY                    3000
// Test mode bit
#define TEST_MODE_ENABLE                      0x80
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
//static uint8  i,j;
//static uint8  x[2];
//static uint8  y[2];
//static uint8  uuid[16];
//static uint8  advuuid[16];
#define  longadv 2400;
#define  shortadv 800;
static uint8  key = false;
static uint8  battery = 0;
 uint8 thermometer_TaskID;   // Task ID for internal task/event processing
//SensorArray
static uint8 Day = 0;
static uint8 Hour = 0;
static uint8 Minute = 0;
static float SRT[3][24] = 0;       //Sensor Record of Temperature
static uint16 SRH[3][24] = 0;      //Sensor Record of Humidity
static float MAX_Temperature = 0;  
static float MIN_Temperature = 100;
static uint16 MAX_Humidity = 0;
static uint16 MIN_Humidity = 100;
static float Average_Temperature = 0;
static uint16 Average_Humidity = 0;
static uint16 Total_Humidity = 0;
static float Total_Temperature = 0;
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

gaprole_States_t gapProfileState = GAPROLE_INIT;

static uint8 LED2PWMLevel = PP_ALERT_LEVEL_NO;     
static uint8 LED3PWMLevel = PP_ALERT_LEVEL_NO; 
static uint8 keyfobProxIMAlertLevel = PP_ALERT_LEVEL_NO;   
static uint8 charValue5[CHAR5_LEN];
// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  0x19,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x51,   // 'Q'
  0x75,   // 'u'
  0x61,   // 'a'
  0x6e,   // 'n'
  0x20,   // ' '
  0x54,   // 'T'
  0x68,   // 'h'
  0x65,   // 'e'
  0x72,   // 'r'
  0x6d,   // 'm'
  0x61,   // 'a'
  0x6c,   // 'l'
  0x6d,   // 'm'
  0x65,   // 'e'
  0x74,   // 't'
  0x65,   // 'e'
  0x72,   // 'r'
  0x33,   // '3'
  
  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
  
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{

 /*
  0x02,   // length of this data
  0x01,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,//1A,   
  0x1A,
  0xFF,
  0x4C,0x00,0x02,0x15,
  //0xE2,0x0A,0x39,0xF4,0x73,0xF5,0x4B,0xC4,0xA1,0x2F,0x17,0xD1,0xAD,0x07,0xA9,0x61,
  //0xe2,0xc5,0x6d,0xb5,0xdf,0xfb,0x48,0xd2,0xb0,0x60,0xd0,0xf5,0xa7,0x10,0x96,0xe0, 
  0xe2,0xc5,0x6d,0xb5,0xdf,0xfb,0x48,0xd2,0xb0,0x60,0xd0,0xf5,0xa7,0x10,0x96,0xe0, 
  0x00,0x05,0x00,0x02,0xC5
 
  */
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID ),
 
};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Quan Thermalmeter3";
//static bool connectedToLastAddress = false;

// Sensor State Variables
static bool   humiEnabled = FALSE;
static uint8  humiState = 0;
static uint16 selfTestResult = 0;
//static bool   testMode = FALSE;
// GAP connection handle
uint16 gapConnHandle;
//*********************************************************************
static void PasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,uint8 uiInputs, uint8 uiOutputs );
static void PairStateCB( uint16 connHandle, uint8 state, uint8 status );
// TRUE if pairing started
//static uint8 timeAppPairingStarted = FALSE;

// Bonded state
//static bool timeAppBonded = FALSE;

// TRUE if discovery postponed due to pairing
//static uint8 timeAppDiscPostponed = FALSE;

// Service discovery complete
//static uint8 timeAppDiscoveryCmpl = FALSE;

// Bonded peer address
//static uint8 timeAppBondedAddr[B_ADDR_LEN];

// Last connection address
//static uint8 lastConnAddr[B_ADDR_LEN] = {0xf,0xf,0xf,0xf,0xf,0xe};;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
//static void performPeriodicTask( void );
static void simpleProfileChangeCB( uint8 paramID );

static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys );

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
static char *bdAddr2Str ( uint8 *pAddr );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

static void thermometerCB(uint8 event);
//tatic void hidAdvRemoteAirMouseCursorCtlKeyPress( void );
//senser
static void readHumData( void );
static void humidityChangeCB( uint8 paramID);
static void resetSensorSetup( void );
static void resetCharacteristicValue(uint16 servID, uint8 paramID, uint8 value, uint8 paramLen);
static void resetCharacteristicValues();
//static void iBeaconCB( uint8 attrParamID );
static void SensorRecord( void );
static void ReadRecord( void );
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  PasscodeCB,
  PairStateCB,                     
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};
//senser
static humidityCBs_t sensorTag_HumidCBs =
{
  humidityChangeCB,         // Characteristic value change callback
};
static proxReporterCBs_t keyFob_ProximityCBs =
{
  NULL,//iBeaconCB,              // Whenever the Link Loss Alert attribute changes
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEPeripheral_Init( uint8 task_id )
{
  /*    UUID±q°O¾ÐÅéªÅ¶¡¨ú¥X¨Ã¥B¨ú¥N­ì¥ýªº­È
  VOID osal_snv_read( 0x82, 16, advuuid );
  
   advertData[9]=advuuid[0];
   advertData[10]=advuuid[1];
   advertData[11]=advuuid[2];
   advertData[12]=advuuid[3];
   advertData[13]=advuuid[4];
   advertData[14]=advuuid[5];
   advertData[15]=advuuid[6];
   advertData[16]=advuuid[7];
  
   advertData[17]=advuuid[8];
   advertData[18]=advuuid[9];
   advertData[19]=advuuid[10];
   advertData[20]=advuuid[11];
   advertData[21]=advuuid[12];
   advertData[22]=advuuid[13];
   advertData[23]=advuuid[14];
   advertData[24]=advuuid[15];

  
  VOID osal_snv_read( 0x82, 16, advuuid );
   //°j°é¶ë­È
  for(i=9;i<=24;i++)
  {
      for(j=0;j<i;j++)
      {
      j=i-9;
      advertData[i]=advuuid[j];
      j=i+1;
      }
  }
  //¥D­n­È
  VOID osal_snv_read( 0x80, 1, y );
  advertData[26]=y[0];
  //¦¸­n­È
  VOID osal_snv_read( 0x81, 2, y );
  advertData[28]=y[1];
*/
  simpleBLEPeripheral_TaskID = task_id;
  thermometer_TaskID=task_id; //³]¦¨¦P¤@­Óid´N¥i¥H¦b¦P¤@­Ó³B²z¨ç¦¡¤¤³B²z
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );

  // Setup the GAP Peripheral Role Profile
  {
    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = FALSE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = true;
    #endif

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
    
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

   // GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
   // GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Setup the Thermometer Characteristic Values
  {
    uint8 thermometerSite = THERMOMETER_TYPE_BODY;
    Thermometer_SetParameter( THERMOMETER_TYPE, sizeof ( uint8 ), &thermometerSite );
    
    thermometerIRange_t thermometerIRange= {4,60};
    Thermometer_SetParameter( THERMOMETER_IRANGE, sizeof ( uint16 ), &thermometerIRange );
  }
  HalHumiInit();
  VOID Humidity_RegisterAppCBs( &sensorTag_HumidCBs );
  
  
  
  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
 // DevInfo_AddService();                           // Device Information Service
 // SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
  Humidity_AddService (GATT_ALL_SERVICES );       // Humidity Service
  Batt_AddService( );     // Battery Service
  ProxReporter_AddService( GATT_ALL_SERVICES );
 // resetCharacteristicValues();
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif
  // Setup the SimpleProfile Characteristic Values
  {
    uint8 charValue1 = 1;
    uint8 charValue2 = 2;
    uint8 charValue3 = 3;
    uint8 charValue4 = 4;
    uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &charValue3 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &charValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
  }
//  SK_AddService( GATT_ALL_SERVICES ); // Simple Keys Profile
  // Register for Thermometer service callback
  Thermometer_Register ( thermometerCB );// ¦bThermometer_Write_CB¤¤·|³Q¥s¥Î
  Thermometer_AddService(GATT_ALL_SERVICES);

  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLEPeripheral_TaskID );
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFF; // All port 0 pins (P0.0-P0.7) as output
  P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1E; // All port 1 pins P2.0 as ¿é¤J«ö¶s
  
  P0 = 0;   // All pins on port 0 to low
  P1 = 0;   // All pins on port 1 to low
  P2 = 0x01;   // All pins on p2.0 to high  ¨ä¥Llow 
#if (defined HAL_UART) && (HAL_UART == TRUE)
 // initUART();
#endif
  
#if (defined HAL_LCD) && (HAL_LCD == TRUE)

#if defined FEATURE_OAD
  #if defined (HAL_IMAGE_A)
    HalLcdWriteStringValue( "BLE Peri-A", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #else
    HalLcdWriteStringValue( "BLE Peri-B", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #endif // HAL_IMAGE_A
#else
  HalLcdWriteString( "BLE Peripheral", HAL_LCD_LINE_1 );
#endif // FEATURE_OAD

#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

  // initialize the ADC for battery reads
  HalAdcInit();
  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_NONE, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )

  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{
  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
    {
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
    //VOID ProxReporter_RegisterAppCBs( &keyFob_ProximityCBs );
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );
    // Set timer for first battery read event
    // Start Bond Manager
    //ProxReporter_SetParameter(  LED2_LEVEL,  sizeof ( uint8 ), &LED2PWMLevel );
   // ProxReporter_SetParameter(  LED3_LEVEL,  sizeof ( uint8 ), &LED3PWMLevel );
   // ProxReporter_SetParameter( PP_IM_ALERT_LEVEL,  sizeof ( uint8 ), &keyfobProxIMAlertLevel );
   // ProxReporter_SetParameter( CHAR5, CHAR5_LEN, charValue5 );
    osal_start_timerEx( simpleBLEPeripheral_TaskID, TH_BATTERY_CHECK_EVT, BATTERY_CHECK_PERIOD );
    osal_set_event( simpleBLEPeripheral_TaskID, Record_readsenser_EVT );
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );
    // Set timer for first periodic event    
    return ( events ^ SBP_START_DEVICE_EVT );
  }
 if ( events & TH_START_DISCOVERY_EVT )
 {
    return ( events ^ TH_START_DISCOVERY_EVT );
 }
 
 
    if ( events & TH_PERIODIC_IMEAS_EVT ) //osal_start_timerEx( simpleBLEPeripheral_TaskID, TH_PERIODIC_IMEAS_EVT, 2000 );
  { 
      uint8 new_adv_enabled_status;  
      
      new_adv_enabled_status = TRUE;
      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status ); 
      
    return (events ^ TH_PERIODIC_IMEAS_EVT);
  }
  
 if ( events & TH_BATTERY_CHECK_EVT )
  {
    if( battery==TRUE )
    {
    // Restart timer
    if ( BATTERY_CHECK_PERIOD )
    {
      osal_start_timerEx( simpleBLEPeripheral_TaskID, TH_BATTERY_CHECK_EVT, BATTERY_CHECK_PERIOD );
    }
     //perform battery level check
     Batt_MeasLevel( );
    }
    return (events ^ TH_BATTERY_CHECK_EVT);
  } 
  
  if ( events & TH_DISCONNECT_EVT ) //osal_start_timerEx( thermometer_TaskID, TH_DISCONNECT_EVT, 5000 );
  {    
     uint8 advEnable = FALSE;
         //disable advertising on disconnect
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advEnable ); 
    osal_set_event( simpleBLEPeripheral_TaskID, TH_PERIODIC_IMEAS_EVT );
  return (events ^ TH_DISCONNECT_EVT);
  }
    
  //periodic indications - if enabled
  if ( events & TH_PERIODIC_MEAS_EVT ) //osal_start_timerEx( simpleBLEPeripheral_TaskID, TH_PERIODIC_MEAS_EVT, 1000 );
  { 
     if (humiEnabled)
     {
    // Perform periodic application task
    HalHumiExecMeasurementStep(humiState);
      if (humiState == 2)
      {
        readHumData();
        humiState = 0;
        osal_start_timerEx( simpleBLEPeripheral_TaskID, TH_PERIODIC_MEAS_EVT, 2000 );  //2¬íÄÁÅª¨ú¤@¦¸SI7020¸ê®Æ
      }
      else
      {
        humiState++;
        osal_start_timerEx( simpleBLEPeripheral_TaskID, TH_PERIODIC_MEAS_EVT, 20 );
      }
     }
       else
    {
       resetCharacteristicValues();
    }
    // performPeriodicTask();
     
    return (events ^ TH_PERIODIC_MEAS_EVT);
  } 
   
  if ( events & Record_readsenser_EVT)
  {
    HalHumiExecMeasurementStep(humiState);
      if (humiState == 2)
      {
        readHumData();
        SensorRecord();
        ReadRecord();
        humiState = 0;
        osal_start_timerEx( simpleBLEPeripheral_TaskID, Record_readsenser_EVT, 60000 );  //60¬íÄÁÅª¨ú¤@¦¸SI7020¸ê®Æ
      }
      else
      {
        humiState++;
        osal_start_timerEx( simpleBLEPeripheral_TaskID, Record_readsenser_EVT, 20 );
      }
     
      return (events ^ Record_readsenser_EVT);
   }

#if defined ( PLUS_BROADCASTER )
  if ( events & SBP_ADV_IN_CONNECTION_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );

    return (events ^ SBP_ADV_IN_CONNECTION_EVT);
  }
  
#endif // PLUS_BROADCASTER

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
    case GATT_MSG_EVENT:
      thermometerProcessGattMsg( (gattMsgEvent_t *) pMsg );
      break;

    default:
    // do nothing
    break;
  }
}

#if defined( QB_Keys )
/*********************************************************************
 * @fn      simpleBLEPeripheral_HandleKeys
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
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 SK_Keys = 0;

  VOID shift;  // Intentionally unreferenced parameter
  
  if ( keys & HAL_KEY_SW_1 ) //P2.0 IO  Ãö³¬¼s¼½¶}Ãö
  {
    SK_Keys |= SK_KEY_RIGHT;
    
    // if device is not in a connection, pressing the right key should toggle
    // advertising on and off
    key = true;
    uint8 advEnable = FALSE;
         //disable advertising on disconnect
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advEnable );
          // Terminate Connection
    GAPRole_TerminateConnection();
    if( gapProfileState != GAPROLE_CONNECTED )
    {
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

      if( current_adv_enabled_status == FALSE )
      {
        new_adv_enabled_status = TRUE;
      }
      else
      {
        new_adv_enabled_status = FALSE;
      }
      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
    }
 
  //thermometerCelcius++;
//  thermometerMeasIndicate();
  }
SK_SetParameter( SK_KEY_ATTR, sizeof ( uint8 ), &SK_Keys );
}
#endif // #if defined( QB_Keys )

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */ 
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  if ( gapProfileState != newState )
  {
    switch( newState )
    {
    case GAPROLE_STARTED:
      {
      }
      break;

    //if the state changed to connected, initially assume that keyfob is in range
    case GAPROLE_ADVERTISING:  //¼s¼½ª¬ºA«á5¬íÂà¤J1.5¬í¼s¼½
      {  
       battery=false;
       humiEnabled = false;
       if(key == true)
       {
         key = false;
       }
       else
       {
       uint16 advInt = longadv;
     
      // GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
     //  GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
       GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
       GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
    
       osal_start_timerEx( simpleBLEPeripheral_TaskID, TH_DISCONNECT_EVT, 4500 ); 
         
       }

      }
      break;
      
    //if the state changed to connected, initially assume that keyfob is in range      
    case GAPROLE_CONNECTED:  //³sµ²Â_½u«á5¬í«e500ms¼s¼½
      {
        battery=TRUE;
        humiEnabled = TRUE;
        uint16 advInt = shortadv;
     
      //  GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
      //  GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
        
      }
      break;

    case GAPROLE_WAITING:
      {
      resetSensorSetup();
     // osal_start_timerEx( simpleBLEPeripheral_TaskID, TH_PERIODIC_IMEAS_EVT, 2000 );
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
      //osal_start_timerEx( simpleBLEPeripheral_TaskID, TH_PERIODIC_IMEAS_EVT, 1000 );
      }
      break;
    }
  }
}
/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void simpleProfileChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newValue );

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 1:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &newValue );

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 3:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;

    default:
      // should not reach here!
      break;
  }
}

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)


/*********************************************************************
 * @fn      thermometerCB
 *
 * @brief   Callback function for thermometer service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void thermometerCB(uint8 event)
{  
  switch (event)
  {
  case THERMOMETER_TEMP_IND_ENABLED:
    osal_set_event( simpleBLEPeripheral_TaskID, TH_PERIODIC_MEAS_EVT );
  break;
        
  case  THERMOMETER_TEMP_IND_DISABLED:
    temperatureMeasCharConfig = false;
    osal_stop_timerEx( simpleBLEPeripheral_TaskID, TH_PERIODIC_MEAS_EVT );  
    thMeasTimerRunning = FALSE;
    break;

  case THERMOMETER_IMEAS_NOTI_ENABLED:
    temperatureIMeasCharConfig = true;
    if (gapProfileState == GAPROLE_CONNECTED)
    {
    //  osal_start_timerEx( simpleBLEPeripheral_TaskID, TH_PERIODIC_IMEAS_EVT, 2000 );
    }      
    break;

  case  THERMOMETER_IMEAS_NOTI_DISABLED:
    temperatureIMeasCharConfig = false;
    osal_stop_timerEx( simpleBLEPeripheral_TaskID, TH_PERIODIC_IMEAS_EVT );  
    break;
  
  case THERMOMETER_INTERVAL_IND_ENABLED:
    if (gapProfileState == GAPROLE_CONNECTED)
    {
      temperatureIntervalConfig = true;
    }      
    break;

  case  THERMOMETER_INTERVAL_IND_DISABLED:
    if (gapProfileState == GAPROLE_CONNECTED)
    {
      temperatureIntervalConfig = false;
    } 
    break;   
   
  default:  
    break;
  }
}
/*********************************************************************
 * @fn      PasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void PasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{ uint32  passcode=123456;
  uint8   str[7];

  //¦b?¨½¥i¥H?¸m¦s?¡A«O¦s¤§«e?©wªº±K?¡A??´N¥i¥H??­×§ï°t?±K?¤F¡C
  // Create random passcode
  LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ));
  passcode %= 1000000;

  //¦blcd¤W?¥Ü?«eªº±K?¡A??¤âÉóºÝ¡A®ÚÕu¦¹±K??±µ¡C
  // Display passcode to user
  if ( uiOutputs != 0 )
  {
    HalLcdWriteString( "Passcode:",  HAL_LCD_LINE_1 );
    HalLcdWriteString( (char *) _ltoa(passcode, str, 10),  HAL_LCD_LINE_2 );
  }
  
  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
}

/*********************************************************************
* @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void PairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_STARTED )/*¥DÉó?°_?±µ¡A??¤J?©l?©w??*/
  {
    HalLcdWriteString( "Pairing started", HAL_LCD_LINE_1 );
   // gPairStatus = 0;
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )/*?¥DÉó´£¥æ±K?¦Z¡A??¤J§¹¦¨*/
  {
    if((status == 0x04)||(status == 0x01))
    {
     GAPRole_TerminateConnection();
    }
    if ( status == SUCCESS )
    {
      HalLcdWriteString( "Pairing success", HAL_LCD_LINE_1 );/*±K?¥¿ÚÌ*/
      gPairStatus = 1;
      testx=1;
    }
    else
    {
      HalLcdWriteStringValue( "Pairing fail", status, 10, HAL_LCD_LINE_1 );/*±K?¤£¥¿ÚÌ¡A©ÎªÌ¥ý«e¤w??©w*/
       if(testx == 1 )
      {
        gPairStatus = 1;
      }
      else
      {
        gPairStatus = 0;
      }
    }
    if(gPairStatus !=1)
    {
      GAPRole_TerminateConnection();
    }
  }
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
  {
    if ( status == SUCCESS )
    {
      testx=1;
      HalLcdWriteString( "Bonding success", HAL_LCD_LINE_1 );
    }
  }
}
/*********************************************************************
 * @fn      humidityChangeCB
 *
 * @brief   Callback from Humidity Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void humidityChangeCB( uint8 paramID )
{
  if ( paramID == HUMIDITY_CONF)
  {
    uint8 newValue;

    Humidity_GetParameter( HUMIDITY_CONF, &newValue );

    osal_set_event( simpleBLEPeripheral_TaskID, TH_PERIODIC_MEAS_EVT );
  }
}
/*********************************************************************
 * @fn      readHumData
 *
 * @brief   Read humidity data
 *
 * @param   none
 *
 * @return  none
 */
static void readHumData(void)
{
  uint8 hData[HUMIDITY_DATA_LEN];
  //uint8 humData[1];
  float humData[1];
  humidityRead(hData);
  //humData[0]=(int)(humiCelcius/10);
  humData[0] = 26.1;
  Humidity_SetParameter( HUMIDITY_DATA, HUMIDITY_DATA_LEN, &humData);
  //ProxReporter_SetParameter(LED2_LEVEL, sizeof ( uint8 ), &humData);
  HalIRTempRead2(hData);
  thermometerSendStoredMeas();
}
/*********************************************************************
 * @fn      resetCharacteristicValue
 *
 * @brief   Initialize a characteristic value to zero
 *
 * @param   servID - service ID (UUID)
 *
 * @param   paramID - parameter ID of the value is to be cleared
 *
 * @param   vakue - value to initialise with
 *
 * @param   paramLen - length of the parameter
 *
 * @return  none
 */
static void resetCharacteristicValue(uint16 servUuid, uint8 paramID, uint8 value, uint8 paramLen)
{
  uint8* pData = osal_mem_alloc(paramLen);

  if (pData == NULL)
  {
    return;
  }

  osal_memset(pData,value,paramLen);

  switch(servUuid)
  {
    case HUMIDITY_SERV_UUID:
      //Humidity_SetParameter( paramID, paramLen, pData);
      break;

    default:
      // Should not get here
      break;
  }

  osal_mem_free(pData);
}

/*********************************************************************
 * @fn      resetCharacteristicValues
 *
 * @brief   Initialize all the characteristic values related to the sensors to zero
 *
 * @return  none
 */
static void resetCharacteristicValues( void )
{
  resetCharacteristicValue( HUMIDITY_SERV_UUID, HUMIDITY_DATA, 0, HUMIDITY_DATA_LEN);
  resetCharacteristicValue( HUMIDITY_SERV_UUID, HUMIDITY_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
}


/*********************************************************************
*********************************************************************/
/*********************************************************************
 * @fn      resetSensorSetup
 *
 * @brief   Turn off all sensors that are on
 *
 * @param   none
 *
 * @return  none
 */
static void resetSensorSetup (void)
{
  if (humiEnabled)
  {
    HalHumiInit();
    humiEnabled = FALSE;
  }
  // Reset all characteristics values
}
/*********************************************************************
 * @fn      sensorTag_test
 *
 * @brief   Run a self-test of the sensor TAG
 *
 * @param   none
 *
 * @return  bitmask of error flags
 */
uint16 sensorTag_test(void)
{
  selfTestResult = HalSensorTest();

  // Write the self-test result to the test service

  return selfTestResult;
}
//*********************************************************************
/*static void iBeaconCB( uint8 attrParamID )
{
  switch( attrParamID )
  {
  case CHAR5:
    ProxReporter_GetParameter( CHAR5, &uuid );
     VOID osal_snv_write( 0x82, 16, uuid );
    break;
  case LED2_LEVEL:
    ProxReporter_GetParameter( LED2_LEVEL, &LED2PWMLevel );
    x[1]=LED2PWMLevel;
   VOID osal_snv_write( 0x81, 2, x );
    break;
    
  case LED3_LEVEL:
      ProxReporter_GetParameter( LED3_LEVEL, &LED3PWMLevel );
     if( LED3PWMLevel != PP_ALERT_LEVEL_NO )
      {
       //  HalLedSet( HAL_LED_3, HAL_LED_MODE_ON );
      // PWM3Start(LED3PWMLevel);
      }
      else // proximity monitor turned off alert because the path loss is below threshold
      {
      //   HalLedSet( HAL_LED_3, HAL_LED_MODE_OFF );
        // PWM3Stop();
      }
    break;
 
  case PP_IM_ALERT_LEVEL:
    {
    ProxReporter_GetParameter( PP_IM_ALERT_LEVEL, &keyfobProxIMAlertLevel );
  //  advertData[26]=keyfobProxIMAlertLevel;
    //GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
    x[0]=keyfobProxIMAlertLevel;
    VOID osal_snv_write( 0x80, 1, x );
    }
    break;

  default:
    // should not reach here!
    break;
  }
}*/
/*********************************************************************
 * @fn      SaveSensorData
 *
 * @brief   For the record of Sensor.
 *
 * @param   none
 *
 * @return  none
 */

      //////////////////////////////////////////////////////
      // Per minute temperature&humidity average record.  //
      // An hour only record average data once.           //
      // MAX&MIN data is continue update every minute but //
      // MAX&MIN data will be remove every 24hour once.   //
      //////////////////////////////////////////////////////

static void SensorRecord (void)
{
  if ( Day <= 3 ) // 3 Days
  {
    if ( Hour <= 23 ) // 24 Hours
    {
      float Minute_temperature = 0;
      uint16 Minute_Humidity = 0;
      if ( Minute < 60 )
      {
        Minute_temperature = ((int32)thermometerCelcius)/(float)10;
        Minute_Humidity = (uint32)humiCelcius/10;
        Total_Temperature = Total_Temperature + Minute_temperature; //Calculate total Temperature and Humidity value
        Total_Humidity = Total_Humidity + Minute_Humidity;
          
          if ( Minute_temperature > MAX_Temperature )         //Store MAX&MIN
          {
            MAX_Temperature = Minute_temperature;
          }
          if ( Minute_temperature < MIN_Temperature )
          {
            MIN_Temperature = Minute_temperature;
          }
          if ( Minute_Humidity > MAX_Humidity )
          {
            MAX_Humidity = Minute_Humidity;
          }
          if ( Minute_Humidity < MIN_Humidity )
          {
            MIN_Humidity = Minute_Humidity;
          }        
        Minute = Minute + 1;            
      }
      else if( Minute == 60 )
      {
        Average_Temperature = Total_Temperature / (float)60;  //Calculate Average Temperature and Humidity value
        Average_Humidity = Total_Humidity / 60;
        SRT[Day][Hour] = Average_Temperature;                 //Store record
        SRH[Day][Hour] = Average_Humidity;
        Total_Humidity = 0;                                   //Initialize total
        Total_Temperature = 0;
        Hour = Hour + 1;
        Minute = 0;
      }
                           
      if( Hour == 24 )
      {
        MAX_Temperature = 0;    //Initialize MAX&MIN
        MIN_Temperature = 100;
        MAX_Humidity = 0;
        MIN_Humidity = 100;
        Day = Day + 1;
        Hour = 0;
      }      
    
    }
    if( Day == 3)
    {
      Day = 0;
    }
     
  }
  
}

/*********************************************************************
 * @fn      HostReadRecord
 *
 * @brief   Read humidity data
 *
 * @param   none
 *
 * @return  none
 */
static void ReadRecord(void)
{
  
  uint8 Read_Avg_MaxMin_Humidity[3];
  uint8 DAH[2];
  DAH[0] = (int)Hour;
  DAH[1] = (int)Day;
  Read_Avg_MaxMin_Humidity[0] = (int)MAX_Humidity;
  Read_Avg_MaxMin_Humidity[1] = (int)MIN_Humidity;
  Read_Avg_MaxMin_Humidity[2] = (int)Average_Humidity;
  
  uint8 Read_Avg_MaxMin_Temperature[3];
  Read_Avg_MaxMin_Temperature[0] = (int) MAX_Temperature;
  Read_Avg_MaxMin_Temperature[1] = (int) MIN_Temperature;
  Read_Avg_MaxMin_Temperature[2] = (int) Average_Temperature;
  
  ProxReporter_SetParameter( HUMIDITY_MAX_LEVEL, sizeof ( uint8 ), &Read_Avg_MaxMin_Humidity[0] );
  ProxReporter_SetParameter( HUMIDITY_MIN_LEVEL, sizeof ( uint8 ), &Read_Avg_MaxMin_Humidity[1] );
  ProxReporter_SetParameter( HUMIDITY_AVG_LEVEL, sizeof ( uint8 ), &Read_Avg_MaxMin_Humidity[2] );
  
  ProxReporter_SetParameter( HOUR_LEVEL, sizeof ( uint8 ), &DAH[0] );
  ProxReporter_SetParameter( DAY_LEVEL, sizeof ( uint8 ), &DAH[1] );
  
  ProxReporter_SetParameter( TEMPERATURE_MAX_LEVEL, sizeof ( uint8 ), &Read_Avg_MaxMin_Temperature[0] );
  ProxReporter_SetParameter( TEMPERATURE_MIN_LEVEL, sizeof ( uint8 ), &Read_Avg_MaxMin_Temperature[1] );
  ProxReporter_SetParameter( TEMPERATURE_AVG_LEVEL, sizeof ( uint8 ), &Read_Avg_MaxMin_Temperature[2] );
  
}