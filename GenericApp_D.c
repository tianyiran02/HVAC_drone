/******************************************************************************
  Filename:       GenericApp.c
  Revised:        $Date: 2014-09-07 13:36:30 -0700 (Sun, 07 Sep 2014) $
  Revision:       $Revision: 40046 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2014 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
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
******************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

   
#include "GenericApp_D.h"
#include "DebugTrace.h"

// Transplant from version 4 2014/10/29. ver5
// 
// duplicate decleartion has been removed
#include <string.h>
#include <stdlib.h>


/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_adc.h"
// end

#include "Config_APP.h"

#if !defined( WIN32 ) || defined( ZBIT )
  #include "OnBoard.h"
#endif

/* RTOS */
#if defined( IAR_ARMCM3_LM )
#include "RTOS_App.h"
#endif

/*********************************************************************
 * MACROS
 */
   
/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * TYPEDEFS
 */
typedef struct
{
  uint16               *pIn;
  uint16               *pOut;
} ADC_Ringbuffer_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
// This list should be filled with Application specific Cluster IDs.
const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS] =
{
  GENERICAPP_CLUSTERID
};

const SimpleDescriptionFormat_t GenericApp_SimpleDesc =
{
  GENERICAPP_ENDPOINT,              //  int Endpoint;
  GENERICAPP_PROFID,                //  uint16 AppProfId[2];
  GENERICAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  GENERICAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  GENERICAPP_FLAGS,                 //  int   AppFlags:4;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList,  //  byte *pAppInClusterList;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in GenericApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t GenericApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte GenericApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // GenericApp_Init() is called.

devStates_t GenericApp_NwkState;

byte GenericApp_TransID;  // This is the unique message ID (counter)

// This destination was used to connect with coordinator
afAddrType_t Point_To_Point_DstAddr;

/* system related */
// Drone upload interval minutes counter. 
// When value greater than  UpLoad_NormalPeriod , perform a upload
static uint8 drone_MinCounter = 0;

// oneSecond Timer. 
// After system powerup, avoid upload in the first minute. This is because
// the first minute the ring buffer data is not ready.
static uint8 oneSecondTimercounter = 0;

/* ADC related */
static uint16 ADCReasult[4] = {0}; //0 - Temp; 1 - Current; 2 - HPress; 3 - LPress

// Used to avoid upload data in the first minute
static short ADCReadytoSend = 0; 

// variables for windows buffer 
ADC_Ringbuffer_t DATA1_RingBuffer_pointer; // temp, ADC0
ADC_Ringbuffer_t DATA2_RingBuffer_pointer; // current, ADC1
ADC_Ringbuffer_t DATA3_RingBuffer_pointer; // HP, ADC2
ADC_Ringbuffer_t DATA4_RingBuffer_pointer; // LP, ADC3

// Windows size: 60 bytes
static uint16 DATA1_RingBuffer[65] = {0}; //temperature
static uint16 DATA2_RingBuffer[65] = {0}; // current
static uint16 DATA3_RingBuffer[65] = {0}; // HP
static uint16 DATA4_RingBuffer[65] = {0}; // LP
// end

/* upload control related */
// used to count delay time.
static uint8 requestTimer = 0; 
static uint8 firtHighFreqAttemp = (DRONE_TESTUPLOADPOINT + 1);
static uint8 highFreqSecCounter = DRONE_TESTUPLOADTIME;

// flags
static uint8 drone_resetFlag = FALSE;
static uint8 drone_waitACKTimer = DRONE_ACKWAITTIME;
static uint8 drone_waitACKFlag = FALSE;
static uint8 lastUploadFail = FALSE; 
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );

static void drone_PeriodicDataMessage( void );
static void drone_DataSendingRequest( void );
static void drone_InitialMessage( void );
static uint16 pushBufSample(uint8, uint16);
static uint16 getBufSample(uint8);
static void drone_PrepareData( void );
static void drone_GetADCData( void );
static uint8 drone_Datachange( void );
static void drone_DeviceCMDReact(afIncomingMSGPacket_t *Msg);


#if defined( IAR_ARMCM3_LM )
static void GenericApp_ProcessRtosMessage( void );
#endif

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
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
void GenericApp_Init( uint8 task_id )
{
  GenericApp_TaskID = task_id;
  GenericApp_NwkState = DEV_INIT;
  GenericApp_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  HalAdcInit();
  
  // If the hardware is application specific - add it here.
  HalLedSet (HAL_LED_3, HAL_LED_MODE_OFF); // LED3, D1 positive logic
  
  // If the hardware is other parts of the device add it in main().
  
  Point_To_Point_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  Point_To_Point_DstAddr.endPoint = GENERICAPP_ENDPOINT;
  Point_To_Point_DstAddr.addr.shortAddr = 0x0000; // Send to Coodinator 
  
  // Fill out the endpoint description.
  GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_epDesc.task_id = &GenericApp_TaskID;
  GenericApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
  GenericApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &GenericApp_epDesc );

  // Register for all key events - This app will handle all key events

  // Initialize ring buffer
  DATA1_RingBuffer_pointer.pIn = DATA1_RingBuffer;
  DATA1_RingBuffer_pointer.pOut = DATA1_RingBuffer;

  DATA2_RingBuffer_pointer.pIn = DATA2_RingBuffer;
  DATA2_RingBuffer_pointer.pOut = DATA2_RingBuffer;

  DATA3_RingBuffer_pointer.pIn = DATA3_RingBuffer;
  DATA3_RingBuffer_pointer.pOut = DATA3_RingBuffer;

  DATA4_RingBuffer_pointer.pIn = DATA4_RingBuffer;
  DATA4_RingBuffer_pointer.pOut = DATA4_RingBuffer;
  
  osal_start_timerEx( GenericApp_TaskID,
                  GENERICAPP_WDT_CLEAR_EVT,
                  GENERICAPP_WDT_TIMEOUT);
  
  // Update the display
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "GenericApp", HAL_LCD_LINE_1 );
#endif

#if defined( IAR_ARMCM3_LM )
  // Register this task with RTOS task initiator
  RTOS_RegisterApp( task_id, GENERICAPP_RTOS_MSG_EVT );
#endif
}

/*********************************************************************
 * @fn      GenericApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 GenericApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;
  
  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        // Msg
        case AF_INCOMING_MSG_CMD:
          GenericApp_MessageMSGCB( MSGpkt );
          break;

        // network condition changeed - normally the first time enter network  
        case ZDO_STATE_CHANGE:
          GenericApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (GenericApp_NwkState == DEV_ZB_COORD) ||
               (GenericApp_NwkState == DEV_ROUTER) ||
               (GenericApp_NwkState == DEV_END_DEVICE) )
          {
            osal_start_timerEx( GenericApp_TaskID,
                              GENERICAPP_DRONE_PERIODIC_MSG_EVT,
                              GENERICAPP_DRONE_PERIODIC_MSG_TIMEOUT + 5000 );
            osal_start_timerEx( GenericApp_TaskID,
                              GENERICAPP_ADCMEASURE_EVT,
                              GENERICAPP_ADC_TIMEOUT );
            // send the inital Msg to the coordinator          
            drone_InitialMessage();
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  
  /** 
   *    Periodic data timer, check whether need to upload once timeout 
   *
   * @time GENERICAPP_DRONE_PERIODIC_MSG_TIMEOUT
   *
   */
  if ( events & GENERICAPP_DRONE_PERIODIC_MSG_EVT )
  {  
    /* Send Req
     *
     * 1. If data have significant change, send every 1 mintue;
     * 2. Else send every UpLoad_NormalPeriod mintue.
     *
     */
    // Prepare the sending data (have to calculate twice before sending)
    // 1st, datachange? 2st send (request-ask may introduce huge delay)
    // here is 1st
    if(drone_Datachange() || lastUploadFail) // Data changed significantly
    {
      drone_DataSendingRequest(); // Send data sending request
      drone_MinCounter = 0;
    }
    else // Data has no big changed
    {
      // counter adding. counter will only clear after upload
      drone_MinCounter ++;
      if(drone_MinCounter >= UpLoad_NormalPeriod)
        drone_DataSendingRequest(); // Send data sending request
    }
    
    // Setup to send message again in normal period (+ a little jitter (0-3s)*ID time)
    osal_start_timerEx( GenericApp_TaskID, GENERICAPP_DRONE_PERIODIC_MSG_EVT,
        (GENERICAPP_DRONE_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x0AFF) * DRONE_DEVICE_ID ));

    // return unprocessed events
    return (events ^ GENERICAPP_DRONE_PERIODIC_MSG_EVT);
  }
  
  /* 1s timer 
   *
   * 1. read ADC data
   * 2. update ACK timer 
   * 3. handle the request delay
   * 4. upload every 30s when power up
   *
   **/
  if( events & GENERICAPP_ADCMEASURE_EVT)
  {
    /* Read data from ADC */
    drone_GetADCData();
    
    /* updata ACK timer */
    if(drone_waitACKFlag)
    {
      drone_waitACKTimer --;
      
      // if timeout, reset. DRONE_ACKWAITTIME default timeout
      if(drone_waitACKTimer == 0) 
        drone_resetFlag = TRUE;
    }
    
    /* handle the request delay */
    // Check request timer
    // for detail, see in function drone_DeviceCMDReact
    if(requestTimer > 0)
    {
      requestTimer --;
      drone_MinCounter = 0;
      
      // delay finish, resend request
      if(requestTimer == 0)
      {
        drone_DataSendingRequest();
      }
    }
    
    /* upload every setting time after power up
     *
     * This is achieved by few steps. First the 1s timer will count 30s.
     * When 30s, will reload 1min timer to triger in next 1s, and when 
     * in power on condition, the dataChange() function will run another routine
     * which is different from normal. It will upload one ADC data from the 
     * ring buffer directly. By this can achieve the upload in every 30s for 5
     * times after power up without change a large range of existing code.
     *
     */
    
    // if just power up
    if(firtHighFreqAttemp)
    {
      highFreqSecCounter --;
      
      // every 30s
      if(highFreqSecCounter <= 1)
      {
        highFreqSecCounter = 30; // reload
        firtHighFreqAttemp --;
        
        // Setup to send message in 1s
        osal_start_timerEx( GenericApp_TaskID, GENERICAPP_DRONE_PERIODIC_MSG_EVT,
            GENERICAPP_ADC_TIMEOUT);
      }
    }
    
    
    // Reload the timer
    osal_start_timerEx( GenericApp_TaskID, GENERICAPP_ADCMEASURE_EVT,
                        GENERICAPP_ADC_TIMEOUT );
    
    return( events ^ GENERICAPP_ADCMEASURE_EVT );
  }
  
  /*    WDT timer
   *
   */
  if( events & GENERICAPP_WDT_CLEAR_EVT)
  {
    // if reset flag not set, clear WDT every few time
    if(drone_resetFlag == FALSE)
    {
      // clear timer     
      WDCTL |= WDCLP1; 
      WDCTL |= WDCLP2; 
    }
  
    osal_start_timerEx( GenericApp_TaskID,
                  GENERICAPP_WDT_CLEAR_EVT,
                  GENERICAPP_WDT_TIMEOUT);
    
    return( events ^ GENERICAPP_WDT_CLEAR_EVT );
  }
  

#if defined( IAR_ARMCM3_LM )
  // Receive a message from the RTOS queue
  if ( events & GENERICAPP_RTOS_MSG_EVT )
  {
    // Process message from RTOS queue
    GenericApp_ProcessRtosMessage();

    // return unprocessed events
    return (events ^ GENERICAPP_RTOS_MSG_EVT);
  }
#endif

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */


/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
  case GENERICAPP_CLUSTERID:
    break;

  // CMD Msg
  case QUEEN_CMD_CLUSTERID:
    drone_DeviceCMDReact(pkt);
    break;
      
  default:
    break;
  }
}


/******************************************************************************
                             Functional 
******************************************************************************/

/*********************************************************************
 * @fn      drone_DataSendingRequest
 *
 * @brief   send the DataSendingRequest to Queen, set relative flag
 *
 * @param   none
 *
 * @return  none
 */
static void drone_DataSendingRequest( void )
{
  uint8 msgBuf[3] = {0x23,0x00,0x00};
  msgBuf[1] = DRONE_SEND_REQ;
  
  // send request to coordinator
  if ( AF_DataRequest( &Point_To_Point_DstAddr, &GenericApp_epDesc,
               QUEEN_CMD_CLUSTERID,
               3,
               msgBuf,
               &GenericApp_TransID,
               AF_DISCV_ROUTE,
               AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {

  }
  else
  {
  // Error occurred in request to send.
  }
  
  // set waitACKflag true. Clear waitflag when get ACK
  drone_waitACKFlag = TRUE;
}



/*********************************************************************
 * @fn      drone_PeriodicDataMessage
 *
 * @brief   send the periodic sensor data to coodinator
 *
 * @param   none
 *
 * @return  none
 */
static void drone_PeriodicDataMessage( void )
{
  uint8 i = 0;
  // Prepare data to send -- Yiran
  uint8 temp[15] = {0x23,0x40,0x07,0x80,0x02,0};
  
  // unfinish function, the switch status
  i = 0;
  
  /* Determine whether update data */
  if(ADCReadytoSend || firtHighFreqAttemp)
  {
    temp[2] = 10;
    temp[3] = (0x80 | 0x40 | 0x02);
    // 0x80 contain on-off data, 0x40 contain data, 0x02 data type 1
    
    // Insert on-off data, unfinish feature
    switch(i)
    {
    case 0:
      temp[3] |= 0x01;
      break;
    case 1:
      temp[3] &= ~0x01;
      break;
    default:
      break;
    }
    
    temp[4] = 0x04; // 4 Bytes
    temp[5] = (uint8)(ADCReasult[0] >> 5);
    temp[6] = (uint8)(ADCReasult[1] >> 5);
    temp[7] = (uint8)(ADCReasult[2] >> 5);
    temp[8] = (uint8)(ADCReasult[3] >> 5);    
    
    for(i = 0;i <=8;i ++) // CRC
      temp[9] += temp[i];

    
    // Start to send
    if ( AF_DataRequest( &Point_To_Point_DstAddr, &GenericApp_epDesc,
                   GENERICAPP_CLUSTERID,
                   temp[2],
                   temp,
                   &GenericApp_TransID,
                   AF_DISCV_ROUTE,
                   AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
      // Flash LED, later move to ACK
      HalLedBlink( HAL_LED_3, 40, 50, 50 );
      
      // clear sending buf
      drone_MinCounter = 0;
      
      // assume fail untial success upload ACK
      lastUploadFail = TRUE;
    }
    else
    {
    // Error occurred in request to send.
    }
    
    // set flag and wait for ACK
    drone_waitACKFlag = TRUE;
  }
}

/*********************************************************************
 * @fn      drone_Datachange
 *
 * @brief   Compare the current data with the previous data,
 *
 * @param   
 *
 * @return  0 - not significant change; 1 - significant change
 */
static uint8 drone_Datachange( void )
{
  uint32 accTemp = 0;
  uint16 temp = 0;
  uint8 Status = 0;
  
  // Normal conditions
  // Divide by 60, median filter
  // Compare with previous data
  if(!firtHighFreqAttemp)
  {
    for(temp=0;temp<60;temp++)
      accTemp += getBufSample(1);
    temp = accTemp / 60;
    if(abs(temp - ADCReasult[0]) > TempTH)
      Status = 1;
    ADCReasult[0] = temp;
    accTemp = 0;
    
    for(temp=0;temp<60;temp++)
      accTemp += getBufSample(2);
    temp = accTemp / 60;
    if(abs(temp - ADCReasult[1]) > CurrentTH)
      Status = 1;
    ADCReasult[1] = temp;
    accTemp = 0;
    
    for(temp=0;temp<60;temp++)
      accTemp += getBufSample(3);
    temp = accTemp / 60;
    if(abs(temp - ADCReasult[2]) > HPressTH)
      Status = 1;
    ADCReasult[2] = temp;
    accTemp = 0;
    
    for(temp=0;temp<60;temp++)
      accTemp += getBufSample(4);
    temp = accTemp / 60;
    if(abs(temp - ADCReasult[3]) > HPressTH)
      Status = 1;
    ADCReasult[3] = temp;  
    accTemp = 0;
  }
  
  // If high frequency upload after power up
  else
  {
    ADCReasult[0] = getBufSample(1);
    ADCReasult[1] = getBufSample(2);
    ADCReasult[2] = getBufSample(3);
    ADCReasult[3] = getBufSample(4);
    
    Status = 1;
  }
      
  return Status;
}


/*********************************************************************
 * @fn      Sender_InitialMessage
 *
 * @brief   Send the initialize message to coodinator, device type
 *
 * @param   none
 *
 * @return  none
 */
void drone_InitialMessage( void )
{
  uint8 device[9] = {0};
  uint8 Msgbuf[11] = {0};
  
  // prepare device buffer
  device[0] = DRONE_DEVICE_ID;
  osal_memcpy(&device[1],NLME_GetExtAddr(),8);
  
  // prepare Msgbuf
  Msgbuf[0] = 0x23;
  Msgbuf[1] = DRONE_RESEND_INIT;
  osal_memcpy(&Msgbuf[2],device,9);
  
  // Send the init Msg
  if ( AF_DataRequest( &Point_To_Point_DstAddr, &GenericApp_epDesc,
                       QUEEN_CMD_CLUSTERID,
                       11,
                       Msgbuf,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // LED blink for 1s
    HalLedBlink( HAL_LED_3, 4, 50, 500 );
    
    // reset flag, reload timer
    drone_waitACKFlag = FALSE;
    drone_waitACKTimer = DRONE_ACKWAITTIME;
    // clear sending buf
    drone_MinCounter = 0;
  }
  else
  {
    // init msg sending failure, reset device
    drone_resetFlag = TRUE;
  }
  // wait for ACK
  drone_waitACKFlag = TRUE;
}

/*********************************************************************
 * @fn      drone_DeviceCMDReact
 *
 * @brief   React according to the CMD
 *
 * @param   none
 *
 * @return  none
 */
static void drone_DeviceCMDReact(afIncomingMSGPacket_t *Msg)
{
  
  /* if not CMD Msg, LED control msg */
  if((Msg->cmd.Data[0]) != 0x23)
  {
    switch(Msg->cmd.Data[0])
    {
    case 1:
      //LED1_ON;
      break;
      
    case 0:
      //LED1_OFF;
      break;
      
    default:
      break;
    }
  }
  
  /* CMD react */
  else
  {
    switch(Msg->cmd.Data[1])
    {
    case DRONE_SUCCESS_ACK:
      lastUploadFail = FALSE;
      break;
      
    case DRONE_FAIL_ACK:
      lastUploadFail = TRUE;
      break;  
      
    case DRONE_RESEND_INIT: //Ask for re-register
      drone_InitialMessage();
      break;
    
    case DRONE_DATA_ACK: // ACK
      // reset flag, reload timer
      drone_waitACKFlag = FALSE;
      drone_waitACKTimer = DRONE_ACKWAITTIME;
      break;
      
    case DRONE_SEND_REQ:
      /* clear flag first*/
      // reset flag, reload timer
      drone_waitACKFlag = FALSE;
      drone_waitACKTimer = DRONE_ACKWAITTIME;
      
      if(Msg->cmd.Data[2] == AVAILABLE)
      {
        // 2nd time calculate, update data to upload
        // 1st time calculate by drone_Datachange function
        drone_PrepareData();
        // Queen ready to receive data, send data
        drone_PeriodicDataMessage();
      }
      else if(Msg->cmd.Data[2] == UNAVAILABLE)
      {
        // Queen is not ready, delay 15s+(0-5)s resend request
        // use ADC timer to calculate time, set counter value.
        // get a random value from 15-20s.
        requestTimer = 15 + (osal_rand() & 0x0005);
      }
      break;
        
    default:
      break;
    }
  }
}

/******************************************************************************
                                RingBuffer
******************************************************************************/

/*********************************************************************
 * @fn      getBufSample
 *
 * @brief   Read data out from the ring buffer
 *
 * @param   
 *
 * @return  uint16
 */
static uint16 getBufSample(uint8 target)
{
  uint16 value = 0;
  
  switch(target)
  {
  case 1:
    value = *DATA1_RingBuffer_pointer.pOut;
    
    if(DATA1_RingBuffer_pointer.pOut == &DATA1_RingBuffer[64])
      DATA1_RingBuffer_pointer.pOut = DATA1_RingBuffer;
    else
      DATA1_RingBuffer_pointer.pOut ++;
    break;
    
  case 2: 
    value = *DATA2_RingBuffer_pointer.pOut;
    
    if(DATA2_RingBuffer_pointer.pOut == &DATA2_RingBuffer[64])
      DATA2_RingBuffer_pointer.pOut = DATA2_RingBuffer;
    else
      DATA2_RingBuffer_pointer.pOut ++;
    break;

  case 3:
    value = *DATA3_RingBuffer_pointer.pOut;
    
    if(DATA3_RingBuffer_pointer.pOut == &DATA3_RingBuffer[64])
      DATA3_RingBuffer_pointer.pOut = DATA3_RingBuffer;
    else
      DATA3_RingBuffer_pointer.pOut ++;
    break; 
   
  case 4:
    value = *DATA4_RingBuffer_pointer.pOut;
    
    if(DATA4_RingBuffer_pointer.pOut == &DATA4_RingBuffer[64])
      DATA4_RingBuffer_pointer.pOut = DATA4_RingBuffer;
    else
      DATA4_RingBuffer_pointer.pOut ++;
    break;
    
  default:
    break;
  }
  
  return value;
}


/*********************************************************************
 * @fn      pushBufSample
 *
 * @brief   Push one byte data to the ring buffer
 *
 * @param   
 *
 * @return  uint16
 */
static uint16 pushBufSample(uint8 target, uint16 value)
{  
  switch(target)
  {
  case 1:
    *DATA1_RingBuffer_pointer.pIn = value;
    
    if(DATA1_RingBuffer_pointer.pIn == &DATA1_RingBuffer[64])
      DATA1_RingBuffer_pointer.pIn = DATA1_RingBuffer;
    else
      DATA1_RingBuffer_pointer.pIn ++;
    break;
    
  case 2: 
    *DATA2_RingBuffer_pointer.pIn = value;
    
    if(DATA2_RingBuffer_pointer.pIn == &DATA2_RingBuffer[64])
      DATA2_RingBuffer_pointer.pIn = DATA2_RingBuffer;
    else
      DATA2_RingBuffer_pointer.pIn ++;
    break;

  case 3:
    *DATA3_RingBuffer_pointer.pIn = value;
    
    if(DATA3_RingBuffer_pointer.pIn == &DATA3_RingBuffer[64])
      DATA3_RingBuffer_pointer.pIn = DATA3_RingBuffer;
    else
      DATA3_RingBuffer_pointer.pIn ++;
    break;
   
  case 4:
    *DATA4_RingBuffer_pointer.pIn = value;
    
    if(DATA4_RingBuffer_pointer.pIn == &DATA4_RingBuffer[64])
      DATA4_RingBuffer_pointer.pIn = DATA4_RingBuffer;
    else
      DATA4_RingBuffer_pointer.pIn ++;
    break;
    
  default:
    break;
  }
 
  return value;
}



/*********************************************************************
 * @fn      drone_GetADCData
 *
 * @brief   Read data from ADC
 *
 * @param   
 *
 * @return  
 */
static void drone_GetADCData( void )
{
  uint16 temp = 0;
  
  // Temperature
  temp = HalAdcRead (0, HAL_ADC_RESOLUTION_14) - minTempOffset;
  pushBufSample(1,temp);
  
  // Current
  temp = HalAdcRead (1, HAL_ADC_RESOLUTION_14) + CurrentOffset;
  pushBufSample(2,temp);  

  // High pressure
  temp = HalAdcRead (2, HAL_ADC_RESOLUTION_14) + HPressOffset;
  pushBufSample(3,temp);

  // Low pressure
  temp = HalAdcRead (3, HAL_ADC_RESOLUTION_14) + LPressOffset;
  pushBufSample(4,temp);
  
  
  /* Power Saving to reduce send frequence 
   * Change ADCReadytoSend status,
   * 1 stand for data need update
   * 0 stand for no need to update
   */
  if(!ADCReadytoSend)
    oneSecondTimercounter++;
  
  if(oneSecondTimercounter > 60)
    ADCReadytoSend = 1;   
}


/*********************************************************************
 * @fn      drone_PrepareData
 *
 * @brief   Prepare the data every period time, wait to send
 *
 * @param   
 *
 * @return  
 */
static void drone_PrepareData( void )
{
  uint32 accTemp = 0;
  uint16 temp = 0;
  uint8 i = 0;
   
  // Recalculate the ADC reading
  if(!firtHighFreqAttemp)
  {
    for(i=0; i<=3; i++)
    {
      for(temp=0;temp<60;temp++)
        accTemp += getBufSample(i+1);
      
      temp = accTemp / 60;   
      ADCReasult[i] = temp;
      accTemp = 0;
    }
  }
  else
  {
    ADCReasult[0] = getBufSample(1);
    ADCReasult[1] = getBufSample(2);
    ADCReasult[2] = getBufSample(3);
    ADCReasult[3] = getBufSample(4);
  }
}

/******************************************************************************
                                others
******************************************************************************/

#if defined( IAR_ARMCM3_LM )
/*********************************************************************
 * @fn      GenericApp_ProcessRtosMessage
 *
 * @brief   Receive message from RTOS queue, send response back.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_ProcessRtosMessage( void )
{
  osalQueue_t inMsg;

  if ( osal_queue_receive( OsalQueue, &inMsg, 0 ) == pdPASS )
  {
    uint8 cmndId = inMsg.cmnd;
    uint32 counter = osal_build_uint32( inMsg.cbuf, 4 );

    switch ( cmndId )
    {
      case CMD_INCR:
        counter += 1;  /* Increment the incoming counter */
                       /* Intentionally fall through next case */

      case CMD_ECHO:
      {
        userQueue_t outMsg;

        outMsg.resp = RSP_CODE | cmndId;  /* Response ID */
        osal_buffer_uint32( outMsg.rbuf, counter );    /* Increment counter */
        osal_queue_send( UserQueue1, &outMsg, 0 );  /* Send back to UserTask */
        break;
      }

      default:
        break;  /* Ignore unknown command */
    }
  }
}
#endif




/*********************************************************************
 */
