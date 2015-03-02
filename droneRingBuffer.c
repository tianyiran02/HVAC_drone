/*
$License:
  Copy right    Yiran Tian, All Rights Reserved.
$
*/    

/******************************************************************************
*$Id: droneRingBuffer.c$
******************************************************************************/

/**
*  Describe:    This file provide essential ringbuffer function for drone 
*               ADC data.
*
*      
*  List of Funtions:
*               1. uint8 getBufSample(uint8): read data from ringbuffer
*               2. pushBufSample(): push one data to ringbuffer
*               3. uint8 drone_Datachange();
*               4. drone_GetADCData()
*               5. drone_PrepareData();
*
**/

/*********************************************************************
 * INCLUDES
 */
#include "droneRingBuffer.h"


/*********************************************************************
 * GLOBAL VARIABLES
 */
ADC_Ringbuffer_t DATA1_RingBuffer_pointer; // temp, ADC0
ADC_Ringbuffer_t DATA2_RingBuffer_pointer; // current, ADC1
ADC_Ringbuffer_t DATA3_RingBuffer_pointer; // HP, ADC2
ADC_Ringbuffer_t DATA4_RingBuffer_pointer; // LP, ADC3

// Windows size: 60 bytes
uint8 DATA1_RingBuffer[65] = {0};
uint8 DATA2_RingBuffer[65] = {0};
uint8 DATA3_RingBuffer[65] = {0};
uint8 DATA4_RingBuffer[65] = {0};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */
uint8 PushBufSample(uint8, uint8);
uint8 getBufSample(uint8);
void Sender_PrepareData( void );
void Sender_GetADCData( void );
uint8 Sender_Datachange( void );



/*********************************************************************
 * @fn      getBufSample
 *
 * @brief   Read data out from the ring buffer
 *
 * @param   
 *
 * @return  one byte data
 */
uint8 getBufSample(uint8 target)
{
  uint8 value = 0;
  
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
 * @return  one byte data
 */
uint8 pushBufSample(uint8 target, uint8 value)
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
 * @fn      drone_Datachange
 *
 * @brief   Compare the current data with the previous data,
 *
 * @param   
 *
 * @return  0 - not significant change; 1 - significant change
 */
uint8 drone_Datachange( void )
{
  uint8 temp = 0;
  uint8 Status = 0;
  
  // Divide by 60, median filter
  // Compare with previous data
  temp = ADCReasult60SUM[0] / 60;
  if(abs((signed int)temp - ADCReasult[0]) > TempTH)
    Status = 1;
  ADCReasult[0] = temp;
  
  temp = ADCReasult60SUM[1] / 60;
  if(abs((signed int)temp - ADCReasult[1]) > CurrentTH)
    Status = 1;
  ADCReasult[1] = temp;

  temp = ADCReasult60SUM[2] / 60;
  if(abs((signed int)temp - ADCReasult[2]) > HPressTH)
    Status = 1;
  ADCReasult[2] = temp;
  
  temp = ADCReasult60SUM[3] / 60;
  if(abs((signed int)temp - ADCReasult[3]) > HPressTH)
    Status = 1;
  ADCReasult[3] = temp;  
  
  return Status;
}

/*********************************************************************
 * @fn      Sender_GetADCData
 *
 * @brief   Read data from ADC
 *
 * @param   
 *
 * @return  
 */
void drone_GetADCData( void )
{
  uint8 temp = 0;
  
  temp = (uint8)(HalAdcRead (0, HAL_ADC_RESOLUTION_10) >> 1);
  PushBufSample(1,temp);
  
  temp = (uint8)(HalAdcRead (1, HAL_ADC_RESOLUTION_10) >> 1);
  PushBufSample(2,temp);  

  temp = (uint8)(HalAdcRead (2, HAL_ADC_RESOLUTION_10) >> 1);
  PushBufSample(3,temp);

  temp = (uint8)(HalAdcRead (3, HAL_ADC_RESOLUTION_10) >> 1);
  PushBufSample(4,temp);
  
  
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
void drone_PrepareData( void )
{
  uint8 temp = 0;
  
  // Flush the sumation buffer
  memset(ADCReasult60SUM,0,sizeof(ADCReasult60SUM));
  
  // Get sumation of the sampling window buffer
  for(temp=0;temp<60;temp++)
    ADCReasult60SUM[0] += getBufSample(1);
  
  for(temp=0;temp<60;temp++)
    ADCReasult60SUM[1] += getBufSample(2);

  for(temp=0;temp<60;temp++)
    ADCReasult60SUM[2] += getBufSample(3);
  
  for(temp=0;temp<60;temp++)
    ADCReasult60SUM[3] += getBufSample(4);
    
}