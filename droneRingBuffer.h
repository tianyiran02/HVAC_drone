/*
$License:
  Copy right    Yiran Tian, All Rights Reserved.
$
*/    

/******************************************************************************
*$Id: droneRingBuffer.h$
******************************************************************************/

#ifndef DRONERINGBUFFER_H
#define DRONERINGBUFFER_H

#include "ZComDef.h"

/*********************************************************************
 * GLOBAL VARIABLES
 */
typedef struct
{
  uint8               *pIn;
  uint8               *pOut;
} ADC_Ringbuffer_t;

extern ADC_Ringbuffer_t DATA1_RingBuffer_pointer; // temp, ADC0
extern ADC_Ringbuffer_t DATA2_RingBuffer_pointer; // current, ADC1
extern ADC_Ringbuffer_t DATA3_RingBuffer_pointer; // HP, ADC2
extern ADC_Ringbuffer_t DATA4_RingBuffer_pointer; // LP, ADC3

// Windows size: 60 bytes
extern uint8 DATA1_RingBuffer[65];
extern uint8 DATA2_RingBuffer[65];
extern uint8 DATA3_RingBuffer[65];
extern uint8 DATA4_RingBuffer[65];


/*********************************************************************
 * PUBLIC FUNCTIONS
 */
uint8 PushBufSample(uint8, uint8);
uint8 getBufSample(uint8);
void Sender_PrepareData( void );
void Sender_GetADCData( void );
uint8 Sender_Datachange( void );

#endif