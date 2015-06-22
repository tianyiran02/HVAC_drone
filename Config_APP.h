/*********************************************************************
  This .h file contain essential parameters to setup firmware

  Use Macro to define different parameters, for example, buffer 
  offset, drone ID

  @Author Yiran Tian 2015/5/27
*********************************************************************/

#ifndef CONFIG_APP_H
#define CONFIG_APP_H

/*********** Essential ***************/
// drone ID
#define DRONE_DEVICE_ID         3

// Basic upload period, in mintues
#define UpLoad_NormalPeriod     5   

/********* Detail ***********/
// Reset Value

// ADC related
// The threshould was setting to 5% of overall measuring range (0.25v)
#define TempTH      410  
#define CurrentTH   410  
#define LPressTH    410  
#define HPressTH    410   
// offsets
#define minTempOffset   70
#define CurrentOffset   20
#define LPressOffset    20
#define HPressOffset    20

#define DRONE_FAILURE_MAXIMUN   2       // failure receive ACK, reset drone
#define DRONE_ACKWAITTIME       10      // ACK reset wait time in second

#endif