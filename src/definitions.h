#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#include <stdio.h>
#include <SimpleADC.h>
#include <SimpleTimer.h>
#include <SimpleUART.h>
#include <HBridge.h>
#include <QuadratureEncoder.h>
#include <esp_task_wdt.h> //Watch Dog 
#include <PID.h>
#include <SimpleGPIO.h>

/////////////////////  ELBOW DC1 Stuff  ///////////////////////////
//HBridge Stuff
uint8_t ELBOW_PINS[2] = {32, 33},ELBOW_PWMCH[2] = {0, 1}; 
HBridge Elbow; //HBridge class instance
//Quadrature Encoder Stuff
QuadratureEncoder ElbowEncoder;
uint8_t ENC_ELBOW_PINS[] = {36, 39}; const float degrees_per_edge = 0.36437;
//PID DC1 help stuff
PID ElbowControl;
float elbow_gains[3];float elbow_reference=0;
float elbow_measurement;float elbow_error; float elbow_u;

/////////////////////  WRIST DC1 Stuff  ///////////////////////////
//HBridge Stuff
uint8_t WRIST_PINS[2] = {25, 26},WRIST_PWMCH[2] = {2, 3}; 
HBridge Wrist; //HBridge class instance
//Quadrature Encoder Stuff
QuadratureEncoder WristEncoder;
uint8_t ENC_Wrist_PINS[] = {35, 34};
//PID DC1 help stuff
PID WristControl;
float wrist_gains[3];float wrist_reference=0;
float wrist_measurement;float wrist_error; float wrist_u;


// while Timer Stuff
SimpleTimer timer;
bool flag = false;
uint64_t dt_us = 10000; // 10 ms = 10000 us

//UART Stuff
SimpleUART uart(115200); //UART class
char buffer[30];int message_length;

#endif // __DEFINITIONS_H__