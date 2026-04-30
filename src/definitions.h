#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#include <stdio.h>
#include <SimpleADC.h>
#include <SimpleTimer.h>
#include <SimpleUART.h>
#include <HBridge.h>
#include <QuadratureEncoder.h>
#include "esp_task_wdt.h" //Watch Dog 
#include "PID.h"
#include "math.h"
#include "I2CBus.h"

//PID stuff
PID control;
float gains[3];
float reference=0;
int mode;

float measurement;float error; float u;

enum MODE{
    NOTHING = 0,
    SPEED_CONTROL,
    ANGULAR_CONTROL,
    PWM_CONTROL,
};
MODE current_mode;

//HBridge Stuff
uint8_t PWM_PIN[2] = {32, 33};uint8_t PWMCH[2] = {0, 1}; //PWM Channels
HBridge MOTOR_PWM; //HBridge class instance

//Quadrature Encoder Stuff
QuadratureEncoder encoder;
uint8_t EncIN[] = {36, 39}; const float degrees_per_edge = 0.36437;

///////////////// STEPPER MOTOR STUFF //////////////////////////////////
// Absolute encoder (12c) stuff
I2CBus i2c(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 400000);

// while Timer Stuff
SimpleTimer timer;
bool flag = false;
uint64_t dt_us = 10000; // 10 ms = 10000 us

//UART Stuff
SimpleUART uart(115200); //UART class
char buffer[30];int message_length;

#endif // __DEFINITIONS_H__