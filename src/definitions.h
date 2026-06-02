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
#include "AS5600.h"
#include "SimpleGPIO.h"

#define DEG_PER_EDGE 0.33445f

//PID stuff
PID control;
float gains[3];
float reference=0;
int mode;
float measurement;float error; float u;

// Help variables 
volatile float wirstSpeed,getWirstAngle, rawWristAngle; // Wirst
float frequency = 1000; // Hip 

enum MODE{
    NOTHING = 0,
    SPEED_CONTROL,
    ANGULAR_CONTROL,
    PWM_CONTROL,
};
MODE current_mode;

///////////////// ELBOW DCM STUFF ////////////////////////////////////
uint8_t ElbowPIN[2] = {32, 33};uint8_t ElbowPWMCH[2] = {0, 1}; //PWM Channels
HBridge ElbowDCM; //HBridge class instance
//Quadrature Encoder Stuff
QuadratureEncoder ElbowEncoder;
uint8_t ElbowEncPIN[] = {39,36};
// Calibration Limit Switch Stuff 
SimpleGPIO Calibration; const uint8_t CalibPin = 27;

///////////////// WRIST DCM STUFF ////////////////////////////////////
uint8_t WristPIN[2] = {25, 26};uint8_t WristPWMCH[2] = {2, 3}; //PWM Channels
HBridge WristDCM; //HBridge class instance
//Quadrature Encoder Stuff
QuadratureEncoder WristEncoder;
uint8_t WristEncPIN[] = {34,35}; 

///////////////// STEPPER HIP STUFF //////////////////////////////////
SimplePWM Step1;
SimpleGPIO Dir1;
const uint8_t step_pin1 = 18, dir_pin1 = 19,step1_channel = 4;
TimerConfig stepper1_config{
    .timer = LEDC_TIMER_1,
    .frequency = 1000, // 1kHz
    .bit_resolution = LEDC_TIMER_10_BIT,
    .mode = LEDC_HIGH_SPEED_MODE};
// Absolute encoder (12c) stuff
I2CBus i2c(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 400000);
AS5600 AbsEnc(&i2c);

///////////////// UP-DOWN Stepper stuff //////////////////////////////
SimplePWM Step2;
SimpleGPIO Dir2;
const uint8_t step_pin2 = 16, dir_pin2 = 17,step2_channel = 5;
TimerConfig stepper2_config{
    .timer = LEDC_TIMER_0,
    .frequency = 1000, // 400Hz
    .bit_resolution = LEDC_TIMER_10_BIT,
    .mode = LEDC_HIGH_SPEED_MODE};
// Limit Up Down stuff
SimpleGPIO UpDown_LS; const uint8_t UpDownPin = 4;

///////////////////// GRIPPER  stuff //////////////////////////////////
SimpleGPIO Gripper; const uint8_t GripperPin = 13;

// while Timer Actuation Stuff
SimpleTimer timer1;
bool flag1 = false;
uint64_t dt_us1 = 1000; // 10 ms = 10000 us

// while Timer Prints Stuff
SimpleTimer timer2;
bool flag2 = false;
uint64_t dt_us2 = 10000; // 10 ms = 10000 us

// UART Stuff
SimpleUART uart(115200); //UART class
char buffer[30];int message_length;

// Reset Quadrature function 
float wrapAngle360(float angle)
{
    while (angle >= 360.0f)
    {
        angle -= 360.0f;
    }

    while (angle <= -360.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

#endif // __DEFINITIONS_H__