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

///////////////// ELBOW DCM STUFF ////////////////////////////////////
uint8_t PWM_PIN[2] = {32, 33};uint8_t PWMCH[2] = {0, 1}; //PWM Channels
HBridge MOTOR_PWM; //HBridge class instance
//Quadrature Encoder Stuff
QuadratureEncoder encoder;
uint8_t EncIN[] = {36, 39}; const float degrees_per_edge = 0.36437;

///////////////// STEPPER HIP STUFF //////////////////////////////////
SimplePWM Step1;
SimpleGPIO Dir1;
const uint8_t step_pin1 = 18, dir_pin1 = 19,step1_channel = 4;
TimerConfig stepper1_config{
    .timer = LEDC_TIMER_1,
    .frequency = 1000, // 400Hz
    .bit_resolution = LEDC_TIMER_10_BIT,
    .mode = LEDC_HIGH_SPEED_MODE};
// Absolute encoder (12c) stuff
I2CBus i2c(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 400000);
AS5600 AbsEnc(&i2c);

///////////////// UP-DOWN Stepper stuff //////////////////////////////
SimplePWM Step2;
SimpleGPIO Dir2;
const uint8_t step_pin2 = 33, dir_pin2 = 32,step2_channel = 1;
TimerConfig stepper2_config{
    .timer = LEDC_TIMER_0,
    .frequency = 1000, // 400Hz
    .bit_resolution = LEDC_TIMER_10_BIT,
    .mode = LEDC_HIGH_SPEED_MODE};
// Help vars
int frequency = 1000, direction2, on_off_2; 
// Limit switches stuff
SimpleGPIO down_LS, up_LS;
int8_t DOWN_PIN = 35, UP_PIN = 34;

// while Timer Stuff
SimpleTimer timer;
bool flag = false;
uint64_t dt_us = 10000; // 10 ms = 10000 us

//UART Stuff
SimpleUART uart(115200); //UART class
char buffer[30];int message_length;

#endif // __DEFINITIONS_H__