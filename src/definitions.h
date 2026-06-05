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
#include "SimplePWM.h"

#define DEG_PER_EDGE 0.33445f

#define HIP_MOTOR_TEETH 20.0f
#define HIP_ROBOT_TEETH 68.0f
#define HIP_MOTOR_PER_ROBOT (HIP_ROBOT_TEETH / HIP_MOTOR_TEETH) // 3.4
#define HIP_ROBOT_PER_MOTOR (HIP_MOTOR_TEETH / HIP_ROBOT_TEETH) // 0.2941176

/////////////////////// Hip PID stuff///////////////////////////////////
PID HipControl;
float HipGains[3] = {1.4f, 0.4f, 0.5f}; // Kp, Ki, Kd
float HipReferenceRobot = 0.0f;   // grados reales del robot
float HipReferenceMotor = 0.0f;   // grados del eje del stepper
float HipMeasurement = 0.0f;      // grados del eje del stepper, multi-turn
float HipError = 0.0f;
float Hip_u = 0.0f; 
// Stepper position control limits
const float HIP_TOLERANCE_DEG = 0.3f; // init: 1
const uint32_t HIP_MIN_FREQ = 18;  // init: 100
const uint32_t HIP_MAX_FREQ = 1500;

/////////////////////// Elbow PID stuff /////////////////////////////////
PID ElbowControl;
float ElbowGains[3] = {0.5f, 0.0f, 0.2f}; // Kp, Ki, Kd
float ElbowReferenceRobot = 0.0f;   
float ElbowReferenceMotor = 0.0f;   
float ElbowMeasurement = 0.0f;      
float ElbowError = 0.0f;
float Elbow_u = 0.0f; 

// Help variables 
volatile float ElbowSpeed,getWirstAngle, rawWristAngle; // Wirst
uint32_t frequency = 1000;
static uint32_t lastHipFrequency;
volatile int direction = 1, GripperOnOff;

enum MODE{
    NOTHING = 0,
    SPEED_CONTROL,
    ANGULAR_CONTROL,
    PWM_CONTROL,
};
MODE current_mode;

///////////////// ELBOW DCM STUFF ////////////////////////////////////
uint8_t ElbowPIN[2] = {25, 26};uint8_t ElbowPWMCH[2] = {0, 1}; //PWM Channels
HBridge ElbowDCM; //HBridge class instance
//Quadrature Encoder Stuff
QuadratureEncoder ElbowEncoder;
uint8_t ElbowEncPIN[] = {34,35};

///////////////// WRIST DCM STUFF ////////////////////////////////////
uint8_t WristPIN[2] = {32, 33};uint8_t WristPWMCH[2] = {2, 3}; //PWM Channels
HBridge WristDCM; //HBridge class instance
//Quadrature Encoder Stuff
QuadratureEncoder WristEncoder;
uint8_t WristEncPIN[] = {39,36}; 

///////////////// STEPPER HIP STUFF //////////////////////////////////
SimplePWM Step1;
SimpleGPIO Dir1;
const uint8_t step_pin1 = 18, dir_pin1 = 19,step1_channel = 4;
TimerConfig stepper1_config{
    .timer = LEDC_TIMER_1,
    .frequency = 1350, // 1kHz
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
    .frequency = 1050, // 400Hz
    .bit_resolution = LEDC_TIMER_10_BIT,
    .mode = LEDC_HIGH_SPEED_MODE};
// Limit Switches Up Down stuff
SimpleGPIO Down_LS; const uint8_t DownPin = 4;
SimpleGPIO Up_LS; const uint8_t UpPin = 27;

///////////////////// GRIPPER  stuff //////////////////////////////////
SimpleGPIO Gripper; const uint8_t GripperPin = 13;

// while Timer Actuation Stuff
SimpleTimer timer1;
bool flag1 = false;
uint64_t dt_us1 = 10000; // 1 ms = 1000 us

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