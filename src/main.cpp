#include <definitions.h>

static void IRAM_ATTR timerISR(void *arg); void setups();

extern "C" void app_main()
{
    esp_task_wdt_deinit(); setups();
    while (1)
    {
        if (timer.interruptAvailable())
        {
            // WristDCM.setSpeed(speed);
            // getSpeed = WristEncoder.getSpeed();
            // getAngle = WristEncoder.getAngle();

            // message_length = uart.available();
            // if (message_length)
            // {
            //     uart.read(buffer, message_length); // Echo back
            //     sscanf(buffer, "%f", &speed);
            // }
            // printf("Speed: %f,%f,%f\n", speed, getSpeed, getAngle);

            Step1.setDuty(80);
            Dir1.set(1);
            if (AbsEnc.update() == ESP_OK)
            {
                printf("Angle: %.2f deg | Speed: %.2f deg/s\n",
                   AbsEnc.getAngleDegrees(),
                   AbsEnc.getAngularVelocityDegS());
            }
        }
    }  
}

void setups()
{
    //// Elbow DCM setup
    ElbowDCM.setup(ElbowPIN, ElbowPWMCH);
    /// Quadrature encoder setup
    ElbowEncoder.setup(ElbowEncPIN, degrees_per_edge);
    control.setup(gains, dt_us / 1000000.0f); //PID 
    // Calibration Limit Switch setup 
    Calibration.setup(CalibPin,GPI);

    // Wrist DCM setup
    WristDCM.setup(WristPIN, WristPWMCH);
    // Quadrature encoder setup
    WristEncoder.setup(WristEncPIN, degrees_per_edge);

    // While timer Setup
    timer.setup(timerISR, "MainTimer");timer.startPeriodic(dt_us);

    // Stepper HIP setup 
    Step1.setup(step_pin1, step1_channel, &stepper1_config);
    Dir1.setup(dir_pin1, GPO);
    /// Absolute Encoder Setup
    i2c.init(); AbsEnc.init();

    // Stepper UpDown Setup 
    Step2.setup(step_pin2, step2_channel, &stepper2_config);
    Dir2.setup(dir_pin2, GPO);
    /// UpDown Limit Switch Setup 
    UpDown_LS.setup(UpDownPin,GPI);

    // Gripper Setup 
    Gripper.setup(GripperPin,GPO);
}

static void IRAM_ATTR timerISR(void *arg)
{
    timer.setInterrupt();
}