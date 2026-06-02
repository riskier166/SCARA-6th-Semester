#include <definitions.h>

static void IRAM_ATTR timerISR1(void *arg), timerISR2(void *arg);
void setups();

esp_err_t create_tasks();

void actuation(void *arg)
{
    while (1)
    {
        if (timer1.interruptAvailable())
        {
            // DCM
            WristDCM.setSpeed(wirstSpeed);
            // Activate HIP Stepper
            Step1.setDuty(50);
            //Step1.setFrequency(frequency);
            Dir1.set(1);
            // Activate UpDown Stepper
            Step2.setDuty(80);
            Dir2.set(1);

            message_length = uart.available();
            if (message_length)
            {
                uart.read(buffer, message_length); // Echo back
                sscanf(buffer, "%f,%f", &wirstSpeed,&frequency);
            }
        }
    }
}

void prints_help(void *arg)
{
    while (1)
    {
        if (timer2.interruptAvailable())
        {
            rawWristAngle = WristEncoder.getAngle();
            getWirstAngle = wrapAngle360(rawWristAngle);

            if (AbsEnc.update() == ESP_OK)
            {
                printf("Hip angle: %.2f deg, Wrist Angle: %.2f deg\n",
                       AbsEnc.getAngleDegrees(),
                       getWirstAngle);
            }
        }
    }
}

extern "C" void app_main()
{
    esp_task_wdt_deinit();
    setups();
    create_tasks();
}

void setups()
{
    //// Elbow DCM setup
    ElbowDCM.setup(ElbowPIN, ElbowPWMCH);
    /// Quadrature encoder setup
    ElbowEncoder.setup(ElbowEncPIN, DEG_PER_EDGE);
    control.setup(gains, dt_us1 / 1000000.0f); // PID
    // Calibration Limit Switch setup
    Calibration.setup(CalibPin, GPI);

    // Wrist DCM setup
    WristDCM.setup(WristPIN, WristPWMCH);
    // Quadrature encoder setup
    WristEncoder.setup(WristEncPIN, DEG_PER_EDGE);

    // While timer Actuation Setup
    timer1.setup(timerISR1, "ActuationTimer");
    timer1.startPeriodic(dt_us1);

    // While timer Prints Setup
    timer2.setup(timerISR2, "PrintsTimer");
    timer2.startPeriodic(dt_us2);

    // Stepper HIP setup
    Step1.setup(step_pin1, step1_channel, &stepper1_config);
    Dir1.setup(dir_pin1, GPO);
    /// Absolute Encoder Setup
    i2c.init();
    AbsEnc.init();

    // Stepper UpDown Setup
    Step2.setup(step_pin2, step2_channel, &stepper2_config);
    Dir2.setup(dir_pin2, GPO);
    /// UpDown Limit Switch Setup
    UpDown_LS.setup(UpDownPin, GPI);

    // Gripper Setup
    Gripper.setup(GripperPin, GPO);
}

static void IRAM_ATTR timerISR1(void *arg)
{
    timer1.setInterrupt();
}

static void IRAM_ATTR timerISR2(void *arg)
{
    timer2.setInterrupt();
}

esp_err_t create_tasks()
{
    static uint8_t ucParameterToPass;
    TaskHandle_t xHandle = NULL;
    xTaskCreate(actuation,
                "Commutation",
                4096,
                &ucParameterToPass,
                1,
                &xHandle);
    xTaskCreate(prints_help,
                "Current Control",
                4096,
                &ucParameterToPass,
                2,
                &xHandle);
    return ESP_OK;
}