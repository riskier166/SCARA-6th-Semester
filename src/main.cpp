#include <definitions.h>

static void IRAM_ATTR timerISR1(void *arg), timerISR2(void *arg);
void setups(), HipPositionControl(float reference), write_message();

esp_err_t create_tasks();

void actuation(void *arg)
{
    lastHipFrequency = 0;

    Step1.setDuty(0); // Hip stepper initially stopped

    while (1)
    {
        if (timer1.interruptAvailable())
        {
            message_length = uart.available();
            write_message();

            // Wrist DC motor control
            WristDCM.setSpeed(wirstSpeed);

            // Hip absolute positioning
            HipPositionControl(HipReference);

            // UpDown stepper
            Dir2.set(direction);
            if (UpDown_LS.get() == 1 && direction == 1)
                Step2.setDuty(0);
            else if (Calibration.get() == 1 && direction == 0)
                Step2.setDuty(0);
            else
                Step2.setDuty(50);

            // Gripper activation
            Gripper.set(GripperOnOff);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
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

            printf("Hip Ref: %.2f deg, Hip Angle: %.2f deg, Error: %.2f deg, u: %.2f Hz, Wrist Angle: %.2f deg\n",
                   HipReference,
                   HipMeasurement,
                   HipError,
                   Hip_u,
                   getWirstAngle);
        }
    }
}

extern "C" void app_main()
{
    esp_task_wdt_deinit();
    setups();
    create_tasks();
}

void write_message()
{
    if (message_length > 0)
    {
        if (message_length >= sizeof(buffer))
            message_length = sizeof(buffer) - 1;

        uart.read(buffer, message_length);
        buffer[message_length] = '\0';

        float newWristSpeed;
        float newReference;
        int newDirection, newGripperOnOff;

        int parsed = sscanf(buffer, "%f,%f,%d,%d",
                            &newWristSpeed,
                            &newReference,
                            &newDirection,
                            &newGripperOnOff);
        if (parsed == 4)
        {
            wirstSpeed = newWristSpeed;

            // Hip condition 
            if (newReference >= 0.0f && newReference < 360.0f)
                HipReference = newReference;
            else
                printf("Invalid hip reference: %.2f\n", newReference);

            // UpDown Condition 
            if (newDirection == 0 || newDirection == 1)
                direction = newDirection;
            else
                printf("Invalid direction: %d\n", newDirection);

            // Gripper condition 
            if (newGripperOnOff == 0 || newGripperOnOff == 1)
                GripperOnOff = newGripperOnOff;
            else
                printf("Invalid gripper state: %d\n", newGripperOnOff);
        }
        else
            printf("UART parse error: %s\n", buffer);
    }
}

void HipPositionControl(float Reference)
{
    HipReference = Reference;
    if (AbsEnc.update() == ESP_OK)
    {
        HipMeasurement = AbsEnc.getAngleDegrees();
        HipError = shortestAngleError(HipReference, HipMeasurement);

        if (fabsf(HipError) <= HIP_TOLERANCE_DEG)
        {
            Step1.setDuty(0);
            lastHipFrequency = 0;
        }
        else
        {
            if (HipError > 0.0f)
                Dir1.set(0);
            else
                Dir1.set(1);

            float absError = fabsf(HipError);

            uint32_t hipFrequency = (uint32_t)(8.0f * absError);

            if (hipFrequency > 1000)
                hipFrequency = 1000;

            if (hipFrequency < 50)
                hipFrequency = 50;

            if (hipFrequency != lastHipFrequency)
            {
                Step1.setFrequency(hipFrequency);
                lastHipFrequency = hipFrequency;
            }

            Step1.setDuty(50);
        }
    }
}

void setups()
{
    //// Elbow DCM setup
    ElbowDCM.setup(ElbowPIN, ElbowPWMCH);
    /// Quadrature encoder setup
    ElbowEncoder.setup(ElbowEncPIN, DEG_PER_EDGE);
    // Calibration Limit Switch setup
    Calibration.setup(CalibPin, GPI);

    // Wrist DCM setup
    WristDCM.setup(WristPIN, WristPWMCH);
    // Quadrature encoder setup
    WristEncoder.setup(WristEncPIN, DEG_PER_EDGE);

    // Stepper HIP setup
    Step1.setup(step_pin1, step1_channel, &stepper1_config);
    Dir1.setup(dir_pin1, GPO);
    /// Absolute Encoder Setup
    i2c.init();
    AbsEnc.init();
    HipControl.setup(HipGains, dt_us1 / 1000000.0f); // PID

    // Stepper UpDown Setup
    Step2.setup(step_pin2, step2_channel, &stepper2_config);
    Dir2.setup(dir_pin2, GPO);
    /// UpDown Limit Switch Setup
    UpDown_LS.setup(UpDownPin, GPI);

    // Gripper Setup
    Gripper.setup(GripperPin, GPO);

    // While timer Actuation Setup
    timer1.setup(timerISR1, "ActuationTimer");
    timer1.startPeriodic(dt_us1);

    // While timer Prints Setup
    timer2.setup(timerISR2, "PrintsTimer");
    timer2.startPeriodic(dt_us2);
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
                2,
                &xHandle);
    xTaskCreate(prints_help,
                "Current Control",
                4096,
                &ucParameterToPass,
                1,
                &xHandle);
    return ESP_OK;
}