#include <definitions.h>

static void IRAM_ATTR timerISR1(void *arg), timerISR2(void *arg);
void setups(), HipPositionControl(float reference), write_message(), ElbowPositionControl(float reference);

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
            // Elbow DC motor control
            ElbowPositionControl(ElbowReferenceRobot);
            // Hip absolute positioning
            HipPositionControl(HipReferenceRobot);
            // UpDown stepper
            Dir2.set(direction);
            if (Down_LS.get() == 1 && direction == 1)
                Step2.setDuty(0);
            else if (Up_LS.get() == 1 && direction == 0)
                Step2.setDuty(0);
            else
                Step2.setDuty(50);
            // Gripper activation
            Gripper.set(GripperOnOff);
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

            printf("Hip Ref Robot: %.2f, Hip Ref Motor: %.2f, Hip Motor Angle: %.2f, Hip error: %.2f, Elbow Robot Ref: %.2f, Elbow Motor Ref: %.2f, Elbow angle: %.2f\n, Elbow Error: %.2f",
                   HipReferenceRobot,     HipReferenceMotor,       HipMeasurement,     HipError,   ElbowReferenceRobot,   ElbowReferenceMotor,    ElbowMeasurement,       ElbowError
                );
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

        float newElbowReference;
        float newReference;
        int newDirection, newGripperOnOff;

        int parsed = sscanf(buffer, "%f,%f,%d,%d",
                            &newElbowReference,
                            &newReference,
                            &newDirection,
                            &newGripperOnOff);
        if (parsed == 4)
        {
            // Elbow Speed condition
            if (newElbowReference <= 360 || newElbowReference >= 0)
                ElbowReferenceRobot = newElbowReference;
            else
                printf("Invalid Elbow speed: %.2f\n", newElbowReference);

            // Hip condition
            if (newReference >= 0.0f && newReference <= 360.0f)
                HipReferenceRobot = newReference;
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

void ElbowPositionControl(float reference)
{
    ElbowMeasurement = ElbowEncoder.getAngle();
    ElbowReferenceMotor = reference * HIP_MOTOR_PER_ROBOT;
    ElbowError = ElbowReferenceMotor - ElbowMeasurement;
    Elbow_u = ElbowControl.calc(ElbowError);
    if (Elbow_u > 70) Elbow_u = 70;
    else if (Elbow_u < -70) Elbow_u = -70;
    ElbowDCM.setSpeed(Elbow_u);
}

void HipPositionControl(float reference_robot_deg)
{
    esp_err_t err = AbsEnc.update();

    if (err != ESP_OK)
    {
        Step1.setDuty(0);
        return;
    }

    // 360° robot = 1224° motor.
    HipReferenceMotor = reference_robot_deg * HIP_MOTOR_PER_ROBOT;

    HipMeasurement = -1.0 * AbsEnc.getContinuousAngleDegrees(); // Cambia de signo

    // Multi-turn error.
    HipError = HipReferenceMotor - HipMeasurement;

    float tolerance_motor_deg = HIP_TOLERANCE_DEG * HIP_MOTOR_PER_ROBOT;

    if (fabs(HipError) <= tolerance_motor_deg)
    {
        Step1.setDuty(0);
        lastHipFrequency = 0;
        Hip_u = 0.0f;
        return;
    }

    // Direction depends on error sign.
    if (HipError > 0.0f)
    {
        Dir1.set(1);
    }
    else
    {
        Dir1.set(0);
    }

    // For stepper control, use absolute error for speed.
    Hip_u = HipControl.calc(fabs(HipError));

    frequency = (uint32_t)fabs(Hip_u);

    if (frequency < HIP_MIN_FREQ)
    {
        frequency = HIP_MIN_FREQ;
    }

    if (frequency > HIP_MAX_FREQ)
    {
        frequency = HIP_MAX_FREQ;
    }

    if (frequency != lastHipFrequency)
    {
        Step1.setFrequency(frequency);
        lastHipFrequency = frequency;
    }

    Step1.setDuty(50);
}

void setups()
{
    //// Elbow DCM setup
    ElbowDCM.setup(ElbowPIN, ElbowPWMCH);
    /// Quadrature encoder setup
    ElbowEncoder.setup(ElbowEncPIN, DEG_PER_EDGE);
    ElbowControl.setup(ElbowGains, dt_us1 / 1000000.0f); // PID

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
    AbsEnc.resetContinuousAngle(0.0f);
    HipControl.setup(HipGains, dt_us1 / 1000000.0f); // PID

    // Stepper UpDown Setup
    Step2.setup(step_pin2, step2_channel, &stepper2_config);
    Dir2.setup(dir_pin2, GPO);
    Down_LS.setup(DownPin, GPI); // Down Limit Switch Setup
    Up_LS.setup(UpPin, GPI);     // Up Limit Switch Setup

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