#include <definitions.h>

static void IRAM_ATTR timerISR(void *arg); void setups();

extern "C" void app_main()
{
    esp_task_wdt_deinit(); setups();
    while (1)
    {
        if (timer.interruptAvailable())
        {
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
    // DC Motor setup
    MOTOR_PWM.setup(PWM_PIN, PWMCH);
    // Quadrature encoder setup
    encoder.setup(EncIN, degrees_per_edge);
    control.setup(gains, dt_us / 1000000.0f);

    // While timer Setup
    timer.setup(timerISR, "MainTimer");timer.startPeriodic(dt_us);

    // Stepper HIP setup 
    Step1.setup(step_pin1, step1_channel, &stepper1_config);
    Dir1.setup(dir_pin1, GPO);
    // Absollute Encoder Setup
    i2c.init(); AbsEnc.init();
}

static void IRAM_ATTR timerISR(void *arg)
{
    timer.setInterrupt();
}