#include <definitions.h>

static void IRAM_ATTR timerISR(void *arg);

extern "C" void app_main()
{
    esp_task_wdt_deinit();
    encoder.setup(EncIN, degrees_per_edge);
    control.setup(gains, dt_us / 1000000.0f);
    MOTOR_PWM.setup(PWM_PIN, PWMCH);
    timer.setup(timerISR, "MainTimer");
    timer.startPeriodic(dt_us);
    i2c.init();
    while (1)
    {
        if (timer.interruptAvailable())
        {
            if (i2c.deviceAvailable(0x36))
            {
                ESP_LOGI("MAIN", "AS5600 detected");
            }
            else
            {
                ESP_LOGE("MAIN", "AS5600 not detected");
            }
        }
    }
}

static void IRAM_ATTR timerISR(void *arg)
{
    timer.setInterrupt();
}