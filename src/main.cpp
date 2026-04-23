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
    while (1)
    {
        if (timer.interruptAvailable())
        {
            measurement = encoder.getAngle();
            error = reference - measurement;
            u = fabs(control.calc(error));
            if (u>95) u=95;
            else if (u<0) u=0; 
            MOTOR_PWM.setSpeed(u);

            if (fabs(error) <= 0 ) 
            {
                measurement = 0; reference = 0;
            }

            message_length = uart.available();
            if (message_length)
            {
                uart.read(buffer, message_length); // Echo back
                sscanf(buffer, "%f,%f,%f,%f\n", &gains[0], &gains[1], &gains[2], &reference);
                // sscanf(buffer, "%f,%f,%f,%f\n", &gains[0], &gains[1], &gains[2], &reference);
                control.setup(gains, dt_us / 1000000.0f);
            }
            printf("U: %.2f, Measured position: %.2f, desired angle: %.2f, error: %.2f\n", u, measurement, reference, error);
        }
    }
}

static void IRAM_ATTR timerISR(void *arg)
{
    timer.setInterrupt();
}