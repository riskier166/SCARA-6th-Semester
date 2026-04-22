#include <definitions.h>

static void IRAM_ATTR timerISR(void *arg); void setup();

extern "C" void app_main()
{
    setup();
    while (1)
    {
        if (timer.interruptAvailable())
        {
            // Angular DC control
            measurement = encoder.getAngle();
            error = reference - measurement;
            u = control.calc(error);
            MOTOR_PWM.setSpeed(u); 
            
            // UART Communication
            message_length = uart.available();
            if (message_length)
            {
                uart.read(buffer, message_length); // Echo back
                sscanf(buffer, "%f,%f,%f,%f\n", &gains[0], &gains[1], &gains[2], &reference); // DC Angular position UART
                control.setup(gains, dt_us / 1000000.0f);
            } 
            //printf("%.2f,%.2f,%.2f,%.2f\n",u, measurement, encoder.getSpeed(), encoder.getAngle());
        }
    }
}

static void IRAM_ATTR timerISR(void *arg)
{
    timer.setInterrupt();
}

void setup()
{
    esp_task_wdt_deinit();

    // Motor DC1 setup
    encoder.setup(EncIN, degrees_per_edge);
    control.setup(gains, dt_us / 1000000.0f);
    MOTOR_PWM.setup(PWM_PIN, PWMCH);

    timer.setup(timerISR, "MainTimer");timer.startPeriodic(dt_us);
}