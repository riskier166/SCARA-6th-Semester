#include <definitions.h>

static void IRAM_ATTR timerISR(void *arg); void setup();

extern "C" void app_main()
{
    setup();
    while (1)
    {
        if (timer.interruptAvailable())
        {
            // Elbow DC control
            elbow_measurement = ElbowEncoder.getAngle();
            elbow_error = elbow_reference - elbow_measurement;
            elbow_u = ElbowControl.calc(elbow_error);
            Elbow.setSpeed(elbow_reference); 

            // Wrist DC control
            wrist_measurement = WristEncoder.getAngle();
            wrist_error = wrist_reference - wrist_measurement;
            wrist_u = WristControl.calc(wrist_error);
            Wrist.setSpeed(wrist_reference); 
            
            // UART Communication
            message_length = uart.available();
            if (message_length)
            {
                uart.read(buffer, message_length); // Echo back
                sscanf(buffer, "%f,%f", &elbow_reference, &wrist_reference);
                // sscanf(buffer, "%f,%f,%f,%f,%f,%f,%f,%f\n"
                // , &elbow_gains[0], &elbow_gains[1], &elbow_gains[2], &elbow_reference
                // , &wrist_gains[0], &wrist_gains[1], &wrist_gains[2], &wrist_reference); // DC Angular position UART
                // ElbowControl.setup(elbow_gains, dt_us / 1000000.0f); // Elbow PID Gains setup
                // WristControl.setup(wrist_gains, dt_us / 1000000.0f); // Wrist PID Gains setup
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

    // ELBOW DC1 setup
    Elbow.setup(ELBOW_PINS, ELBOW_PWMCH);
    ElbowEncoder.setup(ENC_ELBOW_PINS, degrees_per_edge);
    ElbowControl.setup(wrist_gains, dt_us / 1000000.0f);

    // ELBOW DC1 setup
    Wrist.setup(WRIST_PINS, WRIST_PWMCH);
    WristEncoder.setup(ENC_Wrist_PINS, degrees_per_edge);
    WristControl.setup(wrist_gains, dt_us / 1000000.0f);

    timer.setup(timerISR, "MainTimer");timer.startPeriodic(dt_us);
}