#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "mpu9250.h"
#include "hardware/spi.h"
#include "hardware/adc.h"
#include "hardware/timer.h"

//Timers del FreeRTOS
#include "timers.h"

#define measureTIMER_PERIOD pdMS_TO_TICKS(2500)

//Callback del timer cada seg
void measureTimerCallback(TimerHandle_t xTimer);


int main()
{
    stdio_init_all();
    TimerHandle_t xMeasureTimer;
    BaseType_t xMeasureTimerStarted;

    xMeasureTimer = xTimerCreate("MeasureTimer", measureTIMER_PERIOD, pdTRUE, 0, measureTimerCallback);
    

    if(xMeasureTimer != NULL){
        xMeasureTimerStarted = xTimerStart(xMeasureTimer, 0);

        if(xMeasureTimerStarted == pdPASS){
            vTaskStartScheduler();
        }
    }
    for(;;);
}

void measureTimerCallback(TimerHandle_t xTimer){
    printf("Este mensaje se mostrará cada 2.5 segundos\n");
}