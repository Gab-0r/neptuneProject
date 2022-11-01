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

//Grupo de eventos para tomar medidas
#define measureBIT_0 (1UL << 0UL)
#define measureBIT_1 (1UL << 1UL)
#define measureBIT_2 (1UL << 2UL)
#define measureBIT_3 (1UL << 3UL)

//Event group de medida
EventGroupHandle_t xEventGroup;

//Tareas
void readIMUTask(void *pvParameters);

void settingTask(void *pvParameters);

int main()
{
    stdio_init_all();

    sleep_ms(5000);
    printf("Iniciando...\r\n");

    printf("Creando grupos de eventos...\r\n");
    xEventGroup = xEventGroupCreate();

    printf("Creando tareas...\r\n");
    xTaskCreate(readIMUTask, "IMUReader", 1000, NULL, 2, NULL);
    xTaskCreate(settingTask, "BitSetter", 1000, NULL, 1, NULL);
    
    printf("Iniciando scheduler...\r\n");
    vTaskStartScheduler();

    for(;;);
    return 0;
}

void settingTask(void *pvParamters){
    const TickType_t xDelay1sec = pdMS_TO_TICKS(1000UL), xDontBlock = 0;
    while(true){
        vTaskDelay(xDelay1sec);
        printf("Seteando bit 0...\r\n");
        xEventGroupSetBits(xEventGroup, measureBIT_0);
    }
}

void readIMUTask(void *pvParameters){
    EventBits_t xEventGroupValue;
    const EventBits_t xBitsToWaitFor = measureBIT_0;
    while(true){
        xEventGroupValue = xEventGroupWaitBits(xEventGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando lectura de IMU...\n");
        /*
            FUNCIONES DE LECTURA DE LA IMU
        */
    }
}