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
void readWindDirTask(void *pvParameters);
void readWindSpeedTask(void *pvParameters);
void readWiFi(void *pvParameters);

void settingTask(void *pvParameters);

int main()
{
    stdio_init_all();

    sleep_ms(5000);
    printf("Iniciando...\r\n");

    //Creación del grupo de eventos
    printf("Creando grupos de eventos...\r\n");
    xEventGroup = xEventGroupCreate();

    //Creación de tareas
    printf("Creando tareas...\r\n");
    xTaskCreate(settingTask, "BitSetter", 1000, NULL, 1, NULL);
    xTaskCreate(readIMUTask, "IMUReader", 1000, NULL, 2, NULL);
    xTaskCreate(readWindDirTask, "WindDirReader", 1000, NULL, 2, NULL);
    xTaskCreate(readWindSpeedTask, "WindSpeedReader", 1000, NULL, 2, NULL);
    xTaskCreate(readWiFi, "WiFiReader", 1000, NULL, 2, NULL);
    
    //Iniciar el scheduler
    printf("Iniciando scheduler...\r\n");
    vTaskStartScheduler();

    for(;;);
    return 0;
}

void settingTask(void *pvParamters){
    const TickType_t xDelay1sec = pdMS_TO_TICKS(3000UL), xDontBlock = 0;
    while(true){
        vTaskDelay(xDelay1sec);
        //printf("Seteando bit 0...\r\n");
        printf("---- < LANZANDO ETAPA DE MEDIDA > ----\r\n");
        xEventGroupSetBits(xEventGroup, measureBIT_0);
        //printf("Seteando bit 1...\r\n");
        xEventGroupSetBits(xEventGroup, measureBIT_1);
        //printf("Seteando bit 2...\r\n");
        xEventGroupSetBits(xEventGroup, measureBIT_2);
        //printf("Seteando bit 3...\r\n");
        xEventGroupSetBits(xEventGroup, measureBIT_3);
    }
}

void readIMUTask(void *pvParameters){
    //Valor del grupo de eventos
    EventBits_t xEventGroupValue;

    //Bits del grupo de eventos por los que se va a esperar
    const EventBits_t xBitsToWaitFor = measureBIT_0;
    while(true){
        xEventGroupValue = xEventGroupWaitBits(xEventGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando lectura de IMU...\r\n");
        /*
            FUNCIONES DE LECTURA DE LA IMU
        */
    }
}

void readWindDirTask(void *pvParameters){
    EventBits_t xEventGroupValue;

    const EventBits_t xBitsToWaitFor = measureBIT_1;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xEventGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando lectura de direccion del viento...\r\n");
        /*
            FUNCIONES DE LECTURA DE LA DIRECCION DEL VIENTO
        */
    }
}

void readWindSpeedTask(void *pvParameters){
    EventBits_t xEventGroupValue;

    const EventBits_t xBitsToWaitFor = measureBIT_2;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xEventGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando lectura de velocidad del viento...\r\n");

        /*
            FUNCIONES DE LECTURA DE LA VELOCIDAD DEL VIENTO
        */
    }
}

void readWiFi(void *pvParameters){
    EventBits_t xEventGroupValue;

    const EventBits_t xBitsToWaitFor = measureBIT_3;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xEventGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando lectura de módulos WiFi...\r\n");

        /*
            FUNCIONES DE LECTURA DEL MODULO WIFI
        */
    }
}

