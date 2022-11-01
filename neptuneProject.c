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

//Grupo de eventos
#define BIT_0 (1UL << 0UL)
#define BIT_1 (1UL << 1UL)
#define BIT_2 (1UL << 2UL)
#define BIT_3 (1UL << 3UL)
#define BIT_4 (1UL << 4UL)

#define CONTROLTRIGGER (1UL << 3UL)|(1 << 2UL)|(1 << 1UL)|(1 << 0UL)

//Event group de medida
EventGroupHandle_t xMeasureEventGroup;
EventGroupHandle_t xProcEventGroup;
EventGroupHandle_t xControlEventGroup;

//Tareas
void readIMUTask(void *pvParameters);
void readWindDirTask(void *pvParameters);
void readWindSpeedTask(void *pvParameters);
void readWiFi(void *pvParameters);

void procesIMUTask(void *pvParameters);
void procesWindDirTask(void *pvParameters);
void procesWiFiTask(void *pvParameters);

void controlActionTask(void *pvParameters);
void sendPayloadTask(void *pvParameters);

void settingTask(void *pvParameters);

int main()
{
    stdio_init_all();

    sleep_ms(5000);
    printf("Iniciando...\r\n");

    //Creación del grupo de eventos
    printf("Creando grupos de eventos...\r\n");
    xMeasureEventGroup = xEventGroupCreate();
    xProcEventGroup = xEventGroupCreate();
    xControlEventGroup = xEventGroupCreate();

    //Creación de tareas
    printf("Creando tareas...\r\n");
    //Tarea trigger (Cada segundo)
    xTaskCreate(settingTask, "BitSetter", 1000, NULL, 1, NULL);
    //Tareas de medición
    xTaskCreate(readIMUTask, "IMUReader", 1000, NULL, 2, NULL);
    xTaskCreate(readWindDirTask, "WindDirReader", 1000, NULL, 2, NULL);
    xTaskCreate(readWindSpeedTask, "WindSpeedReader", 1000, NULL, 2, NULL);
    xTaskCreate(readWiFi, "WiFiReader", 1000, NULL, 2, NULL);

    //Tareas de procesamiento
    xTaskCreate(procesIMUTask, "IMUProces", 1000, NULL, 2, NULL);
    xTaskCreate(procesWindDirTask, "WindProces", 1000, NULL, 2, NULL);
    xTaskCreate(procesWiFiTask, "WiFiProces", 1000, NULL, 2, NULL);
    
    //Tarea de control
    xTaskCreate(controlActionTask, "ControlAction", 1000, NULL, 3, NULL);

    //Tarea de comunicacion
    xTaskCreate(sendPayloadTask, "Send", 1000, NULL, 2, NULL);

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
        xEventGroupSetBits(xMeasureEventGroup, BIT_0);
        //printf("Seteando bit 1...\r\n");
        xEventGroupSetBits(xMeasureEventGroup, BIT_1);
        //printf("Seteando bit 2...\r\n");
        xEventGroupSetBits(xMeasureEventGroup, BIT_2);
        //printf("Seteando bit 3...\r\n");
        xEventGroupSetBits(xMeasureEventGroup, BIT_3);
    }
}

void readIMUTask(void *pvParameters){
    //Valor del grupo de eventos
    EventBits_t xEventGroupValue;

    //Bits del grupo de eventos por los que se va a esperar
    const EventBits_t xBitsToWaitFor = BIT_0;
    while(true){
        xEventGroupValue = xEventGroupWaitBits(xMeasureEventGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando lectura de IMU...\r\n");

        /*
            FUNCIONES DE LECTURA DE LA IMU
        */

       xEventGroupSetBits(xProcEventGroup, BIT_0);

    }
}

void readWindDirTask(void *pvParameters){
    EventBits_t xEventGroupValue;

    const EventBits_t xBitsToWaitFor = BIT_1;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xMeasureEventGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando lectura de direccion del viento...\r\n");
        /*
            FUNCIONES DE LECTURA DE LA DIRECCION DEL VIENTO
        */

       xEventGroupSetBits(xProcEventGroup, BIT_1);
    }
}

void readWindSpeedTask(void *pvParameters){
    EventBits_t xEventGroupValue;

    const EventBits_t xBitsToWaitFor = BIT_2;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xMeasureEventGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando lectura de velocidad del viento...\r\n");

        /*
            FUNCIONES DE LECTURA DE LA VELOCIDAD DEL VIENTO
        */
       xEventGroupSetBits(xControlEventGroup, BIT_2);
    }
}

void readWiFi(void *pvParameters){
    EventBits_t xEventGroupValue;

    const EventBits_t xBitsToWaitFor = BIT_3;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xMeasureEventGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando lectura de módulos WiFi...\r\n");

        /*
            FUNCIONES DE LECTURA DEL MODULO WIFI
        */
       xEventGroupSetBits(xProcEventGroup, BIT_2);
    }
}

void procesIMUTask(void *pvParameters){
    //Valor del grupo de eventos
    EventBits_t xEventGroupValue;

    //Bits del grupod e eventos por lo que se va a esperar
    const EventBits_t xBitsToWaitfor = BIT_0;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xProcEventGroup, xBitsToWaitfor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando procesamiento de la IMU...\r\n");

        /*
            FUNCIONES PARA EL PROCESMAIENTO DE LA IMU
        */

       xEventGroupSetBits(xControlEventGroup, BIT_0);
    }
}

void procesWindDirTask(void *pvParameters){
    //Valor del grupo de eventos
    EventBits_t xEventGroupValue;

    //Bits del grupod e eventos por lo que se va a esperar
    const EventBits_t xBitsToWaitfor = BIT_1;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xProcEventGroup, xBitsToWaitfor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando procesamiento de la dirección del viento...\r\n");

        /*
            FUNCIONES PARA EL PROCESMAIENTO DE LA DIRECCION DEL VIENTO
        */

       xEventGroupSetBits(xControlEventGroup, BIT_1);
    }
}

void procesWiFiTask(void *pvParameters){
    //Valor del grupo de eventos
    EventBits_t xEventGroupValue;

    //Bits del grupo de eventos por lo que se va a esperar
    const EventBits_t xBitsToWaitfor = BIT_2;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xProcEventGroup, xBitsToWaitfor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando procesamiento del WiFi...\r\n");

        /*
            FUNCIONES PARA EL PROCESMAIENTO DE LA DIRECCION DEL VIENTO
        */

       xEventGroupSetBits(xControlEventGroup, BIT_3);
    }
}

void controlActionTask(void *pvParameters){
    //Valor del grupo de eventos
    EventBits_t xEventGroupValue;

    //Bits del grupo de eventos por lo que se va a esperar
    const EventBits_t xBitsToWaitfor = (BIT_0 | BIT_1 | BIT_2 | BIT_3);

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xControlEventGroup, xBitsToWaitfor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("---- < INICIANDO CONTROL > ----\r\n");

        /*
            FUNCIONES PARA EL PROCESMAIENTO DE LA DIRECCION DEL VIENTO
        */
        xEventGroupSetBits(xControlEventGroup, BIT_4);
    }
}

void sendPayloadTask(void *pvParameters){
    //Valor del grupo de eventos
    EventBits_t xEventGroupValue;

    //Bits del grupo de eventos por lo que se va a esperar
    const EventBits_t xBitsToWaitfor = BIT_4;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xControlEventGroup, xBitsToWaitfor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("---- < ENVIANDO PAYLOAD > ----\r\n");

        /*
            FUNCIONES PARA EL PROCESMAIENTO DE LA DIRECCION DEL VIENTO
        */
    }
}
