#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "mpu9250.h"
#include "hardware/spi.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "queue.h"
#include "mpu9250.h"

//Timers del FreeRTOS
#include "timers.h"


#define PI 3.14159265

//Bits para eventos
#define BIT_0 (1UL << 0UL)
#define BIT_1 (1UL << 1UL)
#define BIT_2 (1UL << 2UL)
#define BIT_3 (1UL << 3UL)
#define BIT_4 (1UL << 4UL)

//Event group de medida
EventGroupHandle_t xMeasureEventGroup;
EventGroupHandle_t xProcEventGroup;
EventGroupHandle_t xControlEventGroup;

//Colas IMU
QueueHandle_t xAcelQueue[3];

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

//Inicialización del hardware
void hardwareInit(void);

//Creación de tareas
void createTasks(void);


//Variables de la IMU
int16_t acceleration[3], gyro[3], gyroCal[3], eulerAngles[2], fullAngles[2], magnet[3];
absolute_time_t timeOfLastCheck;

//Funciones de la IMU
void init_mpu9250(int loop);
void updateAngles();
void printDataImu();


int main()
{
    hardwareInit();

    sleep_ms(5000);
    printf("Iniciando...\r\n");

    //Creación del grupo de eventos
    printf("Creando grupos de eventos...\r\n");
    xMeasureEventGroup = xEventGroupCreate();
    xProcEventGroup = xEventGroupCreate();
    xControlEventGroup = xEventGroupCreate();

    //Creación de colas Acelerometro
    //xAcelQueue = xQueueCreate(10, sizeof(int16_t));
    for (int i = 0; i < 3; i++)
    {
        xAcelQueue[i] = xQueueCreate(10, sizeof(int16_t));
    }

    createTasks();

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

    //Variables para la Queue
    uint16_t ValueToSend;
    BaseType_t xStatus[3];

    //Variables para la IMU
    int16_t xAcelData[3], xGyroData[3], xMagnetData[3];

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xMeasureEventGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando lectura de IMU...\r\n");

        /*
            FUNCIONES DE LECTURA DE LA IMU
        */
       
        //Se llena la cola con datos
        /*
        ValueToSend = 20;
        for (uint16_t i = 0; i < 10; i++){
            ValueToSend += 1;
            xStatus = xQueueSendToBack(xAcelQueue, &ValueToSend, 0);
            printf("Enviando: %u\r\n", ValueToSend);
        }
        */
        for (int i = 0; i < 50; i++){
            updateAngles(xAcelData);
            printf("Valores leidos acelerometro: %d,%d,%d\r\n", xAcelData[0], xAcelData[1], xAcelData[2]);
            sleep_ms(100);
        }
       
       updateAngles(xAcelData);

       
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

    uint16_t buffer = 0;
    uint16_t AcelReceived[10];

    BaseType_t xStatus;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xProcEventGroup, xBitsToWaitfor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando procesamiento de la IMU...\r\n");

        /*
            FUNCIONES PARA EL PROCESMAIENTO DE LA IMU
        */
        /*
        for (int i = 0; i < 10; i++)
        {
            xStatus = xQueueReceive(xAcelQueue, &buffer, 0);
            printf("Valor recibido: %u\r\n", buffer);
        }
        printf("FIN DE LA COLA\r\n");
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

void hardwareInit(void){
    stdio_init_all();

    //Uso del default LED
    const uint ONBOARD_LED = PICO_DEFAULT_LED_PIN;
    gpio_init(ONBOARD_LED);
    gpio_set_dir(ONBOARD_LED, GPIO_OUT);
    gpio_put(ONBOARD_LED, 1);

    //Init IMU
    init_mpu9250(100);
}

void createTasks(void){
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
}

void init_mpu9250(int loop){
    start_spi();
    calibrate_gyro(gyroCal, loop);
    mpu9250_read_raw_accel(acceleration);
    calculate_angles_from_accel(eulerAngles, acceleration);
    timeOfLastCheck = get_absolute_time();
}

void updateAngles(int16_t acelTemp[3]){
    mpu9250_read_raw_accel(acelTemp);
    timeOfLastCheck = get_absolute_time();
}
