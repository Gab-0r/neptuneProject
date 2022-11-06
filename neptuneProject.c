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

//PI
#define PI 3.14159265

//Número de datos leidos a promediar
#define DATA_NUM_AVG    10

//Bits para eventos
#define BIT_0   (1UL << 0UL)
#define BIT_1   (1UL << 1UL)
#define BIT_2   (1UL << 2UL)
#define BIT_3   (1UL << 3UL)
#define BIT_4   (1UL << 4UL)

//Event group de medida
EventGroupHandle_t xMeasureEventGroup;
EventGroupHandle_t xProcEventGroup;
EventGroupHandle_t xControlEventGroup;

//Cola acelerometro
QueueHandle_t xAcelQueue[3];

//Cola giroscopio
QueueHandle_t xGyroQueue[3];

//Cola magnetometro
QueueHandle_t xMagnetoQueue[3];

//cola velocidad del viento
QueueHandle_t xWindDirQueue;

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

//Variables para la dirección del viento
const uint16_t windDirPin = 28;

//Factor de converion del adc
const float conversion_factor = 3.3f / (1 << 12);

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

    //Creación de colas de la IMU
    for (int i = 0; i < 3; i++)
    {
        xAcelQueue[i] = xQueueCreate(DATA_NUM_AVG, sizeof(int16_t));
        xGyroQueue[i] = xQueueCreate(DATA_NUM_AVG, sizeof(int16_t));
        xMagnetoQueue[i] = xQueueCreate(DATA_NUM_AVG, sizeof(int16_t));
    }

    //Creación de colas
    xWindDirQueue = xQueueCreate(DATA_NUM_AVG, sizeof(uint16_t));

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

    //Variables para la IMU
    int16_t xAcelData[3], xGyroData[3], xMagnetoData[3];

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xMeasureEventGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando lectura de IMU...\r\n");

        for (int i = 0; i < DATA_NUM_AVG; i++){
            updateAngles(xAcelData, xGyroData, xMagnetoData);

            for (int n = 0; n < 3; n++){
                xQueueSendToBack(xAcelQueue[n], &xAcelData[n], 0);
                xQueueSendToBack(xGyroQueue[n], &xGyroData[n], 0);
                xQueueSendToBack(xMagnetoQueue[n], &xMagnetoData[n], 0);
            }

            //printf("Enviado: %d,%d,%d\r\n", xAcelData[0], xAcelData[1], xAcelData[2]);
            //printf("Enviado: %d,%d,%d\r\n", xGyroData[0], xGyroData[1], xGyroData[2]);
            //printf("Enviado: %d,%d,%d\r\n", xMagnetoData[0], xMagnetoData[1], xMagnetoData[2]);
            //sleep_ms(100);
        }
       xEventGroupSetBits(xProcEventGroup, BIT_0);
    }
}

void readWindDirTask(void *pvParameters){
    
    uint16_t windDirAnalog;
    //float windDirGrades;

    EventBits_t xEventGroupValue;

    const EventBits_t xBitsToWaitFor = BIT_1;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xMeasureEventGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando lectura de direccion del viento...\r\n");

        //Leyendo y enviando por cola
        for(int i = 0; i < DATA_NUM_AVG; i++){
            windDirAnalog = adc_read();
            xQueueSendToBack(xWindDirQueue, &windDirAnalog, 0);
            //printf("Enviando: %d\r\n", windDirAnalog);
        }
        //windDirGrades = (windDirAnalog * conversion_factor) * (359.0/3.3);

       xEventGroupSetBits(xProcEventGroup, BIT_1);
    }
}

void readWindSpeedTask(void *pvParameters){

    EventBits_t xEventGroupValue;

    const EventBits_t xBitsToWaitFor = BIT_2;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xMeasureEventGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando lectura de velocidad del viento...\r\n");
        
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

    //Acumuladores
    int accAcel[3], accGyro[3], accMagneto[3];

    //Variables del acelerometro y recepcion de datos
    int16_t buffer[3], AcelAvg[3], GyroAvg[3], MagnetoAvg[3];

    BaseType_t xStatus;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xProcEventGroup, xBitsToWaitfor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando procesamiento de la IMU...\r\n");

        //Inicializacion variables que almacenan los datos promediados
        for (int i = 0; i < 3; i++)
        {
            AcelAvg[i] = 0;
            buffer[i] = 0;
            accAcel[i] = 0;
            accGyro[i] = 0;
            GyroAvg[i] = 0;
            accMagneto[i] = 0;
            MagnetoAvg[i] = 0;
        }
        
        //Extrayendo y acumulando datos del acelerometro
        for (int i = 0; i < DATA_NUM_AVG; i++){

            //Se extraen los datos de cada eje del acelerometro y se acumulan
            for(int n = 0; n < 3; n++){
                xQueueReceive(xAcelQueue[n], &buffer[n], 0);
                accAcel[n] += buffer[n];
            }

            //Se extraen los datos de cada eje del giroscopio y se acumulan
            for(int n = 0; n < 3; n++){
                xQueueReceive(xGyroQueue[n], &buffer[n], 0);
                accGyro[n] += buffer[n];
            }


            //Se extraeb kis datis de cada eje del giroscopio y se acumulan
            for(int n = 0; n < 3; n++){
                xQueueReceive(xMagnetoQueue[n], &buffer[n], 0);
                accMagneto[n] += buffer[n];
            }

            //printf("Recibido: %d,%d,%d\r\n", buffer[0], buffer[1], buffer[2]);
        }

        //Se dividen los valores acumulados para hallar el promedio
        for (int i = 0; i < 3; i++){
            AcelAvg[i] = accAcel[i]/DATA_NUM_AVG;
            GyroAvg[i] = accGyro[i]/DATA_NUM_AVG;
            MagnetoAvg[i] = accMagneto[i]/DATA_NUM_AVG;
        }
        //printf("Promedio acelerometro: %d,%d,%d\r\n", AcelAvg[0], AcelAvg[1], AcelAvg[2]);
        //printf("Promedio giroscopio: %d,%d,%d\r\n", GyroAvg[0], GyroAvg[1], GyroAvg[2]);
        //printf("Promedio magnetometro: %d,%d,%d\r\n", MagnetoAvg[0], MagnetoAvg[1], MagnetoAvg[2]);
        xEventGroupSetBits(xControlEventGroup, BIT_0);
    }
}

void procesWindDirTask(void *pvParameters){
    //Valor del grupo de eventos
    EventBits_t xEventGroupValue;

    //Variables
    float dirAvg = 0; 
    uint16_t accDir = 0;
    uint16_t buffer;

    //Bits del grupod e eventos por lo que se va a esperar
    const EventBits_t xBitsToWaitfor = BIT_1;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xProcEventGroup, xBitsToWaitfor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando procesamiento de la dirección del viento...\r\n");

        //Variables
        dirAvg = 0; 
        accDir = 0;
        buffer =0;

        for(int i = 0; i < DATA_NUM_AVG; i++){
            xQueueReceive(xWindDirQueue, &buffer, 0);
            accDir += buffer;
            //printf("Recibido: %d\r\n", buffer);
        }

        dirAvg = ((accDir/DATA_NUM_AVG)*conversion_factor) * (359.0/3.3);
        //printf("Dirección promedio: %f\r\n", dirAvg);
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

    //Init Wind
    adc_init();
    adc_gpio_init(windDirPin);
    adc_select_input(2);

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

void updateAngles(int16_t acelTemp[3], int16_t gyroTemp[3], int16_t magnetoTemp[3]){
    mpu9250_read_raw_accel(acelTemp);
    mpu9250_read_raw_gyro(gyroTemp);
    mpu9250_read_raw_magneto(magnetoTemp);
    timeOfLastCheck = get_absolute_time();
}

