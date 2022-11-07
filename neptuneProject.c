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
#include "nrf24_driver.h"

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

#define ENCODER_HOLES   72
#define ENCODER_RADIUS  0.09

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

//cola direccion del viento
QueueHandle_t xWindDirQueue;

//Cola velocidad del viento
QueueHandle_t xWindSpeedQueue;

//Colas con datos procesados y de control para enviar
QueueHandle_t xAcelAvgPayload[3];
QueueHandle_t xGyroAvgPayload[3];
QueueHandle_t xMagnetoAvgPayload[3];
QueueSetHandle_t xWindDirAvgPayload;
QueueSetHandle_t xWindSpeedAvgPayload;

//Direcciones de los pipes RF
const uint8_t pipezero_addr[5] = {0x37,0x37,0x37,0x37,0x37};
const uint8_t pipeone_addr[5] = {0xC7,0xC7,0xC7,0xC7,0xC7};
const uint8_t pipetwo_addr[5] = {0xC8,0xC7,0xC7,0xC7,0xC7};

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

void addHole();


//Variables de la IMU
int16_t acceleration[3], gyro[3], gyroCal[3], eulerAngles[2], fullAngles[2], magnet[3];
absolute_time_t timeOfLastCheck;

//Variables para la dirección del viento
const uint16_t windDirPin = 28;
const uint16_t intHole = 16;

//Variables para la velocidad del viento
uint16_t holeCount = 0;

//Factor de conversion del adc
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

        xAcelAvgPayload[i] = xQueueCreate(1, sizeof(int16_t));
        xGyroAvgPayload[i] = xQueueCreate(1, sizeof(int16_t));
        xMagnetoAvgPayload[i] = xQueueCreate(1, sizeof(int16_t));
    }

    //Creación de colas
    xWindDirQueue = xQueueCreate(DATA_NUM_AVG, sizeof(uint16_t));
    xWindSpeedQueue = xQueueCreate(1, sizeof(float));

    xWindDirAvgPayload = xQueueCreate(1, sizeof(uint16_t));
    //xWindSpeedAvgPayload = xQueueCreate(1, sizeof(float));

    createTasks();

    //Iniciar el scheduler
    printf("Iniciando scheduler...\r\n");
    vTaskStartScheduler();

    for(;;);
    return 0;
}

void settingTask(void *pvParamters){
    const TickType_t xDelay1sec = pdMS_TO_TICKS(1150UL), xDontBlock = 0;
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

    //Variables
    float radSeg, windSpeed;

    EventBits_t xEventGroupValue;

    const TickType_t xDelay1sec = pdMS_TO_TICKS(1000UL), xDontBlock = 0;

    const EventBits_t xBitsToWaitFor = BIT_2;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xMeasureEventGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY);
        vTaskDelay(xDelay1sec);
        radSeg = 0;
        windSpeed = 0;
        printf("Iniciando lectura de velocidad del viento...\r\n");
        radSeg = ((holeCount * 60) / ENCODER_HOLES)*2*PI/60;
        windSpeed = radSeg * ENCODER_RADIUS;
        holeCount = 0;

        xQueueSendToBack(xWindSpeedQueue, &windSpeed, 0);

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
        
        //Extrayendo y acumulando datos
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
        
        //Se envian los valores procesados en la cola
        for (int i = 0; i < 3; i++)
        {
            xQueueSendToBack(xAcelAvgPayload[i], &AcelAvg[i],0);
            xQueueSendToBack(xGyroAvgPayload[i], &GyroAvg[i],0);
            xQueueSendToBack(xMagnetoAvgPayload[i], &MagnetoAvg[i],0);
        }
        
        
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
        xQueueSendToBack(xWindDirAvgPayload, &dirAvg, 0);
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

    //Inicializacion RF
    pin_manager_t pins_rf = { 
        .copi = 3,
        .cipo = 4, 
        .sck = 2,
        .csn = 5, 
        .ce = 6 
    };

    nrf_manager_t my_config = {
        // AW_3_BYTES, AW_4_BYTES, AW_5_BYTES
        .address_width = AW_5_BYTES,
        // dynamic payloads: DYNPD_ENABLE, DYNPD_DISABLE
        .dyn_payloads = DYNPD_ENABLE,
        // retransmission delay: ARD_250US, ARD_500US, ARD_750US, ARD_1000US
        .retr_delay = ARD_500US,
        // retransmission count: ARC_NONE...ARC_15RT
        .retr_count = ARC_10RT,
        // data rate: RF_DR_250KBPS, RF_DR_1MBPS, RF_DR_2MBPS
        .data_rate = RF_DR_1MBPS,
        // RF_PWR_NEG_18DBM, RF_PWR_NEG_12DBM, RF_PWR_NEG_6DBM, RF_PWR_0DBM
        .power = RF_PWR_NEG_12DBM,
        // RF Channel 
        .channel = 120,
    };

    // SPI baudrate
    uint32_t my_baudrate = 5000000;

    nrf_client_t my_nrf;

    // initialise my_nrf
    nrf_driver_create_client(&my_nrf);

    // configure GPIO pins and SPI
    my_nrf.configure(&pins_rf, my_baudrate);

    // not using default configuration (my_nrf.initialise(NULL)) 
    my_nrf.initialise(&my_config);

    //set to Standby-I Mode
    my_nrf.standby_mode();

    //Estructuras de payloads para los pipes

    // payload sent to receiver data pipe 0
    //Estructura de datos para enviar los datos de la IMU
    typedef struct payload_zero_s {
        uint8_t tagAcel;
        int16_t acelX;
        int16_t acelY;
        int16_t acelZ;
        uint8_t tagGyro;
        int16_t gyroX;
        int16_t gyroY;
        int16_t gyroZ;
        uint8_t tagMag;
        int16_t magX;
        int16_t magY;
        int16_t magZ;
    } payload_zero_t;

    // payload sent to receiver data pipe 1
    typedef struct payload_one_s {
        int16_t windDir;
        float windSpeed; 
    } payload_one_t;


    // result of packet transmission
    fn_status_t success;

    uint64_t time_sent = 0; // time packet was sent
    uint64_t time_reply = 0; // response time after packet sent

    //Valor del grupo de eventos
    EventBits_t xEventGroupValue;

    //Valores a enviar
    int16_t Acel2Send[3], Gyro2Send[3], Mag2Send[3], buffer, WindDir2Send;
    float WindSpeed2Send;

    //Bits del grupo de eventos por lo que se va a esperar
    const EventBits_t xBitsToWaitfor = BIT_4;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xControlEventGroup, xBitsToWaitfor, pdTRUE, pdTRUE, portMAX_DELAY);
        
        //Extrayendo valores del acelerometro
        //printf("---- < EXTRAYENDO VALORES DEL ACELEROMETRO > ----\r\n");
        for(int i = 0; i <3; i++){
            xQueueReceive(xAcelAvgPayload[i], &buffer, 0);
            Acel2Send[i] = buffer;
        }

        //Extrayendo valores del giroscopio
        //printf("---- < EXTRAYENDO VALORES DEL GIROSCOPIO > ----\r\n");
        for(int i = 0; i <3; i++){
            xQueueReceive(xGyroAvgPayload[i], &buffer, 0);
            Gyro2Send[i] = buffer;
        }

        //Extrayendo valores del magnetometro
        //printf("---- < EXTRAYENDO VALORES DEL MAGNETOMETRO > ----\r\n");
        for(int i = 0; i <3; i++){
            xQueueReceive(xMagnetoAvgPayload[i], &buffer, 0);
            Mag2Send[i] = buffer;
        }

        
        //Extrayendo valores del sensor de viento
        //printf("---- < EXTRAYENDO VALORES DEL VIENTO > ----\r\n");
        xQueueReceive(xWindDirAvgPayload, &WindDir2Send,0);
        xQueueReceive(xWindSpeedQueue, &WindSpeed2Send,0);

        //Creando payload zero
        //printf("---- < CREANDO PAYLOAD 0 > ----\r\n");
        payload_zero_t payload_zero = {
            .acelX = Acel2Send[0],
            .acelY = Acel2Send[1],
            .acelZ = Acel2Send[2],
            .gyroX = Gyro2Send[0] - gyroCal[0],
            .gyroY = Gyro2Send[1] - gyroCal[1],
            .gyroZ = Gyro2Send[2] - gyroCal[2],
            .magX = Mag2Send[0],
            .magY = Mag2Send[1],
            .magZ = Mag2Send[2]
        };

        //Creando payload uno
        //printf("---- < CREANDO PAYLOAD 1 > ----\r\n");
        payload_one_t payload_one = {
            .windDir = WindDir2Send,
            .windSpeed = WindSpeed2Send 
        };

        printf("---- < ENVIANDO PAYLOAD > ----\r\n");

        //Enviando datos al pipe 0
        //send to receiver's DATA_PIPE_0 address
        my_nrf.tx_destination(pipezero_addr);

        // time packet was sent
        time_sent = to_us_since_boot(get_absolute_time()); // time sent

        // send packet to receiver's DATA_PIPE_0 address
        success = my_nrf.send_packet(&payload_zero, sizeof(payload_zero));

        // time auto-acknowledge was received
        time_reply = to_us_since_boot(get_absolute_time()); // response time

        if (success){
            printf("\nPacket sent:- Response: %lluμS | Payload: %d,%d,%d,%d,%d,%d,%d,%d,%d\n",time_reply - time_sent, 
            payload_zero.acelX, payload_zero.acelY, payload_zero.acelZ, payload_zero.gyroX, payload_zero.gyroY, payload_zero.gyroZ,
            payload_zero.magX, payload_zero.magY, payload_zero.magZ);
        } else {
            printf("\nPacket not sent:- Receiver not available.\n");
        }

        // send to receiver's DATA_PIPE_1 address
        my_nrf.tx_destination(pipeone_addr);

        // time packet was sent
        time_sent = to_us_since_boot(get_absolute_time()); // time sent

        // send packet to receiver's DATA_PIPE_1 address
        success = my_nrf.send_packet(&payload_one, sizeof(payload_one));
        
        // time auto-acknowledge was received
        time_reply = to_us_since_boot(get_absolute_time()); // response time

        if (success){
            printf("\nPacket sent:- Response: %lluμS | Payload: %d,%d\n", time_reply - time_sent, payload_one.windSpeed, payload_one.windDir);
        } else {
            printf("\nPacket not sent:- Receiver not available.\n");
        }

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
    gpio_set_irq_enabled_with_callback(intHole, GPIO_IRQ_EDGE_RISE, true, &addHole);

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
    xTaskCreate(sendPayloadTask, "Send", 1000, NULL, 3, NULL);
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

void addHole(){
    holeCount += 1;
}

