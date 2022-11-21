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
#include "hardware/pwm.h"

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


//Colas con datos procesados y de control para enviar
QueueHandle_t xAcelAvgPayload[3];
QueueHandle_t xGyroAvgPayload[3];
QueueHandle_t xMagnetoAvgPayload[3];
QueueSetHandle_t xWindDirAvgPayload;
QueueSetHandle_t xWindSpeedAvgPayload;

//Variable para controlar el envío de la telemtría
bool sendTele = false;

//Variables para RF
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

//Variables del Servo
const float clockDiv = 64;
float wrap = 39062;
const int degree = 0;

//Servomotores
const int servoVela = 7;
const int servoTimon1 = 8;
const int servoTimon2 = 9;

//Servomotor 2
const int servoPin2 = 1;
const int rightPin = 12;
const int leftPin = 15;
float grados2 = 90;
bool rightButton = false;
bool leftButton = false;

//Funciones para los servos
void InitServo(int servoPin, float startDegree);
void setDegree(int servoPin, float degree);

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

    createTasks();

    //Iniciar el scheduler
    printf("Iniciando scheduler...\r\n");
    
    //Control de los servos
    setDegree(servoVela, 90);
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

            //printf("LEIDO: %d,%d,%d\r\n", xAcelData[0], xAcelData[1], xAcelData[2]);
            //printf("LEIDO: %d,%d,%d\r\n", xGyroData[0], xGyroData[1], xGyroData[2]);
            //printf("LEIDO: %d,%d,%d\r\n", xMagnetoData[0], xMagnetoData[1], xMagnetoData[2]);
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
    int16_t buffer, AcelAvg[3], GyroAvg[3], MagnetoAvg[3];

    BaseType_t xStatus;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xProcEventGroup, xBitsToWaitfor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando procesamiento de la IMU...\r\n");

        //Inicializacion variables que almacenan los datos promediados
        for (int i = 0; i < 3; i++)
        {
            AcelAvg[i] = 0;
            buffer = 0;
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
                xQueueReceive(xAcelQueue[n], &buffer, 0);
                accAcel[n] += buffer;
            }

            //Se extraen los datos de cada eje del giroscopio y se acumulan
            for(int n = 0; n < 3; n++){
                xQueueReceive(xGyroQueue[n], &buffer, 0);
                accGyro[n] += buffer;
            }


            //Se extraen los datos de cada eje del giroscopio y se acumulan
            for(int n = 0; n < 3; n++){
                xQueueReceive(xMagnetoQueue[n], &buffer, 0);
                accMagneto[n] += buffer;
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
        sendTele = true;
        xEventGroupSetBits(xControlEventGroup, BIT_4);
    }
}

void sendPayloadTask(void *pvParameters){

    //Definición estruturas de Payloads

    // payload sent to receiver data pipe 0
    //uint8_t payload2Send[5] = "Hello";

    typedef struct payload2Send_s{
        bool request;
        int16_t Acel[3];
        int16_t Gyro[3];
        int16_t Magneto[3];
        int16_t WindDir;
        float WindSpeed;
    } payload2Send_t;

    //payload2Send_t payload2Send;

    //Payload to receive
    typedef struct payload2Receive_s{
        uint16_t velaDegree;
        bool right;
        bool left; 
    } payload2Receive_t;

    payload2Receive_t payload2Receive;

    // result of packet transmission
    fn_status_t success = 0;

    //Valor del grupo de eventos
    EventBits_t xEventGroupValue;

    uint64_t time_sent = 0; // time packet was sent
    uint64_t time_reply = 0; // response time after packet sent

    //Bits del grupo de eventos por lo que se va a esperar
    //const EventBits_t xBitsToWaitfor = BIT_4;

    const TickType_t xDelay = pdMS_TO_TICKS(25UL), xDontBlock = 0;
    const TickType_t xDelayTimeOut = pdMS_TO_TICKS(1UL);

    int timeOutCounter = 0;

    // data pipe number a packet was received on
    uint8_t pipe_numberRX = 0;

    //Valores a enviar
    int16_t Acel2Send[3], Gyro2Send[3], Mag2Send[3], buffer, WindDir2Send;
    float WindSpeed2Send;

    while(true){
        //xEventGroupValue = xEventGroupWaitBits(xControlEventGroup, xBitsToWaitfor, pdTRUE, pdTRUE, portMAX_DELAY);
        //printf("---- < ENVIANDO PAYLOAD > ----\r\n");

        timeOutCounter = 0;

        //Comprobar si hay telemetría para enviar
        if(sendTele){
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
            
        }
        //Montando el payload
        payload2Send_t payload2Send = {
            .request = sendTele,
            .Acel = {Acel2Send[0], Acel2Send[1], Acel2Send[2]},
            .Gyro = {Gyro2Send[0], Gyro2Send[1], Gyro2Send[2]},
            .Magneto = {Mag2Send[0], Mag2Send[1], Mag2Send[2]},
            .WindDir = WindDir2Send,
            .WindSpeed = WindSpeed2Send
        };
        sendTele = 0;
        // send to receiver's DATA_PIPE_0 address
        my_nrf.tx_destination((uint8_t[]){0x37,0x37,0x37,0x37,0x37});


        //Armar payload

        // time packet was sent
        time_sent = to_us_since_boot(get_absolute_time()); // time sent

        // send packet to receiver's DATA_PIPE_2 address
        success = my_nrf.send_packet(&payload2Send, sizeof(payload2Send));
        
        // time auto-acknowledge was received
        time_reply = to_us_since_boot(get_absolute_time()); // response time

        if (success)
        {   
            //sendTele = 0;
            printf("\nPacket sent:- Response: %lluμS | Payload: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %f\n",time_reply - time_sent, payload2Send.request,
                payload2Send.Acel[0], payload2Send.Acel[1], payload2Send.Acel[2], payload2Send.Gyro[0], payload2Send.Gyro[1], payload2Send.Gyro[2],
                payload2Send.Magneto[0], payload2Send.Magneto[1], payload2Send.Magneto[2], payload2Send.WindDir, payload2Send.WindSpeed);
            
            my_nrf.receiver_mode();
            printf("Preparado para recibir control \r\n");

            while(timeOutCounter < 50){
                if(my_nrf.is_packet(&pipe_numberRX))
                {
                    switch (pipe_numberRX)
                    {
                        case DATA_PIPE_0:
                            // read payload
                            my_nrf.read_packet(&payload2Receive, sizeof(payload2Receive));

                            // receiving a two byte struct payload on DATA_PIPE_2
                            printf("\nPacket received:- Payload (%d, %d, %d) on data pipe (%d)\n", payload2Receive.velaDegree, payload2Receive.right, 
                                payload2Receive.left, pipe_numberRX);

                            if(payload2Receive.right & (grados2<=175)){
                                grados2 += 10;
                            }
                            if(payload2Receive.left & (grados2>=5)){
                                grados2 -= 10;
                            }
                            /*
                            if(grados2>=95){
                                grados2 -= 5;
                            }
                            if(grados2<=85){
                                grados2 += 5;
                            }
                            */
                        break;
                        
                        default:
                        break;
                    }
                }
                
                else{
                    timeOutCounter += 1;
                    vTaskDelay(xDelayTimeOut);
                }
            }

            if(timeOutCounter > 100){
                printf("COM TIMEOUT\r\n");
                timeOutCounter = 0;
            }

            //Control de los servos
            setDegree(servoVela, payload2Receive.velaDegree);
            setDegree(servoTimon1, grados2);
            setDegree(servoTimon2, grados2);
        }
        /*
        else {
            printf("\nPacket not sent:- Receiver not available.\n");
        }
        */
        vTaskDelay(xDelay);
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


    //Inicialización de servos
    InitServo(servoVela, 0);
    InitServo(8, 0);
    InitServo(9, 0);

    //Init IMU
    init_mpu9250(100);

    //Inicialización RF
    nrf_driver_create_client(&my_nrf);

    // configure GPIO pins and SPI
    my_nrf.configure(&pins_rf, my_baudrate);

    // not using default configuration (my_nrf.initialise(NULL)) 
    my_nrf.initialise(&my_config);

    /**
     * set addresses for DATA_PIPE_0 - DATA_PIPE_3.
     * These are addresses the transmitter will send its packets to.
     */
    my_nrf.rx_destination(DATA_PIPE_0, (uint8_t[]){0x37,0x37,0x37,0x37,0x37});
    my_nrf.rx_destination(DATA_PIPE_1, (uint8_t[]){0xC7,0xC7,0xC7,0xC7,0xC7});
    my_nrf.rx_destination(DATA_PIPE_2, (uint8_t[]){0xC8,0xC7,0xC7,0xC7,0xC7});
    my_nrf.rx_destination(DATA_PIPE_3, (uint8_t[]){0xC9,0xC7,0xC7,0xC7,0xC7});


    //set to Standby-I Mode
    my_nrf.standby_mode();
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
    xTaskCreate(sendPayloadTask, "Send", 1000, NULL, 4, NULL);
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

//FUNCIONES DEL SERVO
void setDegree(int servoPin, float degree)
{
    float millis;
    millis = ((100/9)*degree) + 400;
    pwm_set_gpio_level(servoPin, (millis/20000.f)*wrap);
}

void InitServo(int servoPin, float startDegree)
{
    gpio_set_function(servoPin, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(servoPin);

    pwm_config config = pwm_get_default_config();
    
    uint64_t clockspeed = clock_get_hz(5); //Get the current frequency of the specified clock

    wrap = clockspeed/clockDiv/50;

    pwm_config_set_clkdiv(&config, clockDiv);
    pwm_config_set_wrap(&config, wrap);
    pwm_init(slice_num, &config, true);

    float startMillis;
    startMillis = ((100/9)*startDegree) + 400;
    setDegree(servoPin, startMillis);
}

