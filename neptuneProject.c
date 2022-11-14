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

//Librerias comunicación y procesamiento WIFI
#include <string.h>
#include <cstdlib>
#include <math.h>
#include "hardware/uart.h"
#include "pico/binary_info.h"
#include "hardware/irq.h"

//Constantes para la comunicación y procesamiento WIFI
#define UART_ID uart1
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE
#define UART_TX_PIN 8 // Conectar a RX del modulo wifi
#define UART_RX_PIN 9 // Conectar a TX
#define F 2.4 //GHz
#define OUTPUT_POWER 20
#define ANTENNA_GAIN 2 //dB
#define NSources 3 //numero de fuentes

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


//Variables Posicionamiento
uint8_t ch; //caracter
static int chars_rxed = 0;//Cantidad de caracteres recibidos
bool timerFlag=0;//Bandera de activación de timer
bool new_ch=0;//Bandera de nuevo caracter
bool powerflag=0;//Bandera de evento nueva potencia registrada
char A_WF[200]={0};//Buffer de texto para líneas recibidas
//MAC de antenas caracterizadas
char *Smac[]={"86:f3:eb:e0:7e:02","ee:fa:bc:1c:bd:95","86:f3:eb:e0:6c:dd"};
float power[3]={0};//Arreglo global de potencias requeridas
//Distancia entre antenas transmisoras
const float D1 = 5;
const float D2 = 5;
float x;
float y;
float P1[4]={0};
float P2[4]={0};
float P3[4]={0};
uint k1=0;
uint k2=0;
uint k3=0;

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

//Funciones Posicionamiento
void on_uart_rx();
bool result_dir(struct repeating_timer *t);
void ESPinit();
void charmanage();
float squared(float a);
float distancia(float potencia,int source);
float calcularDistanciaX(float r0, float r2);
float calcularDistanciaY(float r0, float r1);
void coordenadas();

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
    xWindSpeedQueue = xQueueCreate(1, sizeof(float));

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

        tight_loop_contents();//Revision de interrupciones
        if (new_ch==1){//interrupcion de recepción UART activada
            //printf("%c",ch);
            new_ch=0;
            charmanage();
        }

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

        tight_loop_contents();//Revision de interrupciones
        if(timerFlag==1){//Interrupcion de timer
            timerFlag=0;
            uart_puts(UART_ID,"AT+CWLAP\r\n");//CWLAP es un comando que escaneará el entorno buscando las redes disponibles
            coordenadas();//Actualización de coordenadas
        }

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
    gpio_set_irq_enabled_with_callback(intHole, GPIO_IRQ_EDGE_RISE, true, &addHole);

    //Init Posicionamiento
    ESPinit();
    //iniciaclización de timer
    struct repeating_timer timer1;
    add_repeating_timer_ms(1500,result_dir,NULL,&timer1);
    sleep_ms(2000);
    uart_puts(UART_ID,"AT+CWMODE=3\r\n");//Modo para ser un SoftAP y una estación al tiempo
    sleep_ms(2000);

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

void addHole(){
    holeCount += 1;
}

//Funciones Posicionamiento 
// RX ISR
void on_uart_rx() {
   new_ch=1;
   ch = uart_getc(UART_ID);
}
// timer ISR
bool result_dir(struct repeating_timer *t)
{
    timerFlag=1;
    return true;
}
//Init UART
void ESPinit(){
    // Set up our UART with a basic baud rate.
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID, false);
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);
    uart_puts(UART_ID, "\nHello, uart interrupts\n");
}

void charmanage(){
    //Formato de la trama de llegada
    //AT+CWLAP
    //+CWLAP:(<ecn>,<<ssid>,rssi>,<mac>,<canal>)\n... OK //Pueden llegar varias tramas antes de llegar OK
    //<rssi>: String con el nivel de la señal (potencia)
    //<mac>: Machine address code Para filtrar con mayor confianza las potencias que se requieren
    if(ch=='\n'){ //Cada trama finaliza con un salto de linea
        char rssi[6]={0};//String buffer para guardar informacion de potencia recibida en la cadena
        float rssi_f;//buffer flotante para transformar la potencia identificada a un flotante
        char mac[20]={0};//String buffer para la mac
        A_WF[chars_rxed+1]='\0';//Se cambia el caracter de espacio por un caracter de final
        if(A_WF[0]=='+' && A_WF[1]=='C' && A_WF[2]=='W' && A_WF[3]=='L' && A_WF[4]=='A' && A_WF[5]=='P'){
            uint8_t sepcnt=0;//Contador de comas 
            int j;//iterador para moverse sobre el string
            for(int r=6;r<chars_rxed;r++){
                if(A_WF[r]==','){
                    sepcnt++;
                    if(sepcnt==2){//<rssi>==,-nn.n
                        j=0;
                        while(A_WF[r+j+1]!=','){
                            rssi[j]=A_WF[r+j+1];
                            j++;
                        }
                        rssi[j+1]='\0';
                        rssi_f=std::atof(rssi);//Conversion de string a float
                        //printf("%.1f\n",rssi_f);
                        continue;
                    }
                    if(sepcnt==3){//estructura de <mac>==,"00:00:00:00:00:01"
                        j=0;
                        while(A_WF[r+j+2]!='"'){//se inicia en 2 debido al caracter de 'comillas'
                            mac[j]=A_WF[r+j+2];
                            j++;
                        }
                        mac[j+1]='\0';//caracter de fin de cadena
                        break;
                    }
                }
            }
            //sepcnt=0;//limpiar cuenta de comas (no es necesario si se crea localmente)
            //printf("%s\n",A_WF);
        }
        for (int k=0;k<NSources;k++){
            if(strstr(mac,Smac[k])!=NULL){
                if(k==0){
                    P1[k1]=rssi_f;
                    k1++;
                    if (k1>3)k1=0;
                }
                if(k==1){
                    P2[k2]=rssi_f;
                    k2++;
                    if (k2>3)k2=0;
                }
                if(k==2){
                    P3[k3]=rssi_f;
                    k3++;
                    if (k3>3)k3=0;
                }
            }
        }
        bzero(mac,20);//limpiar buffer strings
        bzero(rssi,5);
        bzero(A_WF,200);
        chars_rxed=0;
    }else{
        A_WF[chars_rxed]=ch;//Si no es un caracter de espacio agregarlo al buffer string
        chars_rxed++;
    }
}

float squared(float a){
    return (a*a);
}

float distancia(float potencia,int source){
    float gain=0.00343f;//por defecto
    float Epow=-0.115f;
    if(source==0){
        gain=0.0033f;
        Epow=-0.115f; 
    }else if(source==1){
        gain=0.0031f;
        Epow=-0.109f;
    }else if (source==2)
    {
        gain=0.0039f;
        Epow=-0.11f;
    }
    //printf("%.5f\n",potencia);
    return gain*exp(potencia*Epow);  //De la regresion exponencial realizada dist=G*e^(-K*x)  
}

float calcularDistanciaX(float r0, float r2){
    return ((squared(r0)-squared(r2)+squared(D2))/(2*D2));
}

float calcularDistanciaY(float r0, float r1){
    return ((squared(r0)-squared(r1)+squared(D1))/(2*D1));
}

void coordenadas(){
    power[0]=(P1[0]+P1[1]+P1[2]+P1[3])/4.0;
    power[1]=(P2[0]+P2[1]+P2[2]+P2[3])/4.0;
    power[2]=(P3[0]+P3[1]+P3[2]+P3[3])/4.0;
    printf("potencias P1:%f P2:%f P3:%f \n",power[0],power[1],power[2]);
    float r0=distancia(power[0],0);
    float r1=distancia(power[1],1);
    float r2=distancia(power[2],2);
    printf("distancia: %.5f , %.5f , %.5f\n",r0,r1,r2);
    float x=calcularDistanciaX(r0,r2);
    float y=calcularDistanciaY(r0,r1);
    printf("Coordenadas(X,Y): (%.2f ,%.2f)\n",x,y);
}