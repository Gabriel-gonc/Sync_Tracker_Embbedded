#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "wifi_acess_point.h"
#include "socket_udp.h"
#include "project_types.h"

/***********************************************************
 * Defines
***********************************************************/
#define TAG "MAIN"
#define SOFTWARE_DATA_ANALISYS 1

#define MSG_SYNC "SYNC"
#define CMD_ACK "ACK"

/***********************************************************
 * Function Prototypes
***********************************************************/
static void IRAM_ATTR grid_itr_callback(void *arg);
static void IRAM_ATTR sensor_itr_callback(void *arg);
static void gpio_init(void);
void system_callback(system_event_t event);
void grid_freq_task(void *pvParameters);
void sensor_freq_task(void *pvParameters);
void measure_pulse_difference(void *pvParameters);

/***********************************************************
 * Variables
***********************************************************/
/** @brief Variáveis para armazenar a diferença de tempo entre os pulsos da rede. */
volatile uint32_t delta_time_freq_grid = 0;

/** @brief Variáveis para armazenar a diferença de tempo entre os pulsos do sensor. */
volatile uint32_t delta_time_freq_sensor = 0;

/** @brief Flag para controlar quando a amostragem da rede é feita. */
volatile bool measurement_ready_freq_grid = false;

/** @brief Flag para controlar quando a amostragem do sensor é feita. */
volatile bool measurement_ready_freq_sensor = false;

volatile bool measurement_ready_diff_pulse = false;

/** @brief Variáveis para armazenar o instante de último pulso da rede. */
volatile uint32_t T_firstpulse_freq_grid = 0;

/** @brief Variáveis para armazenar o instante de último pulso do sensor. */
volatile uint32_t T_firstpulse_freq_sensor = 0;

/** @brief Variáveis para armazenar o valor das frequências cálculadas. */
double Freq_pulse_grid = 0;

/** @brief Variáveis para armazenar o valor das frequências cálculadas. */
double Freq_pulse_sensor = 0;

/** @brief Variáveis para fazer o cálculo da média de frequência da rede. */
volatile int cont_ciclos_freq_grid = 0;
uint32_t somatoria_tempos_freq_grid = 0;

/** @brief Variáveis para fazer o cálculo da média de frequência do sensor. */
volatile int cont_ciclos_freq_sensor = 0;
uint32_t somatoria_tempos_freq_sensor = 0;

volatile int cont_ciclos_diff_pulse = 0;

/** @brief Array to store grid signal periods. */
volatile uint32_t time_array_freq_grid[NUM_CYCLES_FREQ_GRID];

/** @brief Array to store sensor signal periods. */
volatile uint32_t time_array_freq_sensor[NUM_CYCLES_FREQ_SENSOR];

volatile uint32_t time_array_delta_pulse[NUM_CYCLES_DIFF_PULSE]; 

/** @brief Var to sign grid first measure cycle triggered. */
volatile bool triggered_process_freq_grid = false;

/** @brief Var to sign sensor first measure cycle triggered. */
volatile bool triggered_process_freq_sensor = false;

volatile bool sensor_pulse_ready = false;

uint8_t estado = 0;
enum {calibracao, operacao};

/** @brief Event Group variables. */
EventGroupHandle_t main_event_group;
const int WIFI_CONNECTED_BIT = BIT0;
const int DELTA_PULSE_MEASUREMENT_READY_BIT = BIT1;

/** @brief Buffers for udp socket. */
char udp_receive_buffer[UDP_RX_BUFFER_SIZE];
char udp_send_buffer[UDP_TX_BUFFER_SIZE];

SemaphoreHandle_t udp_semaphore = NULL;

volatile uint32_t ref_delta_pulse = 0;

/***********************************************************
 * Main Function
***********************************************************/
void app_main(void)
{
    /* Create event group */
    main_event_group = xEventGroupCreate();

    /* Create Semaphore */
    udp_semaphore = xSemaphoreCreateMutex();

    /* Init wifi*/
    wifi_init_softap();
    wifi_get_callback(system_callback);

    /* Create udp socket */
    xEventGroupWaitBits(main_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
    udp_socket_init();

    /* Establishing connection */
    while(1){
        if (udp_socket_receive(udp_receive_buffer, sizeof(udp_receive_buffer)) > 0)
        {
            if (strncmp(udp_receive_buffer, MSG_SYNC, strlen(MSG_SYNC)) == 0)
            {
                int len = snprintf(udp_send_buffer, sizeof(udp_send_buffer), CMD_ACK);
                udp_socket_send(udp_send_buffer, len);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                break;
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    /* Initializing gpio. */
    gpio_init();

    /* Task Create */
    xTaskCreate(measure_pulse_difference, "measure_pulse_difference", 4096, NULL, 0, NULL);
#ifndef SOFTWARE_DATA_ANALISYS
    xTaskCreate(grid_freq_task, "grid_freq_task", 2048, NULL, 0, NULL);
    xTaskCreate(sensor_freq_task, "sensor_freq_task", 2048, NULL, 0, NULL);
#endif

    /* Main loop. */
    while(true)
    {
        vTaskDelay(20000 / portTICK_PERIOD_MS);
    }
}

/***********************************************************
 * System Callback
***********************************************************/
void system_callback(system_event_t event)
{
    switch (event) {
        case SYSTEM_WIFI_CONNECTED:
            // ESP_LOGI(TAG, "WiFi connected");
            xEventGroupSetBits(main_event_group, WIFI_CONNECTED_BIT);
            break;
        case SYSTEM_WIFI_DISCONNECTED:
            // ESP_LOGI(TAG, "WiFi disconnected");
            break;
        default:
            break;
    }
}

/***********************************************************
 * Funções
***********************************************************/
static void gpio_init(void)
{   
    /* Config for Grid Signal */
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE; //interrupt of rising edge
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<ELETRIC_GRID_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;

    gpio_config(&io_conf); //configure GPIO with the given settings
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT); //install gpio isr service
    gpio_isr_handler_add(ELETRIC_GRID_PIN, grid_itr_callback, NULL); //hook isr handler for specific gpio pin

    /* Config for Sensor Signal */
    io_conf.intr_type = GPIO_INTR_NEGEDGE; //interrupt of rising edge
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<SENSOR_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;

    gpio_config(&io_conf); //configure GPIO with the given settings
    gpio_isr_handler_add(SENSOR_PIN, sensor_itr_callback, NULL); // hook isr handler for specific gpio pin
}

/***********************************************************
 * FreeRTOS Tasks
***********************************************************/
void grid_freq_task(void *pvParameters){

    while(true)
    {
        if (measurement_ready_freq_grid){

            /* Cálculo da frequência média. */
            somatoria_tempos_freq_grid = 0;
            for (int i = 0; i < NUM_CYCLES_FREQ_GRID; i++){
                somatoria_tempos_freq_grid += time_array_freq_grid[i];
            }
            double media_tempos = somatoria_tempos_freq_grid * 1.0 / NUM_CYCLES_FREQ_GRID;
            Freq_pulse_grid = 1.0 / (media_tempos / 1000000);

            /* Formação da string de dados. */
            xSemaphoreTake(udp_semaphore, portMAX_DELAY);
            memset(udp_send_buffer, 0, sizeof(udp_send_buffer));
            int len = snprintf(udp_send_buffer, sizeof(udp_send_buffer), "\nFrequência Rede: %.5f Hz", Freq_pulse_grid);

            /* Envio da frequência via UDP. */
            udp_socket_send(udp_send_buffer, len);
            xSemaphoreGive(udp_semaphore);

            /* Resetando variáveis para o próximo ciclo de medição. */
            measurement_ready_freq_grid = false;
            cont_ciclos_freq_grid = 0;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void sensor_freq_task(void *pvParameters){

    while(true)
    {
        if (measurement_ready_freq_sensor){

            /* Cálculo da frequência média. */
            somatoria_tempos_freq_sensor = 0;
            for (int i = 0; i < NUM_CYCLES_FREQ_SENSOR; i++){
                somatoria_tempos_freq_sensor += time_array_freq_sensor[i];
            }
            double media_tempos = somatoria_tempos_freq_sensor * 1.0 / NUM_CYCLES_FREQ_SENSOR;
            Freq_pulse_sensor = 1.0 / (media_tempos / 1000000);

            /* Formação da string de dados. */
            xSemaphoreTake(udp_semaphore, portMAX_DELAY);
            memset(udp_send_buffer, 0, sizeof(udp_send_buffer));
            int len = snprintf(udp_send_buffer, sizeof(udp_send_buffer), "\nFrequência Sens: %.5f Hz", Freq_pulse_sensor);

            /* Envio da frequência via UDP. */
            udp_socket_send(udp_send_buffer, len);
            xSemaphoreGive(udp_semaphore);

            /* Resetando variáveis para o próximo ciclo de medição. */
            measurement_ready_freq_sensor = false;
            cont_ciclos_freq_sensor = 0;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void measure_pulse_difference(void *pvParameters)
{

    while(true)
    {
        if (measurement_ready_diff_pulse){

            /* Cálculo da frequência média. */
            xSemaphoreTake(udp_semaphore, portMAX_DELAY);
            for (int i = 0; i < NUM_CYCLES_DIFF_PULSE; i++){
                
                /* Formação da string de dados. */
                memset(udp_send_buffer, 0, sizeof(udp_send_buffer));
#ifndef SOFTWARE_DATA_ANALISYS
                int len = snprintf(udp_send_buffer, sizeof(udp_send_buffer), "\nDelta_pulse[%d] = %ld",
                                         i, time_array_delta_pulse[i]);
#else           
                int len = snprintf(udp_send_buffer, sizeof(udp_send_buffer), "%ld", time_array_delta_pulse[i]);
#endif

                udp_socket_send(udp_send_buffer, len);
            }
            xSemaphoreGive(udp_semaphore);
            
            /* Resetando variáveis para o próximo ciclo de medição. */
            measurement_ready_diff_pulse = false;
            cont_ciclos_diff_pulse = 0;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/***********************************************************
 * Callbacks for GPIO Interrupts
***********************************************************/
static void IRAM_ATTR grid_itr_callback(void *arg)
{
    if (!triggered_process_freq_grid)
    {
        T_firstpulse_freq_grid = esp_timer_get_time();
        triggered_process_freq_grid = true;
    }
    else 
    {
        delta_time_freq_grid = esp_timer_get_time() - T_firstpulse_freq_grid;
        if (delta_time_freq_grid > GRID_WINDOW_FILTER) 
        {
            T_firstpulse_freq_grid = esp_timer_get_time();

            /* Permitir o cálculo da frequência a cada segundo sem interromper o processo. */
            if (!measurement_ready_freq_grid)
            {
                time_array_freq_grid[cont_ciclos_freq_grid] = delta_time_freq_grid;
                cont_ciclos_freq_grid++;
                if (cont_ciclos_freq_grid >= (NUM_CYCLES_FREQ_GRID)){
                    measurement_ready_freq_grid = true;
                }
            }

            /* Fazer o cálculo de sincronismo em cada ciclo. */
            if (sensor_pulse_ready)
            {
                //TODO: disparo do alarme ou proteção em caso de perda de sincronismo.
                
                if (!measurement_ready_diff_pulse)
                {
                    time_array_delta_pulse[cont_ciclos_diff_pulse] = T_firstpulse_freq_grid - ref_delta_pulse;
                    cont_ciclos_diff_pulse++;
                    if (cont_ciclos_diff_pulse >= (NUM_CYCLES_DIFF_PULSE)){
                        measurement_ready_diff_pulse = true;
                    }
                }
                sensor_pulse_ready = false;
            }
        }
    }
}

static void IRAM_ATTR sensor_itr_callback(void *arg)
{
    if (!triggered_process_freq_sensor)
    {
        T_firstpulse_freq_sensor = esp_timer_get_time();
        triggered_process_freq_sensor = true;
    }
    else 
    {
        delta_time_freq_sensor = esp_timer_get_time() - T_firstpulse_freq_sensor;
        if (delta_time_freq_sensor > SENSOR_WINDOW_FILTER)
        {
            T_firstpulse_freq_sensor = esp_timer_get_time();

            /* Permitir o cálculo da frequência a cada segundo sem interromper o processo. */
            if (!measurement_ready_freq_sensor)
            {
                time_array_freq_sensor[cont_ciclos_freq_sensor] = delta_time_freq_sensor;
                cont_ciclos_freq_sensor++;
                if (cont_ciclos_freq_sensor >= (NUM_CYCLES_FREQ_SENSOR)){
                    measurement_ready_freq_sensor = true;
                }
            }

            /* Fazer o cálculo de sincronismo em cada ciclo. */
            if (!sensor_pulse_ready){
                sensor_pulse_ready = true;
                ref_delta_pulse = T_firstpulse_freq_sensor;
            }
        }
    }
}