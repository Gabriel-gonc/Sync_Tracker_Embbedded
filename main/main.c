#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"

#include "wifi_acess_point.h"
#include "socket_udp.h"
#include "project_types.h"
#include "gpio_control.h"

/***********************************************************
 * Defines
***********************************************************/
#define MAIN_TAG "MAIN"
#define QUEUE_MAIN_LENGHT 10
#define QUEUE_FAULT_LENGHT 2

/***********************************************************
 * Function Prototypes
***********************************************************/
static void system_callback(system_event_t event);
static system_state_t trait_messages(bool hand_shaking, bool state_comp, bool check_msg, bool rcv_delta_null_time);
static bool check_state_exit(char* msg_exit_criteria);
static void process_sensor_to_grid_diff_time (void);
void grid_freq_task(void *pvParameters);
void sensor_freq_task(void *pvParameters);
static void state_transition(void);

/***********************************************************
 * Variables
***********************************************************/
/** @brief Array to store the time difference between sensor and grid pulses */
static uint16_t time_difference[NUM_CYCLES_DIFF_PULSE] = {0};

/** @brief Array to store grid pulse periods. */
static uint16_t time_array_freq_grid[GRID_FREQUENCY_BUFFER_SIZE] = {0};

/** @brief Array to store sensor pulse periods. */
static uint16_t time_array_freq_sensor[SENSOR_FREQUENCY_BUFFER_SIZE] = {0};

/** @brief Event Group variables. */
EventGroupHandle_t main_event_group = NULL;
const int WIFI_CONNECTED_BIT = BIT0;

/** @brief Buffers for udp socket. */
char udp_receive_buffer[UDP_RX_BUFFER_SIZE] = {0};
char udp_send_buffer[UDP_TX_BUFFER_SIZE] = {0};

/** @brief Variable to store current System State */
static volatile system_state_t current_state = STATE_IDLE;

/** @brief Queue to take diff buffer already filled */
QueueHandle_t queue_time_difference_main = NULL;

/** @brief Queue to take grid pulse period */
QueueHandle_t queue_grid_period_main = NULL;

/** @brief Queue to take sensor pulse period */
QueueHandle_t queue_sensor_period_main = NULL;

/** @brief Queue to take Loss of Synchronism Fault */
QueueHandle_t queue_loss_of_synchronism = NULL;

/** @brief Semaphore for UDP socket */
SemaphoreHandle_t udp_semaphore = NULL;

/** @brief Task handle to freq_grid task */
TaskHandle_t task_handle_freq_grid = NULL;

/** @brief Task handle to freq_sensor task */
TaskHandle_t task_handle_freq_sensor = NULL;

/*-------------------------------------------------------
 * Main Function
**-------------------------------------------------------*/
void app_main(void)
{
    /* Create event group */
    main_event_group = xEventGroupCreate();

    /* Create time diff queue */
    queue_time_difference_main = xQueueCreate(QUEUE_MAIN_LENGHT, sizeof(time_difference));
    if (queue_time_difference_main == NULL) 
    {
        ESP_LOGE(MAIN_TAG, "Failed to create queue");
        esp_restart();
    }

    /* Create grid period queue */
    queue_grid_period_main = xQueueCreate(QUEUE_MAIN_LENGHT, sizeof(time_array_freq_grid));
    if (queue_grid_period_main == NULL) 
    {
        ESP_LOGE(MAIN_TAG, "Failed to create grid period queue");
        esp_restart();
    }

    /* Create sensor period queue */
    queue_sensor_period_main = xQueueCreate(QUEUE_MAIN_LENGHT, sizeof(time_array_freq_sensor));
    if (queue_sensor_period_main == NULL) 
    {
        ESP_LOGE(MAIN_TAG, "Failed to create sensor period queue");
        esp_restart();
    }

    /* Create loss of synchronism queue */
    queue_loss_of_synchronism = xQueueCreate(QUEUE_FAULT_LENGHT, sizeof(bool));
    if (queue_loss_of_synchronism == NULL) 
    {
        ESP_LOGE(MAIN_TAG, "Failed to create loss of synchronism queue");
        esp_restart();
    }

    /* Create udp semaphore */
    udp_semaphore = xSemaphoreCreateMutex();
    if (udp_semaphore == NULL) 
    {
        ESP_LOGE(MAIN_TAG, "Failed to create UDP semaphore");
        esp_restart();
    }

    /* GPIO Init */ 
    if (gpio_init(queue_loss_of_synchronism) != ESP_OK)
    {
        ESP_LOGE(MAIN_TAG, "Failed to initialize GPIO");
        esp_restart();
    }

    /* Init wifi*/
    wifi_init_softap();
    wifi_get_callback(system_callback);

    /* Create udp socket */
    xEventGroupWaitBits(main_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
    udp_socket_init();

    /* Establishing connection */
    current_state = trait_messages(true, false, false, false);
    if (current_state != STATE_CONNECTED)
    {
        ESP_LOGE(MAIN_TAG, "Failed to establish connection!");
        esp_restart();
    }

    /* Switch and settings for the next state */
    state_transition();

    /* Main Loop */
    while (1)
    {
        switch (current_state)
        {
            case STATE_TIME_DIFF_MONITORING:
            {
                /* Take and send sensor to grid pulse diff time */
                process_sensor_to_grid_diff_time(); 

                /* Check state exit criteria */
                if (check_state_exit(MSG_FNSH_TIME_DIFF_MON))
                {
                    /* Disable feature and interrupts */
                    set_diff_time_feature(false); 
                    gpio_disable_interrupts();
                    
                    /* Switch and settings for the next state */
                    state_transition();
                }
                break;
            }
            case STATE_FREQUENCY_MONITORING:
            {
                /* Check state exit criteria */
                if (check_state_exit(MSG_FNSH_FREQ))
                {
                    /* Disable feature and interrupts */
                    set_freq_monitoring_feature(false); 
                    gpio_disable_interrupts();

                    /* Task delete */
                    if (task_handle_freq_grid != NULL)
                    {
                        vTaskDelete(task_handle_freq_grid);
                        task_handle_freq_grid = NULL;
                    }
                    if (task_handle_freq_sensor != NULL)
                    {
                        vTaskDelete(task_handle_freq_sensor);
                        task_handle_freq_sensor = NULL;
                    }
                    
                    /* Switch and settings for the next state */
                    state_transition();
                }
                /* Delay to avoid trig watchdog time */
                vTaskDelay(1000 / portTICK_PERIOD_MS); //Avoid watchdog time
                break;
            }
            case STATE_OPERATIONAL:
            {
                bool fault = false;
                if (xQueueReceive(queue_loss_of_synchronism, &fault, (1000 / portTICK_PERIOD_MS)) == pdTRUE)
                {
                    if (fault)
                    {
                        ESP_LOGW(MAIN_TAG, "Loss of Synchronism Detected! Disabling Operational Mode.");
    
                        /* Send to desktop */
                        int len = snprintf(udp_send_buffer, sizeof(udp_send_buffer), "%s", CMD_FAULT);
                        if (xSemaphoreTake(udp_semaphore, portMAX_DELAY) == pdTRUE) 
                        {
                            udp_socket_send(udp_send_buffer, len);
                        }
                        xSemaphoreGive(udp_semaphore);
                        
                        /* Disable feature and interrupts */
                        set_operational_mode(false); 
                        gpio_disable_interrupts();
                        
                        /* Block */
                        while (true)
                        {
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                        }
                    }
                }
                /* Check state exit criteria */
                if (check_state_exit(MSG_FNSH_OP))
                {
                    /* Disable feature and interrupts */
                    set_operational_mode(false); 
                    gpio_disable_interrupts();

                    /* Delete operation_mode task */
                    delete_operation_mode_task();

                    /* Switch and settings for the next state */
                    state_transition();
                }
            }
            default:
                break;
        }
    }
}

/*-------------------------------------------------------
 * System Callback
**-------------------------------------------------------*/
/** @brief Function to enable inter-components communication */
static void system_callback(system_event_t event)
{
    switch (event) 
    {
        case SYSTEM_WIFI_CONNECTED:
            xEventGroupSetBits(main_event_group, WIFI_CONNECTED_BIT);
            break;
        case SYSTEM_WIFI_DISCONNECTED:
            esp_restart();
            break;
        default:
            break;
    }
}

/*-------------------------------------------------------
 * Functions
**-------------------------------------------------------*/
/** @brief Function to receive, trait and send udp control messages
 * 
 * @param hand_shaking True for hand-shaking process, false otherwise
 * @param state_comp True for state setting process, false otherwise
 */
static system_state_t trait_messages(bool hand_shaking, bool state_comp, bool check_msg, bool rcv_delta_null_time)
{
    while (1)
    {
        int bytes_received = -1;
        if (xSemaphoreTake(udp_semaphore, portMAX_DELAY) == pdTRUE) 
        {
            bytes_received = udp_socket_receive(udp_receive_buffer, sizeof(udp_receive_buffer));
        }
        xSemaphoreGive(udp_semaphore);
        
        if (bytes_received > 0)
        {
            if (hand_shaking && !state_comp && !check_msg && !rcv_delta_null_time)
            {
                if (strncmp(udp_receive_buffer, MSG_SYNC, strlen(MSG_SYNC)) == 0)
                {
                    int len = snprintf(udp_send_buffer, sizeof(udp_send_buffer), "%s", CMD_ACK);
                    if (xSemaphoreTake(udp_semaphore, portMAX_DELAY) == pdTRUE) 
                    {
                        udp_socket_send(udp_send_buffer, len);
                    }
                    xSemaphoreGive(udp_semaphore);
                    ESP_LOGI(MAIN_TAG, "Connected!");
                    return STATE_CONNECTED;  
                }
                else 
                {
                    int len = snprintf(udp_send_buffer, sizeof(udp_send_buffer), "%s", CMD_NACK);
                    if (xSemaphoreTake(udp_semaphore, portMAX_DELAY) == pdTRUE) 
                    {
                        udp_socket_send(udp_send_buffer, len);
                    }
                    xSemaphoreGive(udp_semaphore);
                }
            }

            else if (state_comp && !hand_shaking && !check_msg && !rcv_delta_null_time)
            {
                if (strncmp(udp_receive_buffer, MSG_TIME_DIFF_MON, strlen(MSG_TIME_DIFF_MON)) == 0)
                {
                    ESP_LOGI(MAIN_TAG, "Received time diff monitoring command");
                    int len = snprintf(udp_send_buffer, sizeof(udp_send_buffer), "%s", CMD_ACK);
                    if (xSemaphoreTake(udp_semaphore, portMAX_DELAY) == pdTRUE) 
                    {
                        udp_socket_send(udp_send_buffer, len);
                    }
                    xSemaphoreGive(udp_semaphore);
                    return STATE_TIME_DIFF_MONITORING;
                }
                else if (strncmp(udp_receive_buffer, MSG_OP, strlen(MSG_OP)) == 0)
                {
                    ESP_LOGI(MAIN_TAG, "Received operational command");
                    int len = snprintf(udp_send_buffer, sizeof(udp_send_buffer), "%s", CMD_ACK);
                    if (xSemaphoreTake(udp_semaphore, portMAX_DELAY) == pdTRUE) 
                    {
                        udp_socket_send(udp_send_buffer, len);
                    }
                    xSemaphoreGive(udp_semaphore);
                    return STATE_OPERATIONAL;
                }
                else if (strncmp(udp_receive_buffer, MSG_FREQ, strlen(MSG_FREQ)) == 0)
                {
                    ESP_LOGI(MAIN_TAG, "Received frequency command");
                    int len = snprintf(udp_send_buffer, sizeof(udp_send_buffer), "%s", CMD_ACK);
                    if (xSemaphoreTake(udp_semaphore, portMAX_DELAY) == pdTRUE) 
                    {
                        udp_socket_send(udp_send_buffer, len);
                    }
                    xSemaphoreGive(udp_semaphore);
                    return STATE_FREQUENCY_MONITORING; 
                }
                else 
                {
                    ESP_LOGW(MAIN_TAG, "Invalid command received: %s", udp_receive_buffer);
                    int len = snprintf(udp_send_buffer, sizeof(udp_send_buffer), "%s", CMD_NACK);
                    if (xSemaphoreTake(udp_semaphore, portMAX_DELAY) == pdTRUE) 
                    {
                        udp_socket_send(udp_send_buffer, len);
                    }
                    xSemaphoreGive(udp_semaphore);
                }
            }

            else if (check_msg && !hand_shaking && !state_comp && !rcv_delta_null_time)
            {
                return MSG_RECEIVED;
            }
            
            else if (rcv_delta_null_time && !check_msg && !hand_shaking && !state_comp)
            {
                return MSG_RECEIVED;
            }
        }
        
        else if (check_msg)
        {
            return NO_MSG_RECEIVED;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/** @brief Function to check if the state exit criteria has been received
 *  If the exit criteria has been received the current_state is updated.
 *  
 *  @param msg_exit_criteria Exit criteria received from vision
 * 
 */
static bool check_state_exit(char* msg_exit_criteria)
{
    /* Check if the exit criteria has been received */
    system_state_t msg_received = trait_messages(false, false, true, false);
    
    if (msg_received == MSG_RECEIVED)
    {
        if (strncmp(udp_receive_buffer, msg_exit_criteria, strlen(msg_exit_criteria)) == 0)
        {
            /* Send ACK CMD */
            int len = snprintf(udp_send_buffer, sizeof(udp_send_buffer), "%s", CMD_ACK);
            if (xSemaphoreTake(udp_semaphore, portMAX_DELAY) == pdTRUE) 
            {
                udp_socket_send(udp_send_buffer, len);
            }
            xSemaphoreGive(udp_semaphore);

            return true;
        }
        else 
        {
            int len = snprintf(udp_send_buffer, sizeof(udp_send_buffer), "%s", CMD_NACK);
            if (xSemaphoreTake(udp_semaphore, portMAX_DELAY) == pdTRUE) 
            {
                udp_socket_send(udp_send_buffer, len);
            }
            xSemaphoreGive(udp_semaphore);
            ESP_LOGE(MAIN_TAG, "Waiting Exit Criteria: %s", msg_exit_criteria);
            return false;
        }
    }
    return false;
}

static void state_transition(void)
{
    /* Receiving next state command */
    ESP_LOGI(MAIN_TAG, "Waiting next state CMD");
    current_state = trait_messages(false, true, false, false);
    switch (current_state)
    {
        case STATE_TIME_DIFF_MONITORING:
        {
            /* Clean diff time main queue */
            xQueueReset(queue_time_difference_main);

            /* Enable diff time monitoring feature and interrupts */
            set_diff_time_feature(true); 
            gpio_enable_interrupts();
            break;
        }
        case STATE_FREQUENCY_MONITORING:
        {
            /* Clean frequency monitoring main queues */
            xQueueReset(queue_grid_period_main);
            xQueueReset(queue_sensor_period_main);

            /* Enable frequency monitoring feature and interrupts */
            set_freq_monitoring_feature(true); 
            gpio_enable_interrupts();

            /* Task Create */
            xTaskCreate(grid_freq_task, "grid_freq_task", 4096, NULL, 1, &task_handle_freq_grid);
            xTaskCreate(sensor_freq_task, "sensor_freq_task", 4096, NULL, 1, &task_handle_freq_sensor);
            break;
        }
        case STATE_OPERATIONAL:
        {
            /* Receiving and Setting Gen Empty Diff time */
            trait_messages(false, false, false, true);
            
            /* Convert received value to uint16_t */
            uint16_t value = (uint16_t) atoi((char *)udp_receive_buffer);
            ESP_LOGI(MAIN_TAG, "Gen Empty Diff Time: %u", value);

            /* Set the gen_empty_diff_time */
            set_gen_empty_time_diff(value);

            /* Set the operational flag to allow trip signal */
            set_operational_mode(true);

            /* Enable interrupts */
            gpio_enable_interrupts();
        }
        
        default:
        {
            break;
        }
    }
}

static void process_sensor_to_grid_diff_time (void)
{
    esp_err_t err = time_difference_function(queue_time_difference_main);
    
    if (err == ESP_OK)
    {
        /* Copy time_difference buffer to transmission while sampling continues */
        xQueueReceive(queue_time_difference_main, &time_difference, portMAX_DELAY);
        
        /* Format the data to transmit */
        uint16_t pos = 0;
        for (uint16_t i = 0; i < NUM_CYCLES_DIFF_PULSE; i++)
        {
            int len = snprintf((udp_send_buffer + pos), (sizeof(udp_send_buffer) - pos), "%u,", time_difference[i]);
            pos += len;
        }
    
        /* Send data to vision without null-terminator */
        pos = pos - 1;
        if (xSemaphoreTake(udp_semaphore, portMAX_DELAY) == pdTRUE) 
        {
            udp_socket_send(udp_send_buffer, pos);
            xSemaphoreGive(udp_semaphore);
        }
    }
}

/*-------------------------------------------------------
 * FreeRTOS Tasks
**-------------------------------------------------------*/

void grid_freq_task(void *pvParameters)
{
    while(true)
    {
        esp_err_t err = take_grid_period(queue_grid_period_main);
        if (err == ESP_OK)
        {
            /* Take the grid pulse period buffer */
            xQueueReceive(queue_grid_period_main, &time_array_freq_grid, portMAX_DELAY);

            if (xSemaphoreTake(udp_semaphore, portMAX_DELAY) == pdTRUE) 
            {
                /* Format the data to transmit */
                uint16_t pos = 0;
                for (uint16_t i = 0; i < GRID_FREQUENCY_BUFFER_SIZE; i++)
                {
                    int len = snprintf((udp_send_buffer + pos), (sizeof(udp_send_buffer) - pos), "%u,", time_array_freq_grid[i]);
                    pos += len;
                }
            
                /* Send data to vision without null-terminator */
                pos = pos - 1;

                udp_socket_send(udp_send_buffer, pos);
                xSemaphoreGive(udp_semaphore);
            }
        }
        else
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
}

void sensor_freq_task(void *pvParameters)
{
    while(true)
    {
        esp_err_t err = take_sensor_period(queue_sensor_period_main);
        if (err == ESP_OK)
        {
            /* Take the sensor pulse period buffer */
            xQueueReceive(queue_sensor_period_main, &time_array_freq_sensor, portMAX_DELAY);

            if (xSemaphoreTake(udp_semaphore, portMAX_DELAY) == pdTRUE) 
            {
                /* Format the data to transmit */
                uint16_t pos = 0;
                for (uint16_t i = 0; i < SENSOR_FREQUENCY_BUFFER_SIZE; i++)
                {
                    int len = snprintf((udp_send_buffer + pos), (sizeof(udp_send_buffer) - pos), "%u,", time_array_freq_sensor[i]);
                    pos += len;
                }
            
                /* Send data to vision without null-terminator */
                pos = pos - 1;

                udp_socket_send(udp_send_buffer, pos);
                xSemaphoreGive(udp_semaphore);
            }
        }
        else
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
}