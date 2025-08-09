#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>

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
#define MAIN_TAG "MAIN"

/***********************************************************
 * Function Prototypes
***********************************************************/
static void system_callback(system_event_t event);
static system_state_t trait_messages(bool hand_shaking, bool state_comp);
static void time_difference_mock();

/***********************************************************
 * Variables
***********************************************************/
/** @brief Array to store the time difference between sensor and grid pulses */
static volatile uint16_t time_difference[NUM_CYCLES_DIFF_PULSE] = {0};

/** @brief Event Group variables. */
EventGroupHandle_t main_event_group = NULL;
const int WIFI_CONNECTED_BIT = BIT0;

/** @brief Buffers for udp socket. */
char udp_receive_buffer[UDP_RX_BUFFER_SIZE] = {0};
char udp_send_buffer[UDP_TX_BUFFER_SIZE] = {0};

/** @brief Variable to store current System State */
static volatile system_state_t current_state = STATE_IDLE;

/***********************************************************
 * Main Function
***********************************************************/
void app_main(void)
{
    /* Create event group */
    main_event_group = xEventGroupCreate();

    /* Init wifi*/
    wifi_init_softap();
    wifi_get_callback(system_callback);

    /* Create udp socket */
    xEventGroupWaitBits(main_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
    udp_socket_init();

    /* Establishing connection */
    current_state = trait_messages(true, false);
    if (current_state != STATE_CONNECTED)
    {
        ESP_LOGE(MAIN_TAG, "Failed to establish connection!");
        esp_restart();
    }

    /* Receiving next state command */
    current_state = trait_messages(false, true);
    
    /* Mocking monitoring data */
    if (current_state == STATE_MONITORING)
    {
        time_difference_mock();
    }

    /* Main loop. */
    while(true)
    {
        vTaskDelay(20000 / portTICK_PERIOD_MS);
    }
}

/***********************************************************
 * System Callback
***********************************************************/
/** @brief Function to enable inter-components communication */
static void system_callback(system_event_t event)
{
    switch (event) {
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

/***********************************************************
 * Functions
***********************************************************/
/** @brief Function to receive, trait and send udp control messages
 * 
 * @param hand_shaking True for hand-shaking process, false otherwise
 * @param state_comp True for state setting process, false otherwise
 */
static system_state_t trait_messages(bool hand_shaking, bool state_comp)
{
    while (1)
    {
        int bytes_received = udp_socket_receive(udp_receive_buffer, sizeof(udp_receive_buffer));
        if (bytes_received > 0)
        {
            if (hand_shaking && !state_comp)
            {
                if (strncmp(udp_receive_buffer, MSG_SYNC, strlen(MSG_SYNC)) == 0)
                {
                    int len = snprintf(udp_send_buffer, sizeof(udp_send_buffer), "%s", CMD_ACK);
                    udp_socket_send(udp_send_buffer, len);
                    ESP_LOGI(MAIN_TAG, "Connected!");
                    return STATE_CONNECTED;  
                }
                else 
                {
                    int len = snprintf(udp_send_buffer, sizeof(udp_send_buffer), "%s", CMD_NACK);
                    udp_socket_send(udp_send_buffer, len);
                }
            }

            if (state_comp && !hand_shaking)
            {
                if (strncmp(udp_receive_buffer, MSG_MON, strlen(MSG_MON)) == 0)
                {
                    ESP_LOGI(MAIN_TAG, "Received monitoring command");
                    int len = snprintf(udp_send_buffer, sizeof(udp_send_buffer), "%s", CMD_ACK);
                    udp_socket_send(udp_send_buffer, len);
                    return STATE_MONITORING;
                }
                else if (strncmp(udp_receive_buffer, MSG_OP, strlen(MSG_OP)) == 0)
                {
                    ESP_LOGI(MAIN_TAG, "Received operational command");
                    int len = snprintf(udp_send_buffer, sizeof(udp_send_buffer), "%s", CMD_ACK);
                    udp_socket_send(udp_send_buffer, len);
                    return STATE_OPERATIONAL;
                }
                else 
                {
                    ESP_LOGW(MAIN_TAG, "Invalid command received: %s", udp_receive_buffer);
                    int len = snprintf(udp_send_buffer, sizeof(udp_send_buffer), "%s", CMD_NACK);
                    udp_socket_send(udp_send_buffer, len);
                }
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/** @brief Funtion to mock measure of the pulse difference time */
static void time_difference_mock()
{
    uint16_t delay_time = (uint16_t)((NUM_CYCLES_DIFF_PULSE / SENSOR_FREQUENCY) * 1000);

    while (true)
    {
        for (uint16_t i = 0; i < NUM_CYCLES_DIFF_PULSE; i++)
        {
            time_difference[i] = 10000 + 100 * i; 
        }

        /* Simulate grid sampling delay */
        vTaskDelay(delay_time / portTICK_PERIOD_MS);

        uint16_t pos = 0;
        for (uint16_t i = 0; i < NUM_CYCLES_DIFF_PULSE; i++)
        {
            int len = snprintf((udp_send_buffer + pos), (sizeof(udp_send_buffer) - pos), "%u,", time_difference[i]);
            pos += len;
        }

        /* Avoid send the null terminator. */
        pos = pos - 1;
        udp_socket_send(udp_send_buffer, pos);
    }
}

/***********************************************************
 * FreeRTOS Tasks
***********************************************************/


/***********************************************************
 * Callbacks for GPIO Interrupts
***********************************************************/
