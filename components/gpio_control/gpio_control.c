#include "driver/gpio.h"
#include "esp_attr.h"  
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "project_types.h"

/*********************************************************
 * Defines
*********************************************************/
#define ESP_INTR_FLAG_DEFAULT  ESP_INTR_FLAG_IRAM
#define QUEUE_TIME_DIFF_LENGTH 10

/*********************************************************
 * Variables
*********************************************************/
/** @brief Vars to sign first measure cycle triggered. */
static volatile bool triggered_process_freq_sensor = false;
static volatile bool triggered_process_freq_grid = false;

/** @brief Variable to set reference to calculate period 
 *          for both sensor and grid */
static volatile uint64_t T_firstpulse_sensor = 0;
static volatile uint64_t T_firstpulse_grid = 0;

/** @brief Variable to set reference to calculate time difference
 *          between sensor and grid pulses  */
static volatile uint64_t sensor_pulse_moment_reference = 0;

/** @brief Variable to store the pulse period 
 *          for both sensor and grid */
static volatile uint64_t delta_time_freq_sensor = 0;
static volatile uint64_t delta_time_freq_grid = 0;

/** @brief Var to sign sensor pulse ready to calculate time difference */
static volatile bool sensor_pulse_ready = false;

QueueHandle_t queue_time_difference = NULL;

/**********************************************************
 * Function Prototypes
**********************************************************/
esp_err_t gpio_init(void);

/*********************************************************
 * Callbacks
*********************************************************/
static void IRAM_ATTR grid_itr_callback(void *arg)
{
    if (!triggered_process_freq_grid)
    {
        T_firstpulse_grid = esp_timer_get_time();
        triggered_process_freq_grid = true;
    }
    else 
    {
        delta_time_freq_grid = esp_timer_get_time() - T_firstpulse_grid;
        if (delta_time_freq_grid > GRID_WINDOW_FILTER) 
        {
            T_firstpulse_grid = esp_timer_get_time();
        
            if(sensor_pulse_ready) 
            {
                uint64_t time_diff = T_firstpulse_grid - sensor_pulse_moment_reference;
                xQueueSendFromISR(queue_time_difference, &time_diff, NULL);
                sensor_pulse_ready = false;
            } 
        }
    }
}

static void IRAM_ATTR sensor_itr_callback(void *arg)
{
    if (!triggered_process_freq_sensor)
    {
        T_firstpulse_sensor = esp_timer_get_time();
        triggered_process_freq_sensor = true;
    }
    else 
    {
        delta_time_freq_sensor = esp_timer_get_time() - T_firstpulse_sensor;
        if (delta_time_freq_sensor > SENSOR_WINDOW_FILTER)
        {
            /* Update period time reference */
            T_firstpulse_sensor = esp_timer_get_time();

            /* Enable grid ISR to calculate time difference */
            if (!sensor_pulse_ready)
            {
                /* Take time reference to calculate grid-sensor diff time */
                sensor_pulse_moment_reference = T_firstpulse_sensor;
                sensor_pulse_ready = true;
            }
        }
    }
}

/*********************************************************
 * Private Functions
*********************************************************/
static void time_difference_function()
{
    uint64_t time_diff = 0;
    if (xQueueReceive(queue_time_difference, &time_diff, portMAX_DELAY) == pdTRUE)
    {
        // Process the received time difference
    }
}


/*********************************************************
 * Public Functions
*********************************************************/

esp_err_t gpio_init(void)
{   
    /* Create a queue to hold time difference values */
    queue_time_difference = xQueueCreate(QUEUE_TIME_DIFF_LENGTH, sizeof(uint64_t));
    if (queue_time_difference == NULL) 
    {
        ESP_LOGE("GPIO_CONTROL", "Failed to create queue");
        return ESP_FAIL;
    }

    /* Install ISR Service*/
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    /* Config for Grid Signal */
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE; 
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<ELETRIC_GRID_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;

    gpio_config(&io_conf);
    gpio_isr_handler_add(ELETRIC_GRID_PIN, grid_itr_callback, NULL);

    /* Config for Sensor Signal */
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<SENSOR_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;

    gpio_config(&io_conf);
    gpio_isr_handler_add(SENSOR_PIN, sensor_itr_callback, NULL);

    return ESP_OK;
}

