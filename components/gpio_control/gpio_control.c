#include "driver/gpio.h"
#include "esp_attr.h"  
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "project_types.h"

/*********************************************************
 * Defines
*********************************************************/
#define GPIO_CONTROL_TAG "GPIO_CONTROL"
#define ESP_INTR_FLAG_DEFAULT  ESP_INTR_FLAG_IRAM
#define QUEUE_TIME_DIFF_LENGTH 60 // Arbitrary length for the queue to hold time differences

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

/** @brief Queue to send diff time from ISR to time difference function */
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
        volatile uint64_t time_now = esp_timer_get_time();
        delta_time_freq_grid = time_now - T_firstpulse_grid;
        if (delta_time_freq_grid > GRID_WINDOW_FILTER) 
        {
            T_firstpulse_grid = time_now;
        
            if(sensor_pulse_ready) 
            {
                uint16_t time_diff = (uint16_t)(T_firstpulse_grid - sensor_pulse_moment_reference);
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
        volatile uint64_t time_now = esp_timer_get_time();
        delta_time_freq_sensor = time_now - T_firstpulse_sensor;
        if (delta_time_freq_sensor > SENSOR_WINDOW_FILTER)
        {
            /* Update period time reference */
            T_firstpulse_sensor = time_now;

            if (!sensor_pulse_ready)
            {
                /* Take time reference to calculate grid-sensor diff time */
                sensor_pulse_moment_reference = T_firstpulse_sensor;

                /* Enable grid ISR to calculate time difference and block sensor for a new measure */
                sensor_pulse_ready = true;
            }
        }
    }
}

/*********************************************************
 * Private Functions
*********************************************************/



/*********************************************************
 * Public Functions
*********************************************************/
esp_err_t gpio_init(void)
{   
    /* Create a queue to hold time difference values */
    queue_time_difference = xQueueCreate(QUEUE_TIME_DIFF_LENGTH, sizeof(uint32_t));
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

    /* Start interrupts on pause */
    gpio_isr_handler_add(ELETRIC_GRID_PIN, grid_itr_callback, NULL);
    gpio_intr_disable(ELETRIC_GRID_PIN);

    /* Config for Sensor Signal */
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<SENSOR_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    /* Start interrupts on pause */
    gpio_isr_handler_add(SENSOR_PIN, sensor_itr_callback, NULL);
    gpio_intr_disable(SENSOR_PIN);

    return ESP_OK;
}

esp_err_t time_difference_function(QueueHandle_t queue_time_difference_main)
{
    /* Fill diff buffer with data from ISR */
    uint16_t time_diff[NUM_CYCLES_DIFF_PULSE] = {0};
    for (uint16_t i = 0; i < NUM_CYCLES_DIFF_PULSE; i++)
    {
        if (xQueueReceive(queue_time_difference, &time_diff[i], portMAX_DELAY) != pdTRUE)
        {
            i -= 1; 
            ESP_LOGE(GPIO_CONTROL_TAG, "Failed to receive time difference from ISR");
        }
    }

    /* Send diff buffer to main application */
    if (queue_time_difference_main != NULL)
    {
        if (xQueueSend(queue_time_difference_main, &time_diff, portMAX_DELAY) == pdTRUE)
            return ESP_OK;
    }

    return ESP_FAIL;
}

void gpio_enable_interrupts(void)
{
    /* Enable interrupts for grid and sensor */
    gpio_intr_enable(ELETRIC_GRID_PIN);
    gpio_intr_enable(SENSOR_PIN);
}

void gpio_disable_interrupts(void)
{
    /* Disable interrupts for grid and sensor */
    gpio_intr_disable(ELETRIC_GRID_PIN);
    gpio_intr_disable(SENSOR_PIN);
}
