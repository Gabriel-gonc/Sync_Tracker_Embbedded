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
#define QUEUE_DATA_LENGHT 60 // Arbitrary length for the queue to hold ISR data

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
static uint16_t delta_time_freq_sensor = 0;
static uint16_t delta_time_freq_grid = 0;

/** @brief Var to sign sensor pulse ready to calculate time difference */
static volatile bool sensor_pulse_ready = false;

/** @brief Var to enable sensor-grid pulse diff time */
static volatile bool enable_diff_time_feature = false;

static volatile bool interrupt_enabled = false; // ERASE, JUST FOR TESTS *************************************************************

/** @brief Var to enable frequency monitoring  */
static volatile bool enable_freq_monitoring = false;

/** @brief Queue to send diff time from ISR to time difference function */
QueueHandle_t queue_time_difference = NULL;

/** @brief Queue to send grid pulse period time */
QueueHandle_t queue_grid_period = NULL;

/** @brief Queue to send sensor pulse period time */
QueueHandle_t queue_sensor_period = NULL;

/**********************************************************
 * Function Prototypes
**********************************************************/
esp_err_t gpio_init(void);
esp_err_t time_difference_function(QueueHandle_t queue_time_difference_main);
void gpio_enable_interrupts(void);
void gpio_disable_interrupts(void);
void set_diff_time_feature(bool enable);
void set_freq_monitoring_feature(bool enable);
esp_err_t take_grid_period(QueueHandle_t queue_grid_period_main);
esp_err_t take_sensor_period(QueueHandle_t queue_sensor_period_main);

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
        delta_time_freq_grid = (uint16_t)(time_now - T_firstpulse_grid);
        if (delta_time_freq_grid > GRID_WINDOW_FILTER) 
        {
            T_firstpulse_grid = time_now;

            /* Frequency monitoring feature */
            if (enable_freq_monitoring)
            {
                /* Store grid pulse period */
                xQueueSendFromISR(queue_grid_period, &delta_time_freq_grid, NULL);
            }
            
            /* Diff time ISR feature */
            if((sensor_pulse_ready) && (enable_diff_time_feature))
            {
                uint16_t time_diff = (uint16_t)(T_firstpulse_grid - sensor_pulse_moment_reference);
                xQueueSendFromISR(queue_time_difference, &time_diff, NULL);

                /* Enable sensor ISR for a new diff time cycle */
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
        delta_time_freq_sensor = (uint16_t)(time_now - T_firstpulse_sensor);
        if (delta_time_freq_sensor > SENSOR_WINDOW_FILTER)
        {
            /* Update period time reference */
            T_firstpulse_sensor = time_now;

            /* Frequency monitoring feature */
            if (enable_freq_monitoring)
            {
                /* Store sensor pulse period */
                xQueueSendFromISR(queue_sensor_period, &delta_time_freq_sensor, NULL);
            }

            /* Diff time ISR feature */
            if ((!sensor_pulse_ready) && (enable_diff_time_feature))
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
    queue_time_difference = xQueueCreate(QUEUE_DATA_LENGHT, sizeof(uint32_t));
    if (queue_time_difference == NULL) 
    {
        ESP_LOGE(GPIO_CONTROL_TAG, "Failed to create queue");
        return ESP_FAIL;
    }

    /* Create a queue to hold grid pulse period values */
    queue_grid_period = xQueueCreate(QUEUE_DATA_LENGHT, sizeof(uint32_t));
    if (queue_grid_period == NULL) 
    {
        ESP_LOGE(GPIO_CONTROL_TAG, "Failed to create grid period queue");
        return ESP_FAIL;
    }

    /* Create a queue to hold sensor pulse period values */
    queue_sensor_period = xQueueCreate(QUEUE_DATA_LENGHT, sizeof(uint32_t));
    if (queue_sensor_period == NULL) 
    {
        ESP_LOGE(GPIO_CONTROL_TAG, "Failed to create sensor period queue");
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
        // if (xQueueReceive(queue_time_difference, &time_diff[i], portMAX_DELAY) != pdTRUE)
        // { DISCOMENT
        //     i -= 1; 
        //     ESP_LOGE(GPIO_CONTROL_TAG, "Failed to receive time difference from ISR");
        // }

        if (enable_diff_time_feature && interrupt_enabled)
        {
            time_diff[i] = 10000 - 2 * (i * i);
        }
    }
    // Just for tests, ERASE
    vTaskDelay((SAMPLE_DURATION_SEC * 1000) / portTICK_PERIOD_MS);

    /* Send diff buffer to main application */
    if (queue_time_difference_main != NULL)
    {
        if (xQueueSend(queue_time_difference_main, &time_diff, portMAX_DELAY) == pdTRUE)
            return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t take_grid_period(QueueHandle_t queue_grid_period_main)
{  
    /* Fill grid period buffer with data from ISR */
    uint16_t grid_period[GRID_FREQUENCY_BUFFER_SIZE] = {0};
    grid_period[0] = 1;
    for (uint16_t i = 1; i < (GRID_FREQUENCY_BUFFER_SIZE); i++)
    {
        // if (xQueueReceive(queue_grid_period, &grid_period[i], portMAX_DELAY) != pdTRUE)
        // {
        //     i -= 1; 
        //     ESP_LOGE(GPIO_CONTROL_TAG, "Failed to receive grid period from ISR");
        // }

        if (enable_freq_monitoring && interrupt_enabled)
        {
            srandom(esp_timer_get_time());  // inicializa semente
            uint16_t r = (uint16_t)(random() % 100);    // número entre 0 e 99
            grid_period[i] = 16667 + r;
        }
    }

    // Just for tests, ERASE
    vTaskDelay((SAMPLE_DURATION_SEC * 1000) / portTICK_PERIOD_MS);

    /* Send grid period buffer to main application */
    if (queue_grid_period_main != NULL)
    {
        if (xQueueSend(queue_grid_period_main, &grid_period, portMAX_DELAY) == pdTRUE)
            return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t take_sensor_period(QueueHandle_t queue_sensor_period_main)
{  
    /* Fill sensor period buffer with data from ISR */
    uint16_t sensor_period[SENSOR_FREQUENCY_BUFFER_SIZE] = {0};
    sensor_period[0] = 2;
    for (uint16_t i = 1; i < SENSOR_FREQUENCY_BUFFER_SIZE; i++)
    {
        // if (xQueueReceive(queue_sensor_period, &sensor_period[i], portMAX_DELAY) != pdTRUE)
        // {
        //     i -= 1; 
        //     ESP_LOGE(GPIO_CONTROL_TAG, "Failed to receive sensor period from ISR");
        // }

        if (enable_freq_monitoring && interrupt_enabled)
        {
            srandom(esp_timer_get_time());  // inicializa semente
            uint16_t r = (uint16_t)(random() % 100);    // número entre 0 e 99
            sensor_period[i] = (16667 * 2) + r;
        }
    }

    // Just for tests, ERASE
    vTaskDelay((SAMPLE_DURATION_SEC * 1000) / portTICK_PERIOD_MS);

    /* Send sensor period buffer to main application */
    if (queue_sensor_period_main != NULL)
    {
        if (xQueueSend(queue_sensor_period_main, &sensor_period, portMAX_DELAY) == pdTRUE)
            return ESP_OK;
    }

    return ESP_FAIL;
}

void gpio_enable_interrupts(void)
{
    /* Enable interrupts for grid and sensor */
    gpio_intr_enable(ELETRIC_GRID_PIN);
    gpio_intr_enable(SENSOR_PIN);
    interrupt_enabled = true; // ERASE, JUST FOR TESTS *********************************
}

void gpio_disable_interrupts(void)
{
    /* Disable interrupts for grid and sensor */
    gpio_intr_disable(ELETRIC_GRID_PIN);
    gpio_intr_disable(SENSOR_PIN);
    interrupt_enabled = false; // ERASE, JUST FOR TESTS *********************************
}

void set_diff_time_feature(bool enable)
{
    enable_diff_time_feature = enable;
    if (enable)
    {
        /* Reset the triggered process flags */
        triggered_process_freq_sensor = false;
        triggered_process_freq_grid = false;

        /* Reset the time references */
        T_firstpulse_sensor = 0;
        T_firstpulse_grid = 0;
    }
}

void set_freq_monitoring_feature(bool enable)
{
    enable_freq_monitoring = enable;
    if (enable)
    {
        /* Reset the triggered process flags */
        triggered_process_freq_sensor = false;
        triggered_process_freq_grid = false;

        /* Reset the time references */
        T_firstpulse_sensor = 0;
        T_firstpulse_grid = 0;
    }
}
