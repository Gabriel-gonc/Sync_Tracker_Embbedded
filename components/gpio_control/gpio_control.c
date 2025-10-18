#include "driver/gpio.h"
#include "esp_attr.h"  
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "project_types.h"

/*********************************************************
 * Defines
*********************************************************/
#define GPIO_CONTROL_TAG "GPIO_CONTROL"
#define ESP_INTR_FLAG_DEFAULT  ESP_INTR_FLAG_IRAM
#define QUEUE_DATA_LENGHT 60 // Arbitrary length for the queue to hold ISR data
#define QUEUE_TIMEOUT 100 // Timeout in ms to wait for data from ISR
#define OPER_BUFFER_LENGHT 16
#define OPER_TIME_TOLERANCE 150 // In micros
#define WASTE_SAMPLES 60

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

/** @brief Var to enable frequency monitoring  */
static volatile bool enable_freq_monitoring = false;

/** @brief Var to enable operational mode */
static volatile bool operational_flag = false;

/** @brief Queue to send diff time from ISR to operation_mode task */
static QueueHandle_t queue_time_difference_oper = NULL;

/** @brief Queue to send diff time from ISR to time difference function */
static QueueHandle_t queue_time_difference = NULL;

/** @brief Queue to send grid pulse period time */
static QueueHandle_t queue_grid_period = NULL;

/** @brief Queue to send sensor pulse period time */
static QueueHandle_t queue_sensor_period = NULL;

/** @brief Queue to take Loss of Synchronism Fault */
static QueueHandle_t GPIO_queue_loss_of_synchronism = NULL;

/** @brief Task handle to freq_grid task */
static TaskHandle_t task_handle_oper_mode = NULL;

/** @brief Generator Difference Time at No Load Condition */
static uint16_t gen_empty_diff_time = 00u;

/**********************************************************
 * Function Prototypes
**********************************************************/
static void sync_fault_detected(void);
esp_err_t gpio_init(QueueHandle_t synchronism_fault);
esp_err_t time_difference_function(QueueHandle_t queue_time_difference_main);
void gpio_enable_interrupts(void);
void gpio_disable_interrupts(void);
void set_diff_time_feature(bool enable);
void set_freq_monitoring_feature(bool enable);
void set_operational_mode(bool value);
void delete_operation_mode_task(void);
esp_err_t take_grid_period(QueueHandle_t queue_grid_period_main);
esp_err_t take_sensor_period(QueueHandle_t queue_sensor_period_main);
void set_gen_empty_time_diff(uint16_t value);
static void operation_mode(void *PvParameters);

/*-------------------------------------------------------
 * ISR Callbacks
**-------------------------------------------------------*/
static void IRAM_ATTR grid_itr_callback(void *arg)
{
    if (triggered_process_freq_sensor)
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
                if((sensor_pulse_ready) && (enable_diff_time_feature || operational_flag))
                {
                    /* Calculate time_diff from empty value ref */
                    uint16_t time_diff = (uint16_t)(T_firstpulse_grid - sensor_pulse_moment_reference);

                    /* Send the time_diff for operational mode */
                    if (operational_flag)
                    {
                        /* Variable to check if a higher priority task is free */
                        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

                        /* Add item to the queue */
                        xQueueSendFromISR(queue_time_difference_oper, &time_diff, &xHigherPriorityTaskWoken);

                        /* Schedule the higher priority task if free */
                        if (xHigherPriorityTaskWoken == pdTRUE)
                        {
                            portYIELD_FROM_ISR();
                        }
                    }
                    
                    /* Send the time_diff for monitoring mode */
                    else if (enable_diff_time_feature)
                    {
                        xQueueSendFromISR(queue_time_difference, &time_diff, NULL);
                    }

                    /* Enable sensor ISR for a new diff time cycle */
                    sensor_pulse_ready = false;
                } 
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
            if ((!sensor_pulse_ready) && (enable_diff_time_feature || operational_flag))
            {
                /* Take time reference to calculate grid-sensor diff time */
                sensor_pulse_moment_reference = time_now;

                /* Enable grid ISR to calculate time difference and block sensor for a new measure */
                sensor_pulse_ready = true;
            }
        }
    }
}

/*-------------------------------------------------------
 * Local Functions
**-------------------------------------------------------*/
void sync_fault_detected(void)
{
    /* Open the circuit */
	gpio_set_level(BREAKER_PIN, 1); 

	/* Sign Error */
	bool fault = true;
	if (GPIO_queue_loss_of_synchronism != NULL)
	{
	    xQueueSendFromISR(GPIO_queue_loss_of_synchronism, &fault, NULL);
	}
}

/*-------------------------------------------------------
 * Functions
**-------------------------------------------------------*/
esp_err_t gpio_init(QueueHandle_t synchronism_fault)
{   
    /* Create a queue to hold time difference values for operation_mode task*/
    queue_time_difference_oper = xQueueCreate(QUEUE_DATA_LENGHT, sizeof(uint32_t));
    if (queue_time_difference_oper == NULL) 
    {
        ESP_LOGE(GPIO_CONTROL_TAG, "Failed to create queue");
        return ESP_FAIL;
    }

    /* Create a queue to hold time difference values for time difference function */
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

    /* Receiving loss of synchronism queue */
    if (GPIO_queue_loss_of_synchronism == NULL)
    {
        GPIO_queue_loss_of_synchronism = synchronism_fault;
    }

    /* Install ISR Service*/
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    /* Config for Breaker Signal */
    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<BREAKER_PIN);
    gpio_config(&io_conf);

    /* Set Breaker to LOW (closed) */
    gpio_set_level(BREAKER_PIN, 0); 

    /* Config for Grid Signal */
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
        if (xQueueReceive(queue_time_difference, &time_diff[i], (QUEUE_TIMEOUT / portTICK_PERIOD_MS)) != pdTRUE)
        { 
            time_diff[i] = 65535;
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

esp_err_t take_grid_period(QueueHandle_t queue_grid_period_main)
{  
    /* Fill grid period buffer with data from ISR */
    uint16_t grid_period[GRID_FREQUENCY_BUFFER_SIZE] = {0};
    grid_period[0] = 1;
    for (uint16_t i = 1; i < (GRID_FREQUENCY_BUFFER_SIZE); i++)
    {
        if (xQueueReceive(queue_grid_period, &grid_period[i], (QUEUE_TIMEOUT / portTICK_PERIOD_MS)) != pdTRUE)
        {
            grid_period[i] = 65535;
            ESP_LOGE(GPIO_CONTROL_TAG, "Failed to receive grid period from ISR");
        }
    }

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
        if (xQueueReceive(queue_sensor_period, &sensor_period[i], (QUEUE_TIMEOUT / portTICK_PERIOD_MS)) != pdTRUE)
        {
            sensor_period[i] = 65535;
            ESP_LOGE(GPIO_CONTROL_TAG, "Failed to receive sensor period from ISR");
        }
    }

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
}

void gpio_disable_interrupts(void)
{
    /* Disable interrupts for grid and sensor */
    gpio_intr_disable(ELETRIC_GRID_PIN);
    gpio_intr_disable(SENSOR_PIN);
}

void set_diff_time_feature(bool enable)
{
    enable_diff_time_feature = enable;
    if (enable)
    {
        /* Clean the diff time queue */
        xQueueReset(queue_time_difference);

        /* Reset the triggered process flags */
        triggered_process_freq_sensor = false;
        triggered_process_freq_grid = false;

        /* Reset the time references */
        T_firstpulse_sensor = 0;
        T_firstpulse_grid = 0;

        sensor_pulse_ready = false;
    }
}

void set_freq_monitoring_feature(bool enable)
{
    enable_freq_monitoring = enable;
    if (enable)
    {
        /* Clean frequency monitoring queues */
        xQueueReset(queue_grid_period);
        xQueueReset(queue_sensor_period);

        /* Reset the triggered process flags */
        triggered_process_freq_sensor = false;
        triggered_process_freq_grid = false;

        /* Reset the time references */
        T_firstpulse_sensor = 0;
        T_firstpulse_grid = 0;
    }
}

void set_operational_mode(bool value)
{
    operational_flag = value;

    if (value)
    {
        /* Clean the diff time queue */
        xQueueReset(queue_time_difference_oper);

        /* Create the task */
        xTaskCreate(operation_mode, "operation_mode_task", 4096, NULL, 2, &task_handle_oper_mode);

        /* Reset the triggered process flags */
        triggered_process_freq_sensor = false;
        triggered_process_freq_grid = false;
    
        /* Reset the time references */
        T_firstpulse_sensor = 0;
        T_firstpulse_grid = 0;

        /* Reset grid to sensor pulse sync flag */
        sensor_pulse_ready = false;
    }
}

void delete_operation_mode_task(void)
{
    if (task_handle_oper_mode != NULL)
    {
        vTaskDelete(task_handle_oper_mode);
        task_handle_oper_mode = NULL;
    }
}

void set_gen_empty_time_diff(uint16_t value)
{
    gen_empty_diff_time = value;
}

/*-------------------------------------------------------
 * FreeRTOS Tasks
**-------------------------------------------------------*/
static void operation_mode(void *pvParameters)
{
    /* Start the used variables */
    uint32_t operation_buffer[OPER_BUFFER_LENGHT] = {0u};
    uint32_t current_time_diff = 0u;
    uint64_t time_diff_sum = 0u;
    float time_diff_avg = 0.0f;
    bool swing_flag = false;
    float pre_fault_angle = 0;
    uint32_t max_diff_time = 0;
    float critic_angle_degrees = 0;
    uint32_t critic_diff_time = 0;

    /* Discard the first samples */
    for(uint8_t i = 0; i < WASTE_SAMPLES; i++)
    {
        xQueueReceive(queue_time_difference_oper, &current_time_diff, portMAX_DELAY);
    }

    /* Fill the initial buffer */
    for (uint8_t i = 0; i < OPER_BUFFER_LENGHT; i++)
    {
        xQueueReceive(queue_time_difference_oper, &current_time_diff, portMAX_DELAY);
        operation_buffer[i] = current_time_diff;

        /* Sum to calculate the average */
        time_diff_sum += current_time_diff;
    }

    /* Calculate the initial average */
    time_diff_avg = ((float)time_diff_sum) / OPER_BUFFER_LENGHT;

    while (true)
    {
        /* Receive a new time diff */
        xQueueReceive(queue_time_difference_oper, &current_time_diff, portMAX_DELAY);

        /* Comparison with OPER_TIME_TOLERANCE to detect power swings */
        if (abs(current_time_diff - (uint32_t)(time_diff_avg)) > OPER_TIME_TOLERANCE)
        {
            if (swing_flag)
            {
                if (current_time_diff > critic_diff_time)
                {
                    /* Fault not cleared, generate the trip signal */
                    if ((current_time_diff - operation_buffer[0]) > (operation_buffer[0] - operation_buffer[1]))
                    {
                        sync_fault_detected();
                        vTaskDelete(NULL);
                    }
                    else if (current_time_diff >= max_diff_time)
                    {
                        sync_fault_detected();
                        vTaskDelete(NULL);
                    }
                }
            }
            else
            {
                /* Calculate the pre-fault angle using the last time diff value before swing in degrees */
                pre_fault_angle = (((float)operation_buffer[0] * 360.0f) / 16667.0f);

                /* Calculate the maximum diff time (which correspond to the operation_buffer[0] suplementary angle in time) */
                max_diff_time = 8333u - operation_buffer[0];

                /* Calculate the critic angle */
                float pre_fault_angle_rad = (pre_fault_angle * M_PI) / 180.0f;
                critic_angle_degrees = acosf(((M_PI - 2.0 * pre_fault_angle_rad) * sinf(pre_fault_angle_rad)) - cosf(pre_fault_angle_rad));
                critic_angle_degrees = critic_angle_degrees * 180.0f / M_PI;

                /* Calculate the critic diff time */
                critic_diff_time = (uint32_t)(critic_angle_degrees * 8333.3f / 180.0f);

                /* Assert the swing flag */
                swing_flag = true;
            }
        }
        else
        {
            /* Deassert the swing flag */
            swing_flag = false;
        }
        
        /* Reset the buffer sum */
        time_diff_sum = 0u;
        
        /* Update the operation_buffer, moving terms to open position for the new value */
        for (uint8_t i = (OPER_BUFFER_LENGHT - 1); i > 0; i--)
        {
            operation_buffer[i] = operation_buffer[i - 1];
            
            /* Sum to calculate the average */
            time_diff_sum += operation_buffer[i];
        }
        operation_buffer[0] = current_time_diff;
        time_diff_sum += current_time_diff; 

        /* Update the average */
        time_diff_avg = ((float)time_diff_sum) / OPER_BUFFER_LENGHT;   
    }
}