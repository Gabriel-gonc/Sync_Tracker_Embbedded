#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

#include "esp_err.h"

#include "freertos/queue.h"

#include "project_types.h"

/*********************************************************
 * Public Functions
*********************************************************/
esp_err_t gpio_init(void);

esp_err_t time_difference_function(QueueHandle_t queue_time_difference_main);

void gpio_enable_interrupts(void);

void gpio_disable_interrupts(void);

void set_diff_time_feature(bool enable);

void set_freq_monitoring_feature(bool enable);

esp_err_t take_grid_period(QueueHandle_t queue_grid_period_main);

esp_err_t take_sensor_period(QueueHandle_t queue_sensor_period_main);

void set_gen_empty_time_diff(uint16_t value);

void set_operational_flag(bool value);

#endif // GPIO_CONTROL_H