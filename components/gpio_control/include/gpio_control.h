#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

#include "esp_err.h"

#include "freertos/queue.h"

#include "project_types.h"

/*********************************************************
 * Funções
*********************************************************/
esp_err_t gpio_init(void);

esp_err_t time_difference_function(QueueHandle_t queue_time_difference_main);

#endif // GPIO_CONTROL_H