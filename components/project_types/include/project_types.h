#ifndef PROJECT_TYPES_H
#define PROJECT_TYPES_H

/***********************************************************
 * Project Defines
***********************************************************/
/** @brief Pinout */
#define ELETRIC_GRID_PIN GPIO_NUM_5
#define SENSOR_PIN GPIO_NUM_4

#define ESP_INTR_FLAG_DEFAULT 0

/** @brief Project Constants */
#define PARES_POLOS 2
#define NUM_CYCLES_FREQ_GRID 60
#define NUM_CYCLES_FREQ_SENSOR (NUM_CYCLES_FREQ_GRID / PARES_POLOS)
#define NUM_CYCLES_DIFF_PULSE NUM_CYCLES_FREQ_GRID * 10
#define GRID_WINDOW_FILTER 15000 // 11ms
#define SENSOR_WINDOW_FILTER (GRID_WINDOW_FILTER * PARES_POLOS)

/** @brief UDP Constants */
#define UDP_RX_BUFFER_SIZE 50
#define UDP_TX_BUFFER_SIZE NUM_CYCLES_DIFF_PULSE * 6

/***********************************************************
 * Project Types
***********************************************************/
/** @brief Enum system events. */
typedef enum {
    SYSTEM_WIFI_CONNECTED,
    SYSTEM_WIFI_DISCONNECTED,
    SYSTEM_UDP_CREATED,
} system_event_t;

/** @brief System callback. */
typedef void (*system_callback_t)(system_event_t event);

#endif // PROJECT_TYPES_H