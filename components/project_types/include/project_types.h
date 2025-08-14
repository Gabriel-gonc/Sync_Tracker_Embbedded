#ifndef PROJECT_TYPES_H
#define PROJECT_TYPES_H

/***********************************************************
 * Project Defines
***********************************************************/
/** @brief Pinout */
#define ELETRIC_GRID_PIN GPIO_NUM_5
#define SENSOR_PIN GPIO_NUM_4

/** @brief Project Constants */
#define POLES_PAIRS (2) // If the machine has 1 magnet for each pole pair this value must be 1
#define GRID_FREQUENCY (60)
#define SENSOR_FREQUENCY (GRID_FREQUENCY / POLES_PAIRS)
#define SAMPLE_DURATION_SEC (1)
#define NUM_CYCLES_DIFF_PULSE (SENSOR_FREQUENCY * SAMPLE_DURATION_SEC) 
#define GRID_WINDOW_FILTER (15000) 
#define SENSOR_WINDOW_FILTER (GRID_WINDOW_FILTER * POLES_PAIRS)

/** @brief UDP Constants */
#define UDP_RX_BUFFER_SIZE 50
#define UDP_TX_BUFFER_SIZE NUM_CYCLES_DIFF_PULSE * 6

/** @brief UDP Messages and Commands */
#define MSG_SYNC "SYNC"
#define CMD_ACK "ACK"
#define CMD_NACK "NACK"
#define MSG_MON "MONITORING"
#define MSG_FNSH_MON "FINISH_MONITORING"
#define MSG_OP "OPERATIONAL"
#define MSG_FREQ "FREQUENCY"
#define MSG_FNSH_OP "FINISH_OPERATIONAL"
#define MSG_FNSH_FREQ "FINISH_FREQUENCY"

/***********************************************************
 * Project Types
***********************************************************/
/** @brief Enum system events. */
typedef enum {
    SYSTEM_WIFI_CONNECTED,
    SYSTEM_WIFI_DISCONNECTED,
    SYSTEM_UDP_CREATED,
} system_event_t;

/** @brief System States */
typedef enum {
    EXIT_CRITERIA = -1,
    MSG_RECEIVED,
    NO_MSG_RECEIVED,
    STATE_IDLE,
    STATE_CONNECTED,
    STATE_MONITORING,
    STATE_OPERATIONAL,
    STATE_FREQUENCY,
    STATE_ERROR,
} system_state_t;

/** @brief System callback. */
typedef void (*system_callback_t)(system_event_t event);

#endif // PROJECT_TYPES_H