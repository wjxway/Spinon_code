#ifndef _TASKS_HPP_
#define _TASKS_HPP_
#include "Arduino.h"

/**
 * @brief an idle task to monitor free time on CPU.
 *
 * @note please put it at the second lowest priority (1)
 */
void IRAM_ATTR Idle_stats_task(void *pvParameters);

/**
 * @brief a task that takes away some time and do nothing.
 */
void IRAM_ATTR Occupy_time_task(void *pvParameters);

#endif