/**
 * @file Tasks.hpp
 * @brief User defined tasks to run
 */
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

/**
 * @brief turn off LED!
 * 
 * @param pvParameters 
 */
void LED_off_task(void *pvParameters);

/**
 * @brief send me some messages!
 */
void Send_message_task(void *pvParameters);

/**
 * @brief try localization (a fairly simple and messy solution)
 */
void Simple_localization_task(void *pvParameters);

#endif