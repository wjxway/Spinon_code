#ifndef _TASKS_HPP_
#define _TASKS_HPP_
#pragma once
#include "Arduino.h"
// #include "SPI.h"

// /**
//  * @brief SPI for data communication
//  */
// extern SPIClass *hspi;

// /**
//  * @brief a mutex to allow only one task to use spi at a time
//  */
// extern xSemaphoreHandle HSPI_MUTEX;

/**
 * @brief Task that process timing information
 */
void Time_comm_task(void *pvParameters);

/**
 * @brief Task that process data
 */
void Data_comm_task(void *pvParameters);

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