#ifndef _TASKS_HPP_
#define _TASKS_HPP_
#pragma once
#include "Arduino.h"
#include "SPI.h"

/**
 * @brief SPI for data communication
 */
extern SPIClass *hspi;

/**
 * @brief a mutex to allow only one task to use spi at a time
 */
extern xSemaphoreHandle HSPI_MUTEX;

/**
 * @brief Task that process timing information
 */
void Time_comm_task(void *pvParameters);

/**
 * @brief Task that process data
 */
void Data_comm_task(void *pvParameters);


#endif