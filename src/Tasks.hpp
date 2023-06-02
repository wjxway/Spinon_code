/**
 * @file Tasks.hpp
 * @brief User defined tasks to run
 */
#ifndef TASKS_HPP__
#define TASKS_HPP__

#include "Arduino.h"

/**
 * @brief turn off LED!
 * 
 * @param pvParameters 
 */
void LED_off_task(void *pvParameters);

/**
 * @brief buffer raw timing data
 */
void Buffer_raw_data_task(void *pvParameters);

void Relay_task(void *pvParameters);

#endif