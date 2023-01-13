/**
 * @file Tasks.hpp
 * @brief User defined tasks to run
 */
#ifndef TASKS_HPP__
#define TASKS_HPP__

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
void Buffer_data_task(void *pvParameters);

/**
 * @brief initialize LED PWM using LEDC
 */
void LED_PWM_init();

/**
 * @brief convenience function to set LED intensity
 * 
 * @param color 0,1,2 for R,G,B
 * @param duty 0~1 for intensity (duty)
 */
void LED_set(uint32_t color,float duty);

/**
 * @brief convert localization information to LED states and manages LED_FB_ISR
 * 
 * @param pvParameters 
 */
void LED_control_task(void *pvParameters);

/**
 * @brief similar to LED_control task, but actually turns the motor.
 * 
 * @param pvParameters 
 */
void Motor_control_task(void *pvParameters);

#endif