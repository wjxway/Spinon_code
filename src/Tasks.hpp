/**
 * @file Tasks.hpp
 * @brief User defined tasks to run
 */
#ifndef TASKS_HPP__
#define TASKS_HPP__

#include "Arduino.h"

extern float target_point[3];

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
 * @brief buffer EKF data
 */
void Buffer_EKF_task(void *pvParameters);

/**
 * @brief buffer localization data
 */
void Buffer_data_task(void *pvParameters);

/**
 * @brief buffer raw timing data
 */
void Buffer_raw_data_task(void *pvParameters);

/**
 * @brief initialize LED PWM using LEDC
 * 
 * @param channels bit 0,1,2 for R,G,B
 */
void LED_PWM_init(uint32_t channels);

/**
 * @brief convenience function to set LED intensity
 * 
 * @param color 0,1,2 for R,G,B
 * @param duty 0~1 for intensity (duty)
 */
void LED_set(uint32_t color,float duty);

/**
 * @brief similar to LED_control task, but actually turns the motor.
 * 
 * @param pvParameters 
 */
void Motor_control_task_opt(void *pvParameters);

/**
 * @brief similar to LED_control task, but actually turns the motor.
 * 
 * @param pvParameters 
 */
void Motor_control_task_EKF(void *pvParameters);

/**
 * @brief a task that monitors update of motor control commands and slower /
 * turn off the motor for safety.
 * 
 * @param pvParameters 
 */
void Motor_monitor_task(void *pvParameters);


/**
 * @brief test motor thrust - speed curve
 * 
 * @param pvParameters 
 */
void Motor_test_task(void *pvParameters);

/**
 * @brief print out position messages received
 * 
 * @param pvParameters 
 */
void Message_relay_task(void *pvParameters);

#endif