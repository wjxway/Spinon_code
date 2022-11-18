#include "Tasks.hpp"
#include "RobotDefs.hpp"
#include "Utilities/FastIO.hpp"
#include "Utilities/DebugDefs.hpp"
#include "Utilities/FeedTheDog.hpp"
#include "MotorCtrl.hpp"
#include <iostream>
#include <string>

void IRAM_ATTR Idle_stats_task(void *pvParameters)
{
    // stats resolution in CPU cycles
    // 120 ticks means resolution of 0.5us
    constexpr uint32_t stat_time_resolution = 120;

    // stats update interval in CPU cycles
    // 48,000,000 means an update interval of 0.5s
    constexpr uint32_t stat_update_time = 120000000;

    // please make sure that this is a multiple of stat_time_resolution
    static_assert(stat_update_time % stat_time_resolution == 0);

    // idle task execution cycle count
    // +1 means stat_time_resolution ticks has passed.
    uint32_t execution_count = 0;

    // current time
    uint32_t curr_time = cpu_hal_get_cycle_count();

    // previous CPU ticks count
    uint32_t prev_time = curr_time;

    // startup CPU ticks count
    uint32_t startup_time = curr_time;

    // the real monitor task
    // it should never quit!
    while (true)
    {
        while (cpu_hal_get_cycle_count() - prev_time < stat_time_resolution)
        {
        }

        // another cycle has been evaluated!
        execution_count++;

        curr_time = cpu_hal_get_cycle_count();
        // if this time something happened in between, then start from now.
        if (curr_time - prev_time > 2 * stat_time_resolution)
            prev_time = curr_time;
        // if not, for precision add stat_time_resolution to ignore all excessive execution time.
        else
            prev_time += stat_time_resolution;

        // if exceeding the update time, update all data
        if (curr_time - startup_time > stat_update_time)
        {
            // debug print or something else
            std::string temp_str = "CPU " + std::to_string(xPortGetCoreID()) + " usage: " + std::to_string(100.0f - 100.0f * float(execution_count * stat_time_resolution) / stat_update_time) + "%";
            DEBUG_C(Serial.println(temp_str.c_str()));

            // feed the dog because idle task will never be invoked
            Feed_the_dog();

            // reset
            execution_count = 0;
            curr_time = cpu_hal_get_cycle_count();
            prev_time = curr_time;
            startup_time = curr_time;
        }
    }
}

/**
 * @brief a stupid task that just keep the CPU running with no purpose once in a while
 *
 * @param pvParameters
 */
void IRAM_ATTR Occupy_time_task(void *pvParameters)
{
    // time in ms
    constexpr uint32_t t_active = 5;
    constexpr uint32_t t_idle = 5;

    uint32_t t_start = micros();

    while (true)
    {
        t_start = micros();
        while (micros() - t_start < 1000 * t_active)
        {
        }
        vTaskDelay(pdMS_TO_TICKS(t_idle));
    }
}
