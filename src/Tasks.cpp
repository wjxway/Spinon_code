#include "Tasks.hpp"
#include "RobotDefs.hpp"
#include "Utilities/FastIO.hpp"
#include "Utilities/DebugDefs.hpp"
#include "Utilities/FeedTheDog.hpp"
#include "IrCommunication/IrRX.hpp"
// #include "MotorCtrl/MotorCtrl.hpp"
#include <iostream>
#include <string>
#include <vector>

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

template <typename I>
std::string n2hexstr(I w, size_t hex_len = sizeof(I) << 1)
{
    static const char *digits = "0123456789ABCDEF";
    std::string rc(hex_len, '0');
    for (size_t i = 0, j = (hex_len - 1) * 4; i < hex_len; ++i, j -= 4)
        rc[i] = digits[(w >> j) & 0x0f];
    return rc;
}

void IRAM_ATTR Send_message_task(void *pvParameters)
{
    TickType_t prev_wake_time = xTaskGetTickCount();

    while (1)
    {
        uint32_t curr_flag;

        uint32_t robot_count;
        uint32_t idarr[IR::RX::Max_robots_simultaneous];
        uint32_t time_count;
        IR::RX::Msg_timing_t tarr[IR::RX::Raw_msg_buffer_size];
        IR::RX::Parsed_msg_completed msg_1, msg_4;

        bool repeat = 0;

        do
        {
            if (repeat)
                Serial.println("Oops, interrupted!");
            else
                repeat = 1;

            std::fill_n(idarr, IR::RX::Max_robots_simultaneous, 0);

            curr_flag = IR::RX::Get_io_flag();

            robot_count = IR::RX::Get_neighboring_robots_ID(idarr, 0);
            msg_1 = IR::RX::Get_latest_msg_by_bot(idarr[0], 1);
            msg_4 = IR::RX::Get_latest_msg_by_bot(idarr[0], 4);
            time_count = IR::RX::Get_timing_data(tarr);
        } while (IR::RX::Get_io_flag() != curr_flag);

        std::string temp = "";

        if (robot_count)
        {
            // print robot number
            temp = "\n# Robots: " + std::to_string(robot_count) + "\n There IDs are: ";

            // print robot IDs
            for (size_t i = 0; i < robot_count; i++)
            {
                temp += std::to_string(idarr[i]) + ",";
            }
        }
        else
        {
            temp = "\nNo Robot here! :(";
        }

        if (msg_1.content_length)
        {
            // print first robot's latest msg of type 1
            temp += "\nFirst robot's latest type 1 message is: " + n2hexstr(msg_1.content[0]);
            temp += "\n    last received at: " + std::to_string(msg_1.finish_reception_time);
        }
        else
        {
            temp += "\nNo message of type 1 received.";
        }

        if (msg_4.content_length)
        {
            // print first robot's latest msg of type 4
            temp += "\nFirst robot's latest type 4 message has length of: " + std::to_string(msg_4.content_length) + "\n The contents are: ";

            for (size_t i = 0; i < msg_4.content_length; i++)
            {
                temp += n2hexstr(msg_4.content[i]) + ",";
            }

            temp += "\n    last received at: " + std::to_string(msg_4.finish_reception_time);
        }
        else
        {
            temp += "\nNo message of type 4 received.";
        }

        if (time_count)
        {
            temp += "\nTiming info: ";

            for (size_t i = 0; i < min(time_count, 10u); i++)
            {
                temp += "\n" + std::to_string(tarr[i].time_arr[0]) + "," + std::to_string(tarr[i].time_arr[1]) + "," + std::to_string(tarr[i].time_arr[2]);
            }
        }
        else
        {
            temp += "\nNo timing info. :(";
        }

        temp += "\n\n";

        Serial.print(temp.c_str());

        vTaskDelayUntil(&prev_wake_time, pdMS_TO_TICKS(2000));
    }
}

void IRAM_ATTR Localization_simple(void *pvParameters)
{
    TickType_t prev_wake_time = xTaskGetTickCount();
    while (1)
    {
        uint32_t curr_flag;

        // neighboring robots
        uint32_t robot_count;
        uint32_t robot_ID_list[IR::RX::Max_robots_simultaneous];

        // neighboring robots' position
        std::vector<IR::RX::Parsed_msg_completed> pos_list;

        // timing info
        uint32_t time_count;
        IR::RX::Msg_timing_t time_list[IR::RX::Raw_msg_buffer_size];

        // get data
        do
        {
            curr_flag = IR::RX::Get_io_flag();

            robot_count = IR::RX::Get_neighboring_robots_ID(robot_ID_list, 0);

            pos_list.reserve(robot_count);
            for (size_t i = 0; i < robot_count; i++)
            {
                pos_list.push_back(IR::RX::Get_latest_msg_by_bot(robot_ID_list[i], 4));
            }

            time_count = IR::RX::Get_timing_data(time_list);
        } while (curr_flag != IR::RX::Get_io_flag());

        // processing

        vTaskDelayUntil(&prev_wake_time, pdMS_TO_TICKS(100));
    }
}