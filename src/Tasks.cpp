#include "Tasks.hpp"
#include <RobotDefs.hpp>
#include <FastIO.hpp>
#include <FastMath.hpp>
#include <DebugDefs.hpp>
#include <FeedTheDog.hpp>
#include <Circbuffer.hpp>
#include <IrCommunication.hpp>
#include <Localization.hpp>
#include <iostream>
#include <string>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

using math::fast::square;

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
            std::string temp_str = "CPU " + std::to_string(xPortGetCoreID()) + " usage: " + std::to_string(100.0F - 100.0F * float(execution_count * stat_time_resolution) / stat_update_time) + "%";
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

void IRAM_ATTR Occupy_time_task(void *pvParameters)
{
    // time in ms
    constexpr int64_t t_active = 5;
    constexpr int64_t t_idle = 5;

    while (true)
    {
        int64_t t_start = esp_timer_get_time();
        while (esp_timer_get_time() - t_start < 1000 * t_active)
        {
        }
        vTaskDelay(pdMS_TO_TICKS(t_idle));
    }
}

void LED_off_task(void *pvParameters)
{
    TickType_t prev_wake_time = xTaskGetTickCount();

    while (1)
    {
        // still need to feed the dog
        Feed_the_dog();

        // turn off led if they haven't been refreshed for a while
        if (esp_timer_get_time() - IR::RX::Get_last_RX_time() >= 300)
        {
            // QUENCH_R;
            // QUENCH_G;
            QUENCH_B;
        }

        vTaskDelayUntil(&prev_wake_time, pdMS_TO_TICKS(1));
    }
}

// hex string conversion
template <typename I>
std::string n2hexstr(I w, size_t hex_len = sizeof(I) << 1)
{
    static const char *digits = "0123456789ABCDEF";
    std::string rc(hex_len, '0');
    for (size_t i = 0, j = (hex_len - 1) * 4; i < hex_len; ++i, j -= 4)
        rc[i] = digits[(w >> j) & 0x0f];
    return rc;
}

// void Send_message_task(void *pvParameters)
// {
//     TickType_t prev_wake_time = xTaskGetTickCount();
//
//     while (1)
//     {
//         uint32_t curr_flag;
//
//         uint32_t robot_count;
//         uint32_t idarr[IR::RX::Max_robots_simultaneous];
//         uint32_t time_count;
//         IR::RX::Msg_timing_t tarr[IR::RX::Raw_msg_buffer_size];
//         IR::RX::Parsed_msg_completed msg_1, msg_4;
//
//         bool repeat = 0;
//
//         do
//         {
//             if (repeat)
//                 Serial.println("Oops, interrupted!");
//             else
//                 repeat = 1;
//
//             std::fill_n(idarr, IR::RX::Max_robots_simultaneous, 0);
//
//             curr_flag = IR::RX::Get_io_flag();
//
//             robot_count = IR::RX::Get_neighboring_robots_ID(idarr, 0);
//             msg_1 = IR::RX::Get_latest_msg_by_type(1);
//             msg_4 = IR::RX::Get_latest_msg_by_type(4);
//             time_count = IR::RX::Get_timing_data(tarr);
//         } while (IR::RX::Get_io_flag() != curr_flag);
//
//         std::string temp = "";
//
//         if (robot_count)
//         {
//             // print robot number
//             temp = "\n# Robots: " + std::to_string(robot_count) + "\n There IDs are: ";
//
//             // print robot IDs
//             for (size_t i = 0; i < robot_count; i++)
//             {
//                 temp += std::to_string(idarr[i]) + ",";
//             }
//         }
//         else
//         {
//             temp = "\nNo Robot here! :(";
//         }
//
//         if (msg_1.content_length)
//         {
//             // print latest msg of type 1
//             temp += "\nLatest type 1 message is sent by robot " + std::to_string(msg_1.robot_ID) + " : " + n2hexstr(msg_1.content[0]);
//             temp += "\n    last received at: " + std::to_string(msg_1.finish_reception_time);
//         }
//         else
//         {
//             temp += "\nNo message of type 1 received.";
//         }
//
//         if (msg_4.content_length)
//         {
//             // print first robot's latest msg of type 4
//             temp += "\nLatest type 4 message is sent by robot " + std::to_string(msg_4.robot_ID) + " and has length of: " + std::to_string(msg_4.content_length) + "\n The contents are: ";
//
//             for (size_t i = 0; i < msg_4.content_length; i++)
//             {
//                 temp += n2hexstr(msg_4.content[i]) + ",";
//             }
//
//             temp += "\n    last received at: " + std::to_string(msg_4.finish_reception_time);
//         }
//         else
//         {
//             temp += "\nNo message of type 4 received.";
//         }
//
//         if (time_count)
//         {
//             temp += "\nTiming info: ";
//
//             for (size_t i = 0; i < min(time_count, 10u); i++)
//             {
//                 temp += "\n" + std::to_string(tarr[i].robot_ID) + "@" + std::to_string(tarr[i].emitter_pos) + " : " + std::to_string(tarr[i].time_arr[0]) + "," + std::to_string(tarr[i].time_arr[1]) + "," + std::to_string(tarr[i].time_arr[2]);
//             }
//         }
//         else
//         {
//             temp += "\nNo timing info. :(";
//         }
//
//         temp += "\n\n";
//
//         Serial.print(temp.c_str());
//
//         vTaskDelayUntil(&prev_wake_time, pdMS_TO_TICKS(2000));
//     }
// }

namespace
{
    size_t record_count = 0U;
    constexpr size_t start_record_count = 300U;
    constexpr size_t Position_buffer_max_size = 300U;
    std::vector<IR::Localization::Position_data> Position_buffer;

    void Send_message_task(void *pvParameters)
    {
        while (true)
        {
            vTaskDelay(50);
            if (Serial.available())
            {
                delay(10);
                // deplete serial buffer
                while (Serial.available())
                {
                    Serial.read();
                }

                for (auto &pdat : Position_buffer)
                {
                    std::string v = std::string("x : ") + std::to_string(pdat.x) + std::string(", y : ") + std::to_string(pdat.y) + std::string(", z : ") + std::to_string(pdat.z) + std::string(", var_xy : ") + std::to_string(pdat.var_xy) + std::string(", var_z : ") + std::to_string(pdat.var_z) + std::string(", w : ") + std::to_string(pdat.angular_velocity) + std::string(", err_factor : ") + std::to_string(pdat.mean_error_factor);

                    Serial.println(v.c_str());
                }
            }
        }
    }
}

void Buffer_data_task(void *pvParameters)
{
    // position data is valid for 1s
    constexpr int64_t Position_expire_time = 1000000LL;

    Position_buffer.reserve(Position_buffer_max_size + 1);

    while (true)
    {
        // wait till next localization is done
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        record_count++;

        if (record_count > start_record_count && Position_buffer.size() < Position_buffer_max_size)
        {
            // get data
            uint32_t io_flag;
            IR::Localization::Position_data pos_0;
            bool not_enough_data = false;
            do
            {
                io_flag = IR::Localization::Get_io_flag();

                // check if there's enough data to use
                if (IR::Localization::Get_position_data_count(Position_expire_time) == 0)
                {
                    not_enough_data = true;
                    break;
                }

                pos_0 = IR::Localization::Get_position();

            } while (io_flag != IR::Localization::Get_io_flag());

            if (not_enough_data)
            {
                continue;
            }

            Position_buffer.push_back(pos_0);

            if (Position_buffer.size() == Position_buffer_max_size)
            {
                LIT_G;

                // send me messages through serial!
                xTaskCreatePinnedToCore(
                    Send_message_task,
                    "Send_message_task",
                    20000,
                    NULL,
                    3,
                    NULL,
                    0);
            }
        }
    }
}

// anonymous namespace to hide stuffs
namespace
{
    struct LED_timing_t
    {
        int64_t t_start;
        int64_t duration;
        int64_t period;
    };

    /**
     * @brief a buffer that stores when to turn on and off LED!
     */
    Circbuffer<LED_timing_t, 10> LED_timing_buffer;

    /**
     * @brief trigger timer for LED indicator
     */
    hw_timer_t *LED_trigger_timer;

    /**
     * @brief relative angle of LED
     */
    constexpr float LED_angle_offset = 0.8818719385800353F;

    /**
     * @brief minimum delay time, if lower than this, we trigger after this.
     */
    constexpr int64_t Delay_min_time = 50LL;

    /**
     * @brief last timing used by the interrupt
     */
    LED_timing_t Last_LED_timing;

    /**
     * @brief 0 for off, 1 for on
     *
     * @note initially it should be 0
     */
    bool LED_state = false;

    /**
     * @brief a task that switch on and off LED based on robot's position
     *
     * @note not finished!
     */
    void IRAM_ATTR FB_LED_ISR()
    {
        // how long should we delay before next switch
        uint64_t next_time;

        // this is for indication of XY position feedback.
        // if currently LED is on, then we should turn it off, and also setup the
        // time when it should be on again. Let's use green LED only here.
        if (LED_state)
        {
            QUENCH_R;
            LED_state = 0;

            // get localization data
            LED_timing_t temp = LED_timing_buffer.peek_tail();
            // now this temp is used, update Last_LED_timing
            Last_LED_timing = temp;

            // update time
            next_time = (temp.t_start - (esp_timer_get_time() % temp.period)) % temp.period;
        }
        else
        {
            LIT_R;
            LED_state = 1;
            next_time = Last_LED_timing.duration;
        }

        // not sure if this is necessary, but we will add it anyways, it's not gonna influence anything.
        next_time = (next_time < Delay_min_time) ? Delay_min_time : next_time;
        // reset timer
        timerRestart(LED_trigger_timer);
        timerAlarmWrite(LED_trigger_timer, next_time, false);
        timerAlarmEnable(LED_trigger_timer);
    }
} // anonymous namespace

void LED_control_task(void *pvParameters)
{
    constexpr float K_P = 1.0F;
    // K_D has time unit of s
    constexpr float K_D = 0.0F;
    // K_I has time unit of 1/s
    constexpr float K_I = 0.0F;

    // position data is valid for 1s
    constexpr int64_t Position_expire_time = 1000000LL;

    bool ISR_started = false;

    // integral component
    float I_comp[3] = {0.0F, 0.0F, 0.0F};

    while (true)
    {
        // wait till next localization is done
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // get data
        uint32_t io_flag;
        IR::Localization::Position_data pos_0, pos_1;
        bool not_enough_data = false;
        do
        {
            io_flag = IR::Localization::Get_io_flag();

            // check if there's enough data to use
            if (IR::Localization::Get_position_data_count(Position_expire_time) < 2)
            {
                not_enough_data = true;
                break;
            }

            pos_0 = IR::Localization::Get_filtered_position(0);
            pos_1 = IR::Localization::Get_filtered_position(1);

        } while (io_flag != IR::Localization::Get_io_flag());

        // if not, just wait!
        if (not_enough_data)
        {
            continue;
        }

        // compute PID
        float P_comp[3], D_comp[3];
        // actual PID feedback value
        float FB_val[3];

        P_comp[0] = pos_0.x;
        P_comp[1] = pos_0.y;
        P_comp[2] = pos_0.z;

        float t_coef = float(pos_0.time - pos_1.time) / 1000000.0F;

        I_comp[0] += t_coef * pos_0.x;
        I_comp[1] += t_coef * pos_0.y;
        I_comp[2] += t_coef * pos_0.z;

        t_coef = 1000000.0F / float(pos_0.time - pos_1.time);

        D_comp[0] = t_coef * (pos_0.x - pos_1.x);
        D_comp[1] = t_coef * (pos_0.y - pos_1.y);
        D_comp[2] = t_coef * (pos_0.z - pos_1.z);

        for (size_t i = 0; i < 3; i++)
        {
            FB_val[i] = K_P * P_comp[i] + K_I * I_comp[i] + K_D * D_comp[i];
        }

        // compute timing
        LED_timing_t temp;

        temp.duration = int64_t(0.005F * sqrtf(square(FB_val[0]) + square(FB_val[1])) / pos_0.angular_velocity);
        temp.t_start = pos_0.rotation_time * 5 - temp.duration / 2 - int64_t((LED_angle_offset + pos_0.angle_0 - atan2f(FB_val[1], FB_val[0])) / pos_0.angular_velocity);
        temp.period = pos_0.rotation_time;

        constexpr int64_t Duration_max_time = 20000;
        // cap the duration!
        temp.duration = (temp.duration > Duration_max_time) ? Duration_max_time : temp.duration;

        // push inside buffer
        LED_timing_buffer.push(temp);

        // check if first time, if so, start ISR
        if (!ISR_started)
        {
            ISR_started = true;

            // last led timing is this!
            Last_LED_timing = temp;

            LED_trigger_timer = timerBegin(2, 80, true);
            // add timer interrupt
            timerAttachInterrupt(LED_trigger_timer, &FB_LED_ISR, true);
        }

        // updating timer info
        int64_t t_delay = (temp.t_start - (esp_timer_get_time() % temp.period)) % temp.period;

        // not sure if this is necessary, but we will add it anyways, it's not
        // gonna influence anything.
        t_delay = (t_delay < Delay_min_time) ? Delay_min_time : t_delay;

        if (LED_state == false)
        {
            // how long before/after the trigger time should we prevent triggering in us
            constexpr uint64_t No_operation_time = 500ULL;
            // make sure that we won't reset the timer just after LED turns on.
            // also make sure that we won't reset the timer just before LED
            // turns on.
            if (timerRead(LED_trigger_timer) > No_operation_time || timerAlarmRead(LED_trigger_timer) < timerRead(LED_trigger_timer) + No_operation_time)
            {
                timerAlarmDisable(LED_trigger_timer);
                timerRestart(LED_trigger_timer);
                timerAlarmWrite(LED_trigger_timer, t_delay, false);
                timerAlarmEnable(LED_trigger_timer);
            }
        }

        // QUENCH_G;
    }
}