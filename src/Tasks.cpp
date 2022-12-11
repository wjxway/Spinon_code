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
            QUENCH_R;
            QUENCH_G;
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

/**
 * @brief trigger timer for LED indicator
 */
hw_timer_t *LED_trigger_timer;

/**
 * @brief relative angle of LED
 */
constexpr float LED_angle_offset = 0.8818719385800353F;

// /**
//  * @brief a task that switch on and off LED based on robot's position
//  *
//  * @note not finished!
//  */
// void IRAM_ATTR FB_LED_ISR()
// {
//     uint32_t cp0_regs[18];
//     // get FPU state
//     uint32_t cp_state = xthal_get_cpenable();

//     if (cp_state)
//     {
//         // Save FPU registers
//         xthal_save_cp0(cp0_regs);
//     }
//     else
//     {
//         // enable FPU
//         xthal_set_cpenable(1);
//     }

//     // 0 for now off, 1 for now on
//     // initially it should be 1, so we can setup the initial timing
//     static bool LED_state = 1;

//     // how long should we delay before next on
//     uint64_t delay_time;
//     // how long should next time be on
//     static uint64_t on_time = 0;

//     // this is for indication of XY position feedback.
//     // if currently LED is on, then we should turn it off, and also setup the
//     // time when it should be on again. Let's use green LED only here.
//     if (LED_state)
//     {
//         QUENCH_G;
//         LED_state = 0;

//         // get localization data
//         Position_data res = Position_stack.peek_tail();
//         on_time = 0.01F * sqrtf(square(res.x) + square(res.y)) / res.angular_velocity;
//         // determine next time
//         delay_time = (res.rotation_time * 3 - int64_t(on_time / 2.0F + (LED_angle_offset + res.angle_0 - atan2f(res.y, res.x)) / res.angular_velocity) - (esp_timer_get_time() % res.rotation_time)) % res.rotation_time;
//     }
//     else
//     {
//         LIT_G;
//         LED_state = 1;
//         // we always lit LED for 1ms.
//         delay_time = on_time;
//     }

//     // // this is for indication of X and Y direction.
//     // // if currently LED is on, then we should turn it off, and also setup the
//     // // time when it should be on again. Let's use green LED only here.
//     // if (LED_state)
//     // {
//     //     QUENCH_G;
//     //     LED_state = 0;

//     //     // get localization data
//     //     Position_data res = Position_stack.peek_tail();
//     //     on_time = int64_t(float(M_PI_2) / res.angular_velocity);
//     //     // determine next time
//     //     delay_time = (res.rotation_time * 3 - int64_t((LED_angle_offset + res.angle_0) / res.angular_velocity) - (esp_timer_get_time() % res.rotation_time)) % res.rotation_time;
//     // }
//     // else
//     // {
//     //     LIT_G;
//     //     LED_state = 1;
//     //     // we always lit LED for 1ms.
//     //     delay_time = on_time;
//     // }

//     // reset timer
//     timerRestart(LED_trigger_timer);
//     timerAlarmWrite(LED_trigger_timer, delay_time, false);
//     timerAlarmEnable(LED_trigger_timer);

//     if (cp_state)
//     {
//         // Restore FPU registers
//         xthal_restore_cp0(cp0_regs);
//     }
//     else
//     {
//         // turn it back off
//         xthal_set_cpenable(0);
//     }
// }




// do something with the res, add it to a queue or something else.
// we can setup the interrupt here and let it turn on/off the lights.

// // indicate when position of robot is computed
// LIT_R;
// delayMicroseconds(1000);
// QUENCH_R;

// static bool FB_LED_ISR_started = 0;

// if (!FB_LED_ISR_started)
// {
//     FB_LED_ISR_started = 1;

//     // config timer to transmit TX once in a while
//     // we are using timer 1 to prevent confiction
//     // timer ticks 1MHz
//     LED_trigger_timer = timerBegin(1, 80, true);
//     // add timer interrupt
//     timerAttachInterrupt(LED_trigger_timer, &FB_LED_ISR, true);
//     timerAlarmWrite(LED_trigger_timer, 10000UL, false);
//     timerAlarmEnable(LED_trigger_timer);
// }

// // store and print out the data later.
// // for now we end pushing after we have > 400 elements
// // and we setup the LED to be always on to indicate that the data has been filled.
// if (Position_stack.n_elem < 295)
// {
//     LIT_R;
//     LIT_G;
//     LIT_B;
//     delayMicroseconds(500);
//     QUENCH_R;
//     QUENCH_G;
//     QUENCH_B;
//
//     All_PD tmp;
//     tmp.rel_pos_dat[0] = Loc_data[0];
//     tmp.rel_pos_dat[1] = Loc_data[1];
//     tmp.rel_pos_dat[2] = Loc_data[2];
//     tmp.pos_dat = res;
//
//     Position_stack.push(tmp);
// }
// // if just finished reception
// else if (Data_finished == 0)
// {
//     LIT_R;
//     LIT_G;
//     LIT_B;
//     delayMicroseconds(10000);
//     QUENCH_R;
//     QUENCH_G;
//     QUENCH_B;
//
//     Data_finished = 1;
//     xTaskCreatePinnedToCore(
//         Print_data_task,
//         "Print_data_task",
//         20000,
//         NULL,
//         15,
//         NULL,
//         0);
// }