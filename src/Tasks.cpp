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
#include <esp_timer.h>

#include <MotorCtrl.hpp>

#define _USE_MATH_DEFINES
#include <cmath>

using math::fast::clip;
using math::fast::norm;
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
                    std::string v = std::string("t : ") + std::to_string(pdat.time) + std::string(", x : ") + std::to_string(pdat.x) + std::string(", y : ") + std::to_string(pdat.y) + std::string(", z : ") + std::to_string(pdat.z) + std::string(", var_xy : ") + std::to_string(pdat.var_xy) + std::string(", var_z : ") + std::to_string(pdat.var_z) + std::string(", w : ") + std::to_string(pdat.angular_velocity) + std::string(", err_factor : ") + std::to_string(pdat.mean_error_factor) + "\n";

                    Serial.print(v.c_str());
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

                // pos_0 = IR::Localization::Get_position();
                pos_0 = IR::Localization::Get_filtered_position();
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

namespace
{
    // R,G,B LED corresponds to offset + 0,1,2
    constexpr uint32_t LED_LEDC_channel_offset = 1U;

    // LEDC frequency for LED
    constexpr uint32_t LED_PWM_frequency = 40000U;

    // LEDC resolution for LED
    constexpr uint32_t LED_PWM_resolution = 10U;
} // anonymous namespace

void LED_PWM_init()
{
    ledcSetup(LED_LEDC_channel_offset, LED_PWM_frequency, LED_PWM_resolution);
    ledcAttachPin(LED_PIN_R, LED_LEDC_channel_offset);
    ledcWrite(LED_LEDC_channel_offset, (1 << LED_PWM_resolution) - 1);

    ledcSetup(LED_LEDC_channel_offset + 1, LED_PWM_frequency, LED_PWM_resolution);
    ledcAttachPin(LED_PIN_G, LED_LEDC_channel_offset + 1);
    ledcWrite(LED_LEDC_channel_offset + 1, (1 << LED_PWM_resolution) - 1);

    ledcSetup(LED_LEDC_channel_offset + 2, LED_PWM_frequency, LED_PWM_resolution);
    ledcAttachPin(LED_PIN_B, LED_LEDC_channel_offset + 2);
    ledcWrite(LED_LEDC_channel_offset + 2, (1 << LED_PWM_resolution) - 1);
}

void LED_set(uint32_t color, float duty)
{
    ledcWrite(LED_LEDC_channel_offset + color, uint32_t((1 - duty) * float((1 << LED_PWM_resolution) - 1)));
}

// anonymous namespace to hide stuffs
namespace
{
    struct LED_timing_t
    {
        int64_t t_start;
        int64_t duration;
        int64_t period;
        float intensity_on = 1.0F;
        float intensity_off = 0.0F;
    };

    /**
     * @brief relative angle of LED
     */
    constexpr float LED_angle_offset = 0.8818719385800353F;

    /**
     * @brief minimum delay time, if lower than this, we trigger after this.
     */
    constexpr int64_t Delay_min_time = 50LL;

    struct LED_callback_args
    {
        bool LED_state;                      // current state of LED, 0 (initial state) for off, 1 for on
        LED_timing_t Last_LED_timing;        // last timing info used by the callback
        int64_t Last_trig_time;              // last time we updated the trigger info
        int64_t Next_trig_time;              // when will we trig next time
        int LED_num;                         // 0,1,2 for R,G,B respectively
        Circbuffer<LED_timing_t, 5> *buffer; // timing info buffer
        esp_timer_handle_t handle;           // callback handler
    };

    LED_callback_args callback_args_1, callback_args_2, callback_args_3;
    Circbuffer<LED_timing_t, 5> timing_buf_1, timing_buf_2, timing_buf_3;

    /**
     * @brief a task that switch on and off LED based on robot's position
     *
     * @param void* v_arg arguments
     */
    void IRAM_ATTR LED_callback(void *v_arg)
    {
        uint64_t t_now = esp_timer_get_time();

        LED_callback_args *arg = (LED_callback_args *)v_arg;

        // how long should we delay before next switch
        uint64_t next_time;

        // this is for indication of XY position feedback.
        // if currently LED is on, then we should turn it off, and also setup the
        // time when it should be on again. Let's use green LED only here.
        if (arg->LED_state)
        {
            LED_set(arg->LED_num, arg->Last_LED_timing.intensity_off);
            arg->LED_state = false;

            // get localization data
            LED_timing_t temp = arg->buffer->peek_tail();
            // now this temp is used, update Last_LED_timing
            arg->Last_LED_timing = temp;

            // update time
            // make sure that we won't trigger again in 1/4 cycles
            next_time = ((temp.t_start - (t_now % temp.period) - (temp.period >> 2)) % temp.period) + (temp.period >> 2);
        }
        else
        {
            LED_set(arg->LED_num, arg->Last_LED_timing.intensity_on);
            arg->LED_state = true;
            next_time = arg->Last_LED_timing.duration;
        }

        // not sure if this is necessary, but we will add it anyways, it's not gonna influence anything.
        next_time = (next_time < Delay_min_time) ? Delay_min_time : next_time;
        // reset timer
        arg->Last_trig_time = t_now;
        arg->Next_trig_time = t_now + next_time;
        esp_timer_stop(arg->handle);
        esp_timer_start_once(arg->handle, next_time);
    }

} // anonymous namespace

void LED_control_task(void *pvParameters)
{
    constexpr float K_P_XY = 1.0e-3F;
    constexpr float K_P_Z = 1.0e-3F;
    // K_D has time unit of s
    constexpr float K_D_XY = 0.0F;
    constexpr float K_D_Z = 0.0F;
    // // K_I has time unit of 1/s
    // constexpr float K_I = 0.0F;
    // K_A has time unit of s^2
    constexpr float K_A_XY = 0.0F;
    // gravity compensation
    constexpr float Gravity_compensation = 0.5F;

    // position data is valid for 1s
    constexpr int64_t Position_expire_time = 1000000LL;

    bool ISR_started = false;

    // PID components
    float I_comp[3] = {0.0F, 0.0F, 0.0F};
    float D_comp[3] = {0.0F, 0.0F, 0.0F};
    // variance of D_comp
    float D_xy_var = 1.0e6F, D_z_var = 1.0e6F;
    int64_t D_last_update_time = 0;
    float P_comp[3], FB_val[3];

    // acceleration component
    float A_comp[3] = {0.0F, 0.0F, 0.0F};

    while (true)
    {
        // wait till next localization is done
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // LIT_G;

        // get data
        uint32_t io_flag;
        IR::Localization::Position_data filt_pos_0, filt_pos_1, pos_0, pos_1;
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

            // use unfiltered position for D/I terms
            pos_0 = IR::Localization::Get_position(0);
            pos_1 = IR::Localization::Get_position(1);

            // use filtered position for P terms
            filt_pos_0 = IR::Localization::Get_filtered_position(0);
            filt_pos_1 = IR::Localization::Get_filtered_position(1);

        } while (io_flag != IR::Localization::Get_io_flag());

        // if not, just wait!
        if (not_enough_data)
        {
            // QUENCH_G;
            continue;
        }

        P_comp[0] = filt_pos_0.x;
        P_comp[1] = filt_pos_0.y;
        P_comp[2] = filt_pos_0.z;

        float t_coef = float(pos_0.time - pos_1.time) / 2.0e6F;

        I_comp[0] += t_coef * (pos_0.x + pos_1.x);
        I_comp[1] += t_coef * (pos_0.y + pos_1.y);
        I_comp[2] += t_coef * (pos_0.z + pos_1.z);

        t_coef = 1.0e6F / float(pos_0.time - pos_1.time);

        // store old D_component for acceleration computation
        float old_D_comp[3] = {D_comp[0], D_comp[1], D_comp[2]};

        // use kalmann filter to get new D component
        constexpr float D_error_increase_rate = 500e-6F;
        float D_xy_var_this = t_coef * t_coef * (pos_0.var_xy + pos_1.var_xy);
        float D_z_var_this = t_coef * t_coef * (pos_0.var_z + pos_1.var_z);
        float var_drift = square(float(pos_0.time - D_last_update_time) * D_error_increase_rate);

        // determine z and var_z
        float tmpv = 1.0F / (D_z_var_this + D_z_var + var_drift);
        D_comp[2] = (D_comp[2] * D_z_var_this + (t_coef * (pos_0.z - pos_1.z)) * (D_z_var + var_drift)) * tmpv;
        D_z_var = D_z_var_this * (D_z_var + var_drift) * tmpv;

        // similar for xy and var_xy
        tmpv = 1.0F / (D_xy_var_this + D_xy_var + var_drift);
        D_comp[0] = (D_comp[0] * D_xy_var_this + (t_coef * (pos_0.x - pos_1.x)) * (D_xy_var + var_drift)) * tmpv;
        D_comp[1] = (D_comp[1] * D_xy_var_this + (t_coef * (pos_0.y - pos_1.y)) * (D_xy_var + var_drift)) * tmpv;
        D_xy_var = D_xy_var_this * (D_xy_var + var_drift) * tmpv;

        // we take the simplest approach to acceleration computation
        // if we want to use any more advanced methods, we might as well use
        // Kalman filter in the first place we executing localization.
        // Now it's just a very trivial exponential filter

        // update coefficient, choose the time coefficient to be approximately
        // averaging over 0.5s
        float update_coef = clip(4e-6F * float(pos_0.time - D_last_update_time), 0.0F, 1.0F);

        A_comp[0] = (1.0 - update_coef) * A_comp[0] + update_coef * 1.0e6F * (D_comp[0] - old_D_comp[0]) / float(pos_0.time - D_last_update_time);
        A_comp[1] = (1.0 - update_coef) * A_comp[1] + update_coef * 1.0e6F * (D_comp[1] - old_D_comp[1]) / float(pos_0.time - D_last_update_time);
        A_comp[2] = (1.0 - update_coef) * A_comp[2] + update_coef * 1.0e6F * (D_comp[2] - old_D_comp[2]) / float(pos_0.time - D_last_update_time);

        D_last_update_time = pos_0.time;

        FB_val[0] = -K_P_XY * P_comp[0] - K_D_XY * D_comp[0] - K_A_XY * A_comp[0];
        FB_val[1] = -K_P_XY * P_comp[1] - K_D_XY * D_comp[1] - K_A_XY * A_comp[1];
        FB_val[2] = -K_P_Z * P_comp[2] - K_D_Z * D_comp[2] + Gravity_compensation;

        // for (size_t i = 0; i < 3; i++)
        // {
        //     FB_val[i] = K_P * P_comp[i] + K_I * I_comp[i] + K_D * D_comp[i];
        // }

        // all computation has been finished till now
        // from now on it's the boring setup interrupt part

        // cap the duration!
        constexpr int64_t Duration_max_time = 25000;

        // compute timing for callback_1 that use LED to indicate position
        LED_timing_t Callback_dat_1;

        Callback_dat_1.duration = clip(int64_t(0.005F * norm(P_comp[0], P_comp[1]) / pos_0.angular_velocity), 0LL, Duration_max_time);
        Callback_dat_1.t_start = pos_0.rotation_time * 5 - Callback_dat_1.duration / 2 - int64_t((LED_angle_offset + pos_0.angle_0 - atan2f(P_comp[1], P_comp[0])) / pos_0.angular_velocity);
        Callback_dat_1.period = pos_0.rotation_time;

        // push inside buffer
        timing_buf_1.push(Callback_dat_1);

        // compute timing for callback_2 that use LED to indicate velocity
        LED_timing_t Callback_dat_2;

        Callback_dat_2.duration = clip(int64_t(0.005F * norm(D_comp[0], D_comp[1]) / pos_0.angular_velocity), 0LL, Duration_max_time);
        Callback_dat_2.t_start = pos_0.rotation_time * 5 - Callback_dat_2.duration / 2 - int64_t((LED_angle_offset + pos_0.angle_0 - atan2f(D_comp[1], D_comp[0])) / pos_0.angular_velocity);
        Callback_dat_2.period = pos_0.rotation_time;

        // push inside buffer
        timing_buf_2.push(Callback_dat_2);

        // // compute timing for callback_3 that use LED to indicate acceleration
        // LED_timing_t Callback_dat_3;

        // Callback_dat_3.duration = clip(int64_t(0.005F * norm(A_comp[0], A_comp[1]) / pos_0.angular_velocity), 0LL, Duration_max_time);
        // Callback_dat_3.t_start = pos_0.rotation_time * 5 - Callback_dat_3.duration / 2 - int64_t((LED_angle_offset + pos_0.angle_0 - atan2f(A_comp[1], A_comp[0])) / pos_0.angular_velocity);
        // Callback_dat_3.period = pos_0.rotation_time;

        // // push inside buffer
        // timing_buf_3.push(Callback_dat_3);

        // compute timing for callback_3 that use LED to indicate feedback value
        LED_timing_t Callback_dat_3;

        Callback_dat_3.duration = pos_0.rotation_time / 2;
        Callback_dat_3.t_start = pos_0.rotation_time * 5 - Callback_dat_3.duration / 2 - int64_t((LED_angle_offset + pos_0.angle_0 - atan2f(FB_val[1], FB_val[0])) / pos_0.angular_velocity);
        Callback_dat_3.period = pos_0.rotation_time;

        float add_val = norm(FB_val[0], FB_val[1]);
        if (FB_val[2] > 0.5F)
        {
            if (FB_val[2] + add_val > 1.0F)
            {
                add_val = 1.0F - FB_val[2];
            }
        }
        else
        {
            if (FB_val[2] < add_val)
            {
                add_val = FB_val[2];
            }
        }
        Callback_dat_3.intensity_off = FB_val[2] - add_val;
        Callback_dat_3.intensity_on = FB_val[2] + add_val;

        // push inside buffer
        timing_buf_3.push(Callback_dat_3);

        // check if first time, if so, start ISR
        if (!ISR_started)
        {
            ISR_started = true;

            // Callback/ISR 1
            callback_args_1.buffer = &timing_buf_1;
            callback_args_1.Last_LED_timing = Callback_dat_1;
            callback_args_1.LED_num = 0;
            callback_args_1.LED_state = false;
            callback_args_1.Last_trig_time = 0;
            callback_args_1.Next_trig_time = 0;

            const esp_timer_create_args_t oneshot_timer_callback_args_1 = {
                .callback = &LED_callback,
                /* argument specified here will be passed to timer callback function */
                .arg = (void *)(&callback_args_1),
                .dispatch_method = ESP_TIMER_TASK,
                .name = "position"};

            esp_timer_create(&oneshot_timer_callback_args_1, &(callback_args_1.handle));

            // Callback/ISR 2
            callback_args_2.buffer = &timing_buf_2;
            callback_args_2.Last_LED_timing = Callback_dat_2;
            callback_args_2.LED_num = 1;
            callback_args_2.LED_state = false;
            callback_args_2.Last_trig_time = 0;
            callback_args_2.Next_trig_time = 0;

            const esp_timer_create_args_t oneshot_timer_callback_args_2 = {
                .callback = &LED_callback,
                /* argument specified here will be passed to timer callback function */
                .arg = (void *)(&callback_args_2),
                .dispatch_method = ESP_TIMER_TASK,
                .name = "velocity"};

            esp_timer_create(&oneshot_timer_callback_args_2, &(callback_args_2.handle));

            // Callback/ISR 3
            callback_args_3.buffer = &timing_buf_3;
            callback_args_3.Last_LED_timing = Callback_dat_3;
            callback_args_3.LED_num = 2;
            callback_args_3.LED_state = false;
            callback_args_3.Last_trig_time = 0;
            callback_args_3.Next_trig_time = 0;

            const esp_timer_create_args_t oneshot_timer_callback_args_3 = {
                .callback = &LED_callback,
                /* argument specified here will be passed to timer callback function */
                .arg = (void *)(&callback_args_3),
                .dispatch_method = ESP_TIMER_TASK,
                .name = "acceleration"};

            esp_timer_create(&oneshot_timer_callback_args_3, &(callback_args_3.handle));
        }

        // how long before/after the trigger time should we prevent triggering
        // in us
        constexpr uint64_t No_operation_time = 200ULL;

        // updating timer info
        // not sure if clipping is necessary, but we will add it anyways,
        // it's not gonna influence anything.
        int64_t t_delay = clip((Callback_dat_1.t_start - (esp_timer_get_time() % Callback_dat_1.period)) % Callback_dat_1.period, Delay_min_time, 1000000LL);

        if (callback_args_1.LED_state == false)
        {
            int64_t t_now = esp_timer_get_time();
            // make sure that we won't reset the timer just after LED turns on.
            // also make sure that we won't reset the timer just before LED
            // turns on.
            if (t_now - callback_args_1.Last_trig_time > No_operation_time && (callback_args_1.Next_trig_time == 0 || callback_args_1.Next_trig_time - t_now > No_operation_time))
            {
                callback_args_1.Next_trig_time = t_now + t_delay;
                esp_timer_stop(callback_args_1.handle);
                esp_timer_start_once(callback_args_1.handle, t_delay);
            }
        }

        t_delay = clip((Callback_dat_2.t_start - (esp_timer_get_time() % Callback_dat_2.period)) % Callback_dat_2.period, Delay_min_time, 1000000LL);

        if (callback_args_2.LED_state == false)
        {
            int64_t t_now = esp_timer_get_time();
            // make sure that we won't reset the timer just after LED turns on.
            // also make sure that we won't reset the timer just before LED
            // turns on.
            if (t_now - callback_args_2.Last_trig_time > No_operation_time && (callback_args_2.Next_trig_time == 0 || callback_args_2.Next_trig_time - t_now > No_operation_time))
            {
                callback_args_2.Next_trig_time = t_now + t_delay;
                esp_timer_stop(callback_args_2.handle);
                esp_timer_start_once(callback_args_2.handle, t_delay);
            }
        }

        t_delay = clip((Callback_dat_3.t_start - (esp_timer_get_time() % Callback_dat_3.period)) % Callback_dat_3.period, Delay_min_time, 1000000LL);

        if (callback_args_3.LED_state == false)
        {
            int64_t t_now = esp_timer_get_time();
            // make sure that we won't reset the timer just after LED turns on.
            // also make sure that we won't reset the timer just before LED
            // turns on.
            if (t_now - callback_args_3.Last_trig_time > No_operation_time && (callback_args_3.Next_trig_time == 0 || callback_args_3.Next_trig_time - t_now > No_operation_time))
            {
                callback_args_3.Next_trig_time = t_now + t_delay;
                esp_timer_stop(callback_args_3.handle);
                esp_timer_start_once(callback_args_3.handle, t_delay);
            }
        }
        // QUENCH_G;
    }
}

void Motor_control_task(void *pvParameters)
{
    while (true)
    {
        // wait till next message is received
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // get data
        uint32_t io_flag;
        IR::RX::Parsed_msg_completed msg;
        do
        {
            io_flag = IR::RX::Get_io_flag();

            msg = IR::RX::Get_latest_msg_by_type(1);

        } while (io_flag != IR::RX::Get_io_flag());

        if (msg.content_length)
        {
            LED_set(0, float(msg.content[0]) / float((1 << Motor::PWM_resolution) - 1));
            Motor::Set_speed(msg.content[0]);
        }
    }
}

char ToHEX(uint8_t val)
{
    return (val >= 10) ? 'A' + val - 10 : '0' + val;
}

void Motor_TX_task(void *pvParameters)
{
    while (true)
    {
        vTaskDelay(10);
        if (Serial.available())
        {
            char c = Serial.read();

            uint16_t tval, reg, val;
            String hexString;

            switch (c)
            {
            // thrust value
            case 't':
                tval = Serial.parseInt();

                Serial.print("Thrust => ");
                Serial.println(tval);

                IR::TX::Add_to_schedule(1, {tval}, 2);
                break;

            // register value
            case 'r':
                reg = Serial.parseInt();

                hexString = Serial.readStringUntil('\n');  // Read the HEX number from the serial port and convert it to a string
                val = strtol(hexString.c_str(), NULL, 16); // Convert the HEX string to an integer

                if (reg == 0)
                {
                    Serial.println("Reset!");
                    IR::TX::Add_to_schedule(2, {0}, 3, -1, 2);

                    IR::TX::Add_to_schedule(1, {0}, 2);
                }
                else if (reg < 2 && reg > 24 || val > UINT8_MAX)
                {
                    Serial.println("Invalid register/value!");
                }
                else
                {
                    Serial.print("Register ");
                    Serial.print(reg);
                    Serial.print(" => ");
                    Serial.print(ToHEX(val >> 4));
                    Serial.println(ToHEX(val & 0xF));

                    val += reg << 8;

                    IR::TX::Add_to_schedule(2, {val}, 3, -1, 2);
                }
                break;

            // invalids
            default:
                Serial.println("Invalid input!");
            }

            while (Serial.available())
            {
                Serial.read();
            }
        }
    }
}