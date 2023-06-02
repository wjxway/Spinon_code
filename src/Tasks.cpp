#include "Tasks.hpp"
#include <RobotDefs.hpp>
#include <FastIO.hpp>
#include <FastMath.hpp>
#include <DebugDefs.hpp>
#include <FeedTheDog.hpp>
#include <Circbuffer.hpp>
#include <IrCommunication.hpp>
#include <Localization.hpp>
#include <BitCast.hpp>
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

// target point pos
float target_point[3] = {0.0F, -0.0F, 0.0F};

// K_I has time unit of 1/s
constexpr float K_I_XY = 2.0e-2F;
constexpr float I_XY_range = 150.0F;

constexpr float K_P_XY = 8.0e-2F;
constexpr float P_XY_range = 60.0F;
// K_D has time unit of s
constexpr float K_D_XY = 5.5e-2F;
// K_A has time unit of s^2
constexpr float K_A_XY = 5.0e-2F;

constexpr float K_I_Z = 1.0e-2F;
constexpr float K_P_Z = 2.0e-2F;
constexpr float K_D_Z = 2.0e-2F;

// rotation angle of execution in rad.
constexpr float K_rot = 5.0F / 180.0F * M_PI;
constexpr float P_rot = 5.0F / 180.0F * M_PI;

// time coefficient for filters in s
constexpr float V_filter_t_coef = 0.06F;
constexpr float A_filter_t_coef = 0.30F;

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

        // turn off led if they haven't been refreshed for a while
        if (esp_timer_get_time() - IR::RX::Get_last_RX_time() >= 20000)
        {
            // QUENCH_R;
            // QUENCH_G;
            QUENCH_R;
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
    // implement my own fixed point, because I just need a container.
    template <uint32_t decimal_digits>
    struct fixed_point
    {
    public:
        // divide by 10 ^ decimal_digit is the actual value.
        int16_t val = 0;

        fixed_point(float f)
        {
            val = static_cast<std::int16_t>(std::round(f * std::pow(10.0F, decimal_digits)));
        }

        operator float() const
        {
            return static_cast<float>(val) * std::pow(0.1F, decimal_digits);
        }

        // implicit conversion to string
        operator std::string()
        {
            auto s = std::to_string(float(val) * std::pow(0.1F, decimal_digits));
            size_t pos = s.find('.');
            if (pos != std::string::npos && decimal_digits > 0)
            {
                size_t digits_after_decimal = s.size() - pos - 1;
                if (digits_after_decimal < decimal_digits)
                {
                    s.append(decimal_digits - digits_after_decimal, '0');
                }
                else if (digits_after_decimal > decimal_digits)
                {
                    s.erase(pos + decimal_digits + 1);
                }
            }
            return s;
        }
    };
}

namespace
{
    constexpr uint16_t Compute_throttle(float thrust)
    {
        return uint16_t(4.25F * thrust + 40.0F);
    }

    struct Motor_info
    {
    public:
        Motor_info() {}
        Motor_info(float thrst, uint32_t timet) : speed(Compute_throttle(math::fast::clip(thrst, Motor::Min_thrust, Motor::Max_thrust))), time(timet) {}
        uint16_t speed = 0.0F;
        uint32_t time = 0U;
    };

    constexpr size_t Motor_buffer_max_size = 1500U;
    Circbuffer<Motor_info, Motor_buffer_max_size + 5> Motor_buffer;

    // constexpr size_t Position_buffer_max_size = 500U;
    // Circbuffer<IR::Localization::Position_data, Position_buffer_max_size + 5> Position_buffer;
    // Circbuffer<IR::Localization::Position_data, Position_buffer_max_size + 5> Filtered_Position_buffer;

    // constexpr uint32_t Raw_timing_buffer_max_size = 5;
    // Circbuffer<IR::RX::Msg_timing_t, Raw_timing_buffer_max_size + 5> Raw_timing_buffer;

    // struct Control_info
    // {
    //     int64_t time;
    //     float px;
    //     float py;
    //     float dx;
    //     float dy;
    //     float ax;
    //     float ay;
    // };
    // constexpr uint32_t Control_buffer_max_size = 300U;
    // Circbuffer<Control_info, Control_buffer_max_size + 5> Control_buffer;

    struct Position_data_short
    {
    public:
        Position_data_short(float xx, float yy, float zz, float rr, uint32_t tt) : x(xx), y(yy), z(zz), rot_speed(rr), time(tt) {}
        Position_data_short() {}
        fixed_point<1U> x = 0.0F;         // estimated x position
        fixed_point<1U> y = 0.0F;         // estimated y position
        fixed_point<1U> z = 0.0F;         // estimated z position
        fixed_point<3U> rot_speed = 0.0F; // rotation speed in Hz
        uint32_t time = 0U;               // the time of data, which is the last measurement's time
    };
    constexpr uint32_t Position_data_short_buffer_max_size = 2000U;
    Circbuffer<Position_data_short, Position_data_short_buffer_max_size + 5> Filtered_position_buffer_short;

    // this version sends raw timing readings
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

                Serial.print("Now is ");
                Serial.println(esp_timer_get_time());

                std::string v = "Robot_ID: ";
                v += std::to_string(This_robot_ID) + "\nK_P_Z = " + std::to_string(K_P_Z) + ", K_D_Z = " + std::to_string(K_D_Z) + ", K_I_Z = " + std::to_string(K_I_Z) + "\nK_P_XY = " + std::to_string(K_P_XY) + ", K_D_XY = " + std::to_string(K_D_XY) + ", K_A_XY = " + std::to_string(K_A_XY) + "\nV_filter_t_coef = " + std::to_string(V_filter_t_coef) + ", A_filter_t_coef = " + std::to_string(A_filter_t_coef) + "\nTarget = { " + std::to_string(target_point[0]) + " , " + std::to_string(target_point[1]) + " , " + std::to_string(target_point[2]) + " }\n";

                Serial.print(v.c_str());

                // Serial.println("---- Unfiltered position ----");
                // while (Position_buffer.n_elem > 0)
                // {
                //     auto pdat = Position_buffer.pop();

                //     std::string v = std::string("t : ") + std::to_string(pdat.time) + std::string(", x : ") + std::to_string(pdat.x) + std::string(", y : ") + std::to_string(pdat.y) + std::string(", z : ") + std::to_string(pdat.z) + std::string(", var_xy : ") + std::to_string(pdat.var_xy) + std::string(", var_z : ") + std::to_string(pdat.var_z) + std::string(", w : ") + std::to_string(pdat.angular_velocity) + std::string(", err_factor : ") + std::to_string(pdat.mean_error_factor) + "\n";

                //     Serial.print(v.c_str());
                // }

                // Serial.println("---- Filtered position ----");
                // while (Filtered_Position_buffer.n_elem > 0)
                // {
                //     auto pdat = Filtered_Position_buffer.pop();

                //     std::string v = std::string("t : ") + std::to_string(pdat.time) + std::string(", x : ") + std::to_string(pdat.x) + std::string(", y : ") + std::to_string(pdat.y) + std::string(", z : ") + std::to_string(pdat.z) + std::string(", var_xy : ") + std::to_string(pdat.var_xy) + std::string(", var_z : ") + std::to_string(pdat.var_z) + std::string(", w : ") + std::to_string(pdat.angular_velocity) + std::string(", err_factor : ") + std::to_string(pdat.mean_error_factor) + "\n";

                //     Serial.print(v.c_str());
                // }

                Serial.println("---- Filtered position ----");
                while (Filtered_position_buffer_short.n_elem > 0)
                {
                    auto pdat = Filtered_position_buffer_short.pop();

                    std::string v = std::string("t : ") + std::to_string(pdat.time) + std::string(", x : ") + std::string(pdat.x) + std::string(", y : ") + std::string(pdat.y) + std::string(", z : ") + std::string(pdat.z) + std::string(", f : ") + std::string(pdat.rot_speed) + "\n";

                    Serial.print(v.c_str());
                }

                // Serial.println("---- Raw timing ----");
                // while (Raw_timing_buffer.n_elem > 0)
                // {
                //     auto pdat = Raw_timing_buffer.pop();

                //     std::string v = std::string("ID : ") + std::to_string(pdat.robot_ID) + std::string(", t0 : ") + std::to_string(pdat.time_arr[0]) + std::string(", t1 : ") + std::to_string(pdat.time_arr[1]) + std::string(", t2 : ") + std::to_string(pdat.time_arr[2]) + "\n";

                //     Serial.print(v.c_str());
                // }

                Serial.println("---- Motor feedback data ----");
                while (Motor_buffer.n_elem > 0)
                {
                    auto fdat = Motor_buffer.pop();

                    std::string v = std::string("update_t : ") + std::to_string(fdat.time) + std::string(", thrust : ") + std::to_string(fdat.speed) + "\n";

                    Serial.print(v.c_str());
                }

                // Serial.println("---- Control data ----");
                // while (Control_buffer.n_elem > 0)
                // {
                //     auto cdat = Control_buffer.pop();

                //     std::string v = std::string("px : ") + std::to_string(cdat.px) + std::string(", py : ") + std::to_string(cdat.py) + std::string(", dx : ") + std::to_string(cdat.dx) + std::string(", dy : ") + std::to_string(cdat.dy) + std::string(", ax : ") + std::to_string(cdat.ax) + std::string(", ay : ") + std::to_string(cdat.ay) + ", time : " + std::to_string(cdat.time) + "\n";

                //     Serial.print(v.c_str());
                // }
            }
        }
    }
}

// this version buffer localization data
void Buffer_data_task(void *pvParameters)
{
    // position data is valid for 1s
    constexpr int64_t Position_expire_time = 1000000LL;

    // if send message task has been started.
    bool send_task_not_started = true;

    // send me messages through serial!
    xTaskCreatePinnedToCore(
        Send_message_task,
        "Send_message_task",
        8000,
        NULL,
        3,
        NULL,
        0);

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
            if (IR::Localization::Get_position_data_count(Position_expire_time) == 0)
            {
                not_enough_data = true;
                break;
            }

            // pos_0 = IR::Localization::Get_position();
            pos_1 = IR::Localization::Get_filtered_position();
        } while (io_flag != IR::Localization::Get_io_flag());

        if (not_enough_data)
        {
            continue;
        }

        // Position_buffer.push(pos_0);
        Filtered_position_buffer_short.push(Position_data_short{pos_1.x, pos_1.y, pos_1.z, pos_1.angular_velocity * 1.0e6F / 2.0F / float(M_PI), static_cast<uint32_t>(pos_1.time)});

        // if (send_task_not_started && Position_buffer.n_elem >= Position_buffer_max_size)
        // {
        //     send_task_not_started = false;
        //     LIT_G;
        // }
    }
}

// // this version buffer raw timing data
// void Buffer_raw_data_task(void *pvParameters)
// {
//     // if send message task has been started.
//     bool send_task_not_started = true;

//     // // send me messages through serial!
//     // xTaskCreatePinnedToCore(
//     //     Send_message_task,
//     //     "Send_message_task",
//     //     8000,
//     //     NULL,
//     //     3,
//     //     NULL,
//     //     0);

//     int64_t t_prev = 0, t_now = 0;

//     while (true)
//     {
//         // wait till next timing data is obtained
//         ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//         // get data
//         uint32_t io_flag;
//         uint32_t temp_size = 0;
//         IR::RX::Msg_timing_t temp[25];
//         do
//         {
//             t_now = esp_timer_get_time();

//             io_flag = IR::Localization::Get_io_flag();

//             temp_size = IR::RX::Get_timing_data(temp, t_now - t_prev);

//         } while (io_flag != IR::Localization::Get_io_flag());

//         for (int i = 0; i < temp_size; i++)
//         {
//             Raw_timing_buffer.push(temp[temp_size - i - 1]);
//         }

//         t_prev = t_now;
//     }
// }

namespace
{
    // R,G,B LED corresponds to offset + 0,1,2
    // be aware that this might collide with Motor PWM
    constexpr uint32_t LED_LEDC_channel_offset = 4U;

    // LEDC frequency for LED
    constexpr uint32_t LED_PWM_frequency = 40000U;

    // LEDC resolution for LED
    constexpr uint32_t LED_PWM_resolution = 10U;
} // anonymous namespace

void LED_PWM_init(uint32_t channels)
{
    if (channels & 1)
    {
        ledcSetup(LED_LEDC_channel_offset, LED_PWM_frequency, LED_PWM_resolution);
        ledcAttachPin(LED_PIN_R, LED_LEDC_channel_offset);
        ledcWrite(LED_LEDC_channel_offset, (1 << LED_PWM_resolution) - 1);
    }

    if (channels & 2)
    {
        ledcSetup(LED_LEDC_channel_offset + 1, LED_PWM_frequency, LED_PWM_resolution);
        ledcAttachPin(LED_PIN_G, LED_LEDC_channel_offset + 1);
        ledcWrite(LED_LEDC_channel_offset + 1, (1 << LED_PWM_resolution) - 1);
    }

    if (channels & 4)
    {
        ledcSetup(LED_LEDC_channel_offset + 2, LED_PWM_frequency, LED_PWM_resolution);
        ledcAttachPin(LED_PIN_B, LED_LEDC_channel_offset + 2);
        ledcWrite(LED_LEDC_channel_offset + 2, (1 << LED_PWM_resolution) - 1);
    }
}

void LED_set(uint32_t color, float duty)
{
    ledcWrite(LED_LEDC_channel_offset + color, uint32_t(math::fast::clip(1.0F - duty, 0.0F, 1.0F) * float((1 << LED_PWM_resolution) - 1)));
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
    constexpr float LED_angle_offset = 0.88187194F;

    /**
     * @brief relative angle of motor
     */
    constexpr float Motor_angle_offset = 0.174533F;

    /**
     * @brief minimum delay time, if lower than this, we trigger after this.
     */
    constexpr int64_t Delay_min_time = 60LL;

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
        int64_t next_time;

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

        // reset timer
        arg->Last_trig_time = t_now;
        arg->Next_trig_time += next_time;

        // not sure if this is necessary, but we will add it anyways, it's not gonna influence anything.
        next_time = arg->Next_trig_time - esp_timer_get_time();
        next_time = (next_time < Delay_min_time) ? Delay_min_time : next_time;

        esp_timer_stop(arg->handle);
        esp_timer_start_once(arg->handle, next_time);
    }

    struct Motor_timing_t
    {
        int64_t t_start;          // when we should switch the motor to thrust_high, absolute time.
        int64_t period;           // period of rotation
        float thrust_high = 0.0F; // higher half thrust
        float thrust_low = 0.0F;  // lower half thrust
    };

    struct Motor_callback_args
    {
        uint32_t Motor_state;                  // current state of motor
        Motor_timing_t Last_motor_target;      // last target used by the callback
        int64_t Last_trig_time;                // last time we updated the trigger info
        int64_t Next_trig_time;                // when will we trig next time
        Circbuffer<Motor_timing_t, 5> *buffer; // timing info buffer
        esp_timer_handle_t handle;             // callback handler
    };

    Motor_callback_args callback_args_m;
    Circbuffer<Motor_timing_t, 5> timing_buf_m;

    int64_t Last_position_update_time = 0;
    int64_t Reach_target_speed_time = 0;

    /**
     * @brief a task that turn motor based on robot's position
     *
     * @param void* v_arg arguments
     */
    void IRAM_ATTR Motor_callback(void *v_arg)
    {
        Motor_callback_args *arg = (Motor_callback_args *)v_arg;

        uint64_t t_now = esp_timer_get_time();

        // for arg -> Motor_state
        // state = 0 -> before pulse
        // state = 1 -> in pulse
        // state = 2 -> after pulse

        // store timing for state 2
        static int64_t timing_temp = 0;

        // thrust to be set, 0 -> low, 1 -> high
        static bool to_set_thrust = 0;

        // how long should we delay before next switch
        uint64_t next_time;

        Motor_timing_t temp;

        // state = 0 -> before pulse
        // state = 1 -> in pulse
        // state = 2 -> after pulse
        // always cycle through these three
        switch (arg->Motor_state)
        {
        case 0: // if was 0, the next time period should be in pulse
            to_set_thrust = !to_set_thrust;
            next_time = arg->Last_motor_target.period / 2;
            arg->Motor_state = 1;
            break;

        case 1: // if was 1, the next time period should be after pulse
            to_set_thrust = !to_set_thrust;
            next_time = timing_temp;
            arg->Motor_state = 2;
            break;

        case 2: // if was 2, the next time period should be new before pulse
            // get latest motor timing data
            temp = arg->buffer->peek_tail();
            // now this temp is used, update Last_LED_timing
            arg->Last_motor_target = temp;

            // update time
            next_time = (temp.t_start - (t_now % temp.period)) % temp.period;
            if (next_time >= temp.period / 2)
            {
                to_set_thrust = 1;
                next_time -= temp.period / 2;
            }
            else
            {
                to_set_thrust = 0;
            }
            timing_temp = temp.period / 2 - next_time;
            arg->Motor_state = 0;
            break;

        default:
            break;
        }

        // if started and not ended, we record the FB info
        if (Reach_target_speed_time && !Motor::Get_brake_status())
        {
            LED_set(1, to_set_thrust);

            float actual_thrust_value = to_set_thrust ? arg->Last_motor_target.thrust_high : arg->Last_motor_target.thrust_low;

            Motor_buffer.push(Motor_info{actual_thrust_value, static_cast<uint32_t>(t_now)});
            Motor::Set_thrust(actual_thrust_value);
        }

        // reset timer
        arg->Last_trig_time = t_now;
        arg->Next_trig_time += next_time;

        // not sure if this is necessary, but we will add it anyways, it's not gonna influence anything.
        next_time = arg->Next_trig_time - esp_timer_get_time();
        next_time = (next_time < Delay_min_time) ? Delay_min_time : next_time;

        esp_timer_stop(arg->handle);
        esp_timer_start_once(arg->handle, next_time);
    }
} // anonymous namespace

namespace
{
    // true for speed 1, false for speed 2
    bool motor_state = false;
    uint8_t motor_spd_1 = 0;
    uint8_t motor_spd_2 = 0;

    void IRAM_ATTR Motor_test_callback(void *pvParameters)
    {
        if (motor_state)
        {
            DEBUG_C(setbit(DEBUG_PIN_2));
            Motor::Set_speed(motor_spd_1);
        }
        else
        {
            DEBUG_C(clrbit(DEBUG_PIN_2));
            Motor::Set_speed(motor_spd_2);
        }
        motor_state = !motor_state;
    }
}

void Motor_test_task(void *pvParameters)
{
    static bool just_start = 1;

    // get data
    auto buf_1 = IR::RX::Get_msg_buffer_by_type(1);
    auto buf_2 = IR::RX::Get_msg_buffer_by_type(2);

    constexpr uint64_t motor_spd_switch_time = 20000;

    const esp_timer_create_args_t oneshot_timer_callback_args_1 = {
        .callback = &Motor_test_callback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "test"};

    esp_timer_handle_t tm_handle;
    esp_timer_create(&oneshot_timer_callback_args_1, &tm_handle);

    esp_timer_start_periodic(tm_handle, motor_spd_switch_time);

    while (true)
    {
        // wait till next message is received
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!buf_1.Empty_Q())
        {
            auto msg = buf_1.pop();

            LIT_G;
            vTaskDelay(500);
            QUENCH_G;

            // LED_set(0, float(msg.content[0]) / float((1 << Motor::PWM_resolution) - 1));
            // Serial.println(msg.content[0]);
            // Motor::Set_speed(msg.content[0]);

            motor_spd_1 = (msg.content[0] >> 8) & 0xFF;
            motor_spd_2 = msg.content[0] & 0xFF;
        }

        if (!buf_2.Empty_Q())
        {
            auto msg = buf_2.pop();

            LIT_G;
            vTaskDelay(500);
            QUENCH_G;

            // reset command
            if (msg.content[0] == 0)
            {
                if (!just_start)
                {
                    esp_restart();
                }
            }
            else
            {
                Motor::Config_register((msg.content[0] >> 8) & 0xFF, msg.content[0] & 0xFF);
            }

            just_start = 0;
        }
    }
}

void Motor_control_task(void *pvParameters)
{
    const float cos_rot = cos(P_rot - K_rot);
    const float sin_rot = sin(P_rot - K_rot);

    // position data is valid for 1s
    constexpr int64_t Position_expire_time = 1000000LL;

    bool ISR_started = false;

    // PID components
    float I_comp[3] = {0.0F, 0.0F, 0.0F};
    float P_comp[3] = {0.0F, 0.0F, 0.0F};
    float D_comp[3] = {0.0F, 0.0F, 0.0F};
    float Filtered_D_comp[3] = {0.0F, 0.0F, 0.0F};
    float A_comp[3] = {0.0F, 0.0F, 0.0F};
    float Filtered_A_comp[3] = {0.0F, 0.0F, 0.0F};
    float FB_val[3] = {0.0F, 0.0F, 0.0F};

    // buffer for D time
    int64_t D_last_update_time = 0;

    // if integration term is on
    bool integration_on = false;

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

        // update localization information every 0.25s
        // note that the update interval should not be smaller than the rotation
        // period, or there might be cases where two different messages share
        // the same msg_id_init (which is used for signaling message change),
        // causing confusion.
        static int64_t last_TX_update_time = 0;
        constexpr int64_t TX_update_interval = 50000;
        // if over time, update position based on current position, set priority to 2
        if (esp_timer_get_time() - last_TX_update_time > TX_update_interval)
        {
            last_TX_update_time = esp_timer_get_time();
            IR::TX::Add_to_schedule(4, {std::bit_cast<uint16_t>((int16_t)(filt_pos_0.x)), std::bit_cast<uint16_t>((int16_t)(filt_pos_0.y)), std::bit_cast<uint16_t>((int16_t)(filt_pos_0.z))}, 2);
        }

        // pre-control actions
        // 0 for [0,speed1), 1 for [speed1,speed2), 2 for [speed2,inf)
        static int control_on = 0;
        // release brake when speed reached a certain threshold
        constexpr float Rotation_speed_start_1 = 0.000080F;
        constexpr float Rotation_speed_start_2 = 0.000125F;
        // pre-start thurst ratio
        constexpr float Init_thrust_ratio = 0.7F;
        switch (control_on)
        {
        case 0:
            if (filt_pos_0.angular_velocity >= Rotation_speed_start_1)
            {
                control_on = 1;
                Motor::Active_brake_release();
                Motor::Set_thrust(Init_thrust_ratio * Robot_mass);
            }
            break;
        case 1:
            if (filt_pos_0.angular_velocity >= Rotation_speed_start_2)
            {
                control_on = 2;
                // enable overdrive
                // Motor::Set_overdrive(true);
                Motor::Set_thrust(20);

                Reach_target_speed_time = esp_timer_get_time();
            }
            break;

            // case 2:
            //     if (pos_0.z > -20.0F)
            //     {
            //         control_on = 3;

            //         Reach_target_speed_time = esp_timer_get_time();
            //     }
            //     break;

        default:
            break;
        }

        if (!integration_on && filt_pos_0.z >= -125.0F)
        {
            I_comp[0] = 0;
            I_comp[1] = 0;
            I_comp[2] = 0;

            integration_on = true;
        }

        // // change target point to make step response
        // int64_t t_elapsed = esp_timer_get_time() - Reach_target_speed_time;
        // if (Reach_target_speed_time != 0 && t_elapsed >= 20000000LL)
        // {
        //     if (t_elapsed >= 50000000LL)
        //     {
        //         target_point[0] = -80.0F;
        //     }
        //     else if (t_elapsed >= 35000000LL)
        //     {
        //         target_point[0] = 80.0F;
        //     }
        //     else
        //     {
        //         target_point[0] = -80.0F;
        //     }
        // }

        Last_position_update_time = esp_timer_get_time();

        P_comp[0] = filt_pos_0.x - target_point[0];
        P_comp[1] = filt_pos_0.y - target_point[1];
        P_comp[2] = filt_pos_0.z - target_point[2];

        // add a constraint to horizontal P comp magnitude
        float pmag = norm(P_comp[0], P_comp[1]);
        float pmag_mult = clip(pmag, 0.0F, P_XY_range) / (pmag + 1.0e-6F);
        P_comp[0] *= pmag_mult;
        P_comp[1] *= pmag_mult;

        float t_coef;

        if (integration_on)
        {
            t_coef = float(pos_0.time - pos_1.time) / 2.0e6F;
            I_comp[0] += t_coef * (pos_0.x + pos_1.x - 2.0F * target_point[0]);
            I_comp[1] += t_coef * (pos_0.y + pos_1.y - 2.0F * target_point[1]);
            I_comp[2] += t_coef * (pos_0.z + pos_1.z - 2.0F * target_point[2]);
        }

        // add a constraint to horizontal I comp magnitude
        float imag = norm(I_comp[0], I_comp[1]);
        float imag_mult = clip(imag, 0.0F, I_XY_range) / (imag + 1.0e-6F);
        I_comp[0] *= imag_mult;
        I_comp[1] *= imag_mult;

        t_coef = 1.0e6F / float(pos_0.time - pos_1.time);

        // store old D_component for acceleration computation
        float old_D_comp[3] = {D_comp[0], D_comp[1], D_comp[2]};

        D_comp[0] = t_coef * (filt_pos_0.x - filt_pos_1.x);
        D_comp[1] = t_coef * (filt_pos_0.y - filt_pos_1.y);
        D_comp[2] = t_coef * (filt_pos_0.z - filt_pos_1.z);

        // update coefficient, choose the time coefficient to be approximately
        // averaging over 0.1s, so total is 0.1s + Kalmann Filter delay.
        int64_t D_this_update_time = (filt_pos_0.time >> 1) + (filt_pos_0.time >> 1);
        float update_coef = clip(1e-6F * float(D_this_update_time - D_last_update_time) / V_filter_t_coef, 0.0F, 1.0F);

        float Filtered_D_comp_buffer[3];
        for (size_t i = 0; i < 3; i++)
        {
            Filtered_D_comp_buffer[i] = D_comp[i] * update_coef + Filtered_D_comp[i] * (1.0F - update_coef);
        }

        // compute acceleration based on filtered D comp and also filter with average over 0.2s, so total is 0.3s + Kalmann Filter delay.
        update_coef = clip(1e-6F * float(D_this_update_time - D_last_update_time) / A_filter_t_coef, 0.0F, 1.0F);
        for (size_t i = 0; i < 3; i++)
        {
            A_comp[i] = (Filtered_D_comp_buffer[i] - Filtered_D_comp[i]) * 1.0e6F / (D_this_update_time - D_last_update_time);
            Filtered_A_comp[i] = A_comp[i] * update_coef + Filtered_A_comp[i] * (1.0F - update_coef);

            Filtered_D_comp[i] = Filtered_D_comp_buffer[i];
        }

        D_last_update_time = D_this_update_time;

        // Control_info temp;
        // temp.time = D_last_update_time;
        // temp.px = -K_P_XY * P_comp[0];
        // temp.py = -K_P_XY * P_comp[1];
        // temp.dx = -K_D_XY * Filtered_D_comp[0];
        // temp.dy = -K_D_XY * Filtered_D_comp[1];
        // temp.ax = -K_A_XY * Filtered_A_comp[0];
        // temp.ay = -K_A_XY * Filtered_A_comp[1];
        // Control_buffer.push(temp);

        // construct X,Y,Z control law
        FB_val[0] = -K_P_XY * (cos_rot * P_comp[0] - sin_rot * P_comp[1]) - K_D_XY * Filtered_D_comp[0] - K_A_XY * Filtered_A_comp[0] - K_I_XY * I_comp[0];
        FB_val[1] = -K_P_XY * (sin_rot * P_comp[0] + cos_rot * P_comp[1]) - K_D_XY * Filtered_D_comp[1] - K_A_XY * Filtered_A_comp[1] - K_I_XY * I_comp[1];
        FB_val[2] = -K_P_Z * P_comp[2] - K_D_Z * D_comp[2] - K_I_Z * I_comp[2] + Robot_mass;

        if (norm(pos_0.x - target_point[0], pos_0.y - target_point[1]) > 200.0F)
        {
            FB_val[0] = -(cos_rot * P_comp[0] - sin_rot * P_comp[1]);
            FB_val[1] = -(sin_rot * P_comp[0] + cos_rot * P_comp[1]);
        }

        // all computation has been finished till now
        // from now on it's the boring setup interrupt part

        // cap the duration!
        constexpr int64_t Duration_max_time = 25000;

        // // compute timing for callback_1 that use LED to indicate position
        // LED_timing_t Callback_dat_1;

        // Callback_dat_1.duration = clip(int64_t(0.005F * norm(P_comp[0], P_comp[1]) / pos_0.angular_velocity), 0LL, Duration_max_time);
        // Callback_dat_1.t_start = pos_0.rotation_time * 5 - Callback_dat_1.duration / 2 - int64_t((LED_angle_offset + pos_0.angle_0 - atan2f(P_comp[1], P_comp[0])) / pos_0.angular_velocity);
        // Callback_dat_1.period = pos_0.rotation_time;

        // // push inside buffer
        // timing_buf_1.push(Callback_dat_1);

        // // compute timing for callback_2 that use LED to indicate velocity
        // LED_timing_t Callback_dat_2;

        // Callback_dat_2.duration = clip(int64_t(0.005F * norm(D_comp[0], D_comp[1]) / pos_0.angular_velocity), 0LL, Duration_max_time);
        // Callback_dat_2.t_start = pos_0.rotation_time * 5 - Callback_dat_2.duration / 2 - int64_t((LED_angle_offset + pos_0.angle_0 - atan2f(D_comp[1], D_comp[0])) / pos_0.angular_velocity);
        // Callback_dat_2.period = pos_0.rotation_time;

        // // push inside buffer
        // timing_buf_2.push(Callback_dat_2);

        // compute timing for callback_3 that use LED to indicate feedback value
        Motor_timing_t Callback_dat_m;

        // each cycle start with low->high
        Callback_dat_m.period = pos_0.rotation_time;
        Callback_dat_m.t_start = int64_t((atan2f(FB_val[1], FB_val[0]) + K_rot - Motor_angle_offset - pos_0.angle_0 + 10.5F * M_PI) / pos_0.angular_velocity);

        float min_thrust = Motor::Min_thrust, max_thrust = Motor::Max_thrust;

#if MOTOR_OVERDRIVE_ENABLED
        if (Motor::Get_overdrive_mode())
        {
            min_thrust = Motor::Min_thrust_overdrive;
            max_thrust = Motor::Max_thrust_overdrive;
        }
#endif

        // clip add_val to make sure that the average is still the average and the values will not exceed limits.
        // similar for motor settings.
        FB_val[2] = clip(FB_val[2], min_thrust, max_thrust);

        float add_val = norm(FB_val[0], FB_val[1]);
        if (FB_val[2] > (max_thrust + min_thrust) / 2)
        {
            if (FB_val[2] + add_val > max_thrust)
            {
                add_val = max_thrust - FB_val[2];
            }
        }
        else
        {
            if (FB_val[2] - add_val < min_thrust)
            {
                add_val = FB_val[2] - min_thrust;
            }
        }
        Callback_dat_m.thrust_high = FB_val[2] + add_val;
        Callback_dat_m.thrust_low = FB_val[2] - add_val;

        // push inside buffer
        timing_buf_m.push(Callback_dat_m);

        // check if first time, if so, start ISR
        if (!ISR_started)
        {
            ISR_started = true;

            // // Callback/ISR 1
            // callback_args_1.buffer = &timing_buf_1;
            // callback_args_1.Last_LED_timing = Callback_dat_1;
            // callback_args_1.LED_num = 0;
            // callback_args_1.LED_state = false;
            // callback_args_1.Last_trig_time = 0;
            // callback_args_1.Next_trig_time = 0;

            // const esp_timer_create_args_t oneshot_timer_callback_args_1 = {
            //     .callback = &LED_callback,
            //     /* argument specified here will be passed to timer callback function */
            //     .arg = (void *)(&callback_args_1),
            //     .dispatch_method = ESP_TIMER_TASK,
            //     .name = "position"};

            // esp_timer_create(&oneshot_timer_callback_args_1, &(callback_args_1.handle));

            // // Callback/ISR 2
            // callback_args_2.buffer = &timing_buf_2;
            // callback_args_2.Last_LED_timing = Callback_dat_2;
            // callback_args_2.LED_num = 1;
            // callback_args_2.LED_state = false;
            // callback_args_2.Last_trig_time = 0;
            // callback_args_2.Next_trig_time = 0;

            // const esp_timer_create_args_t oneshot_timer_callback_args_2 = {
            //     .callback = &LED_callback,
            //     /* argument specified here will be passed to timer callback function */
            //     .arg = (void *)(&callback_args_2),
            //     .dispatch_method = ESP_TIMER_TASK,
            //     .name = "velocity"};

            // esp_timer_create(&oneshot_timer_callback_args_2, &(callback_args_2.handle));

            // Callback/ISR 3
            callback_args_m.buffer = &timing_buf_m;
            callback_args_m.Last_motor_target = Callback_dat_m;
            callback_args_m.Motor_state = 0;
            callback_args_m.Last_trig_time = 0;
            callback_args_m.Next_trig_time = 0;

            const esp_timer_create_args_t oneshot_timer_callback_args_m = {
                .callback = &Motor_callback,
                /* argument specified here will be passed to timer callback function */
                .arg = (void *)(&callback_args_m),
                .dispatch_method = ESP_TIMER_TASK,
                .name = "motor"};

            esp_timer_create(&oneshot_timer_callback_args_m, &(callback_args_m.handle));
            esp_timer_start_once(callback_args_m.handle, 50U);
        }

        // how long before/after the trigger time should we prevent triggering
        // in us
        constexpr uint64_t No_operation_time = 200ULL;
        int64_t t_delay;

        // // updating timer info
        // // not sure if clipping is necessary, but we will add it anyways,
        // // it's not gonna influence anything.
        // t_delay = clip((Callback_dat_1.t_start - (esp_timer_get_time() % Callback_dat_1.period)) % Callback_dat_1.period, Delay_min_time, 1000000LL);

        // if (callback_args_1.LED_state == false)
        // {
        //     int64_t t_now = esp_timer_get_time();
        //     // make sure that we won't reset the timer just after LED turns on.
        //     // also make sure that we won't reset the timer just before LED
        //     // turns on.
        //     if (t_now - callback_args_1.Last_trig_time > No_operation_time && (callback_args_1.Next_trig_time == 0 || callback_args_1.Next_trig_time - t_now > No_operation_time))
        //     {
        //         callback_args_1.Next_trig_time = t_now + t_delay;
        //         esp_timer_stop(callback_args_1.handle);
        //         esp_timer_start_once(callback_args_1.handle, t_delay);
        //     }
        // }

        // t_delay = clip((Callback_dat_2.t_start - (esp_timer_get_time() % Callback_dat_2.period)) % Callback_dat_2.period, Delay_min_time, 1000000LL);

        // if (callback_args_2.LED_state == false)
        // {
        //     int64_t t_now = esp_timer_get_time();
        //     // make sure that we won't reset the timer just after LED turns on.
        //     // also make sure that we won't reset the timer just before LED
        //     // turns on.
        //     if (t_now - callback_args_2.Last_trig_time > No_operation_time && (callback_args_2.Next_trig_time == 0 || callback_args_2.Next_trig_time - t_now > No_operation_time))
        //     {
        //         callback_args_2.Next_trig_time = t_now + t_delay;
        //         esp_timer_stop(callback_args_2.handle);
        //         esp_timer_start_once(callback_args_2.handle, t_delay);
        //     }
        // }

        // t_delay = clip((Callback_dat_m.t_start - (esp_timer_get_time() % Callback_dat_m.period)) % Callback_dat_m.period, Delay_min_time, 1000000LL);

        // if (callback_args_m.Motor_state == false)
        // {
        //     int64_t t_now = esp_timer_get_time();
        //     // make sure that we won't reset the timer just after motor turns +.
        //     // also make sure that we won't reset the timer just before motor
        //     // turns on.
        //     if (t_now - callback_args_m.Last_trig_time > No_operation_time && (callback_args_m.Next_trig_time == 0 || callback_args_m.Next_trig_time - t_now > No_operation_time))
        //     {
        //         callback_args_m.Next_trig_time = t_now + t_delay;
        //         esp_timer_stop(callback_args_m.handle);
        //         esp_timer_start_once(callback_args_m.handle, t_delay);
        //     }
        // }
        // QUENCH_G;
    }
}

void Motor_monitor_task(void *pvParameters)
{
    // when will we lower the motor power after no position update
    constexpr uint64_t Power_low_threshold = 200000;

    // when power low, what's the thrust?
    constexpr float Power_low_value = 0.8F;

    // when will we turn off motor power after no position update
    constexpr uint64_t Power_off_threshold = 1000000;

    while (true)
    {
        vTaskDelay(20);

        int64_t t_now = esp_timer_get_time();

        if (!Motor::Get_brake_status())
        {
            if (t_now - Last_position_update_time >= Power_off_threshold)
            {
                Motor::Set_speed(0);
                Motor::Active_brake();
            }

            if (t_now - Last_position_update_time >= Power_low_threshold)
            {
                callback_args_m.Last_motor_target.thrust_high = Power_low_value * Robot_mass;
                callback_args_m.Last_motor_target.thrust_low = Power_low_value * Robot_mass;

                Motor::Set_thrust(Power_low_value * Robot_mass);
            }
        }
    }
}

void Message_relay_task(void *pvParameters)
{
    Circbuffer_copycat<IR::RX::Parsed_msg_completed, 20> msg_buf = IR::RX::Get_msg_buffer_by_type(4);

    while (true)
    {
        // wait till next data is obtained
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (!msg_buf.Empty_Q())
        {
            auto msg = msg_buf.pop();

            std::string str = "";
            str += std::to_string(msg.robot_ID) + ":{" + std::to_string(std::bit_cast<int16_t>(msg.content[0])) + "," + std::to_string(std::bit_cast<int16_t>(msg.content[1])) + "," + std::to_string(std::bit_cast<int16_t>(msg.content[2])) + "}\n";

            Serial.print(str.c_str());
        }
    }
}