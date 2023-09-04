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
#include <EKFTask.hpp>
#include <MotorCtrl.hpp>

#define _USE_MATH_DEFINES
#include <cmath>

using math::fast::clip;
using math::fast::norm;
using math::fast::square;

// target point pos
float target_point[3] = {0.0F, 0.0F, 0.0F};

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
constexpr float K_P_Z = 2.5e-2F;
constexpr float K_D_Z = 1.5e-2F;

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

    // struct Motor_info
    // {
    // public:
    //     Motor_info() {}
    //     Motor_info(uint16_t spd, uint32_t t) : speed(spd), time(t) {}
    //     uint16_t speed = 0U;
    //     uint32_t time = 0U;
    // };

    // constexpr size_t Motor_buffer_max_size = 1000U;
    // Circbuffer<Motor_info, Motor_buffer_max_size + 5> Motor_buffer;

    struct Position_data_short
    {
    public:
        Position_data_short(const float xx, const float yy, const float zz, const float rr, const uint32_t tt) : x(xx), y(yy), z(zz), rot_speed(rr), time(tt) {}
        Position_data_short() {}
        fixed_point<1U> x = 0.0F;         // estimated x position
        fixed_point<1U> y = 0.0F;         // estimated y position
        fixed_point<1U> z = 0.0F;         // estimated z position
        fixed_point<3U> rot_speed = 0.0F; // rotation speed in Hz
        uint32_t time = 0U;               // the time of data, which is the last measurement's time
    };
    constexpr uint32_t Position_data_short_buffer_max_size = 1200U; // 1500U;
    Circbuffer<Position_data_short, Position_data_short_buffer_max_size + 5> Filtered_position_buffer_short;

    struct Timing_data_short
    {
        uint32_t time[3];
        uint16_t rid;
    };

    constexpr uint32_t Timing_buffer_max_size = 5U; // 1500U;
    Circbuffer<Timing_data_short, Timing_buffer_max_size + 5> Timing_buffer;

    struct EKF_all
    {
        EKF::State_vector st;
        uint32_t time;
    };

    constexpr uint32_t EKF_buffer_max_size = 400U; // 1500U;
    Circbuffer<EKF_all, EKF_buffer_max_size + 5> EKF_buffer;

    // struct Motor_timing_t
    // {
    //     int64_t t_start;          // when we should switch the motor to spd_high, absolute time.
    //     int64_t period;           // period of rotation
    //     uint16_t spd_low = 0.0F;  // speed low end
    //     uint16_t spd_high = 0.0F; // speed high end

    //     float thrust_angle = 0.0F; // at which angle the thrust is high
    // };

    // constexpr uint32_t Motor_timing_buffer_max_size = 50U; // 1500U;
    // Circbuffer<Motor_timing_t, Motor_timing_buffer_max_size + 5> Motor_timing_buffer;

    // starting from when we apply constant thrust
    int64_t T_switch_EKF = 0;

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

                Serial.print("Const thrust from ");
                Serial.println(T_switch_EKF);

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

                Serial.println("---- EKF position ----");
                while (EKF_buffer.n_elem > 0)
                {
                    auto pdat = EKF_buffer.pop();

                    std::string v = std::string("t : ") + std::to_string(pdat.time) + std::string(", gamma_x : ") + std::to_string(pdat.st.state[EKF::state_para::gamma_x]) + std::string(", gamma_y : ") + std::to_string(pdat.st.state[EKF::state_para::gamma_y]) + std::string(", v_x : ") + std::to_string(pdat.st.state[EKF::state_para::v_x]) + std::string(", v_y : ") + std::to_string(pdat.st.state[EKF::state_para::v_y]) + std::string(", x : ") + std::to_string(pdat.st.state[EKF::state_para::x]) + std::string(", y : ") + std::to_string(pdat.st.state[EKF::state_para::y]) + std::string(", v_z : ") + std::to_string(pdat.st.state[EKF::state_para::v_z]) + std::string(", z : ") + std::to_string(pdat.st.state[EKF::state_para::z]) + std::string(", omega : ") + std::to_string(pdat.st.state[EKF::state_para::omega]) + "\n";

                    Serial.print(v.c_str());
                }

                // Serial.println("---- Motor timing ----");
                // while (Motor_timing_buffer.n_elem > 0)
                // {
                //     auto mdat = Motor_timing_buffer.pop();

                //     std::string v = std::string("t_start : ") + std::to_string(mdat.t_start) + std::string(", period : ") + std::to_string(mdat.period) + std::string(", angle : ") + std::to_string(mdat.thrust_angle) + std::string(", low : ") + std::to_string(mdat.spd_low) + std::string(", high : ") + std::to_string(mdat.spd_high) + "\n";

                //     Serial.print(v.c_str());
                // }

                // Serial.println("---- act & meas ----");

                // while (EKF::actmeas_list.n_elem)
                // {
                //     auto all = EKF::actmeas_list.pop();
                //     auto adat = all.act;
                //     auto mdat = all.meas;

                //     std::string v = std::string("t_0 : ") + std::to_string(mdat.time) + std::string(", x_0 : ") + std::to_string(mdat.x_0) + std::string(", y_0 : ") + std::to_string(mdat.y_0) + std::string(", z_0 : ") + std::to_string(mdat.z_0) + std::string(", LR_dt : ") + std::to_string(mdat.LR_dt * 1.0e3f) + std::string(", Cent_dt : ") + std::to_string(mdat.Cent_dt * 1.0e3f) + std::string(", F_a : ") + std::to_string(adat.F_a) + std::string(", F_x : ") + std::to_string(adat.F_x) + std::string(", F_y : ") + std::to_string(adat.F_y) + "\n";

                //     Serial.print(v.c_str());
                // }

                // Serial.println("---- Raw timing ----");
                // while (Timing_buffer.n_elem > 0)
                // {
                //     auto pdat = Timing_buffer.pop();

                //     std::string v = std::string("ID : ") + std::to_string(pdat.rid) + std::string(", t_mid : ") + std::to_string(pdat.time[0]) + std::string(", t_left : ") + std::to_string(pdat.time[1]) + std::string(", t_right : ") + std::to_string(pdat.time[2]) + "\n";

                //     Serial.print(v.c_str());
                // }

                // Serial.println("---- Motor timing data ----");
                // while (Motor_buffer.n_elem > 0)
                // {
                //     auto fdat = Motor_buffer.pop();

                //     std::string v = std::string("update_t : ") + std::to_string(fdat.time) + std::string(", spd : ") + std::to_string(fdat.speed) + "\n";

                //     Serial.print(v.c_str());
                // }

                // Serial.println("---- Motor feedback data ----");
                // while (Motor_buffer.n_elem > 0)
                // {
                //     auto fdat = Motor_buffer.pop();

                //     std::string v = std::string("update_t : ") + std::to_string(fdat.time) + std::string(", spd_low : ") + std::to_string(fdat.spd_low) + std::string(", spd_high : ") + std::to_string(fdat.spd_high) + std::string(", meas_spd_low : ") + std::to_string(fdat.meas_spd_low) + std::string(", meas_spd_high : ") + std::to_string(fdat.meas_spd_high) + std::string(", spd_angle : ") + std::to_string(fdat.spd_angle) + "\n";

                //     Serial.print(v.c_str());
                // }

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

void Buffer_EKF_task(void *pvParameters)
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int64_t t_now = esp_timer_get_time();
        EKF_all pos;

        pos.time = t_now;
        pos.st = EKF::Get_state(t_now, EKF::drone_mass);

        EKF_buffer.push(pos);
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

void Buffer_raw_data_task(void *pvParameters)
{
    int64_t last_access_time = 0;

    while (true)
    {
        // wait till next message is received
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // get data
        uint32_t io_flag;

        IR::RX::Msg_timing_t temp[100];
        size_t temp_len;

        do
        {
            io_flag = IR::RX::Get_io_flag();

            // check if there's enough data to use
            temp_len = IR::RX::Get_timing_data(temp, min(esp_timer_get_time() - last_access_time + 1000, 50000LL));

        } while (io_flag != IR::RX::Get_io_flag());

        if (temp_len != 0)
        {
            int64_t last_access_time_new = max(max(temp[0].time_arr[0], temp[0].time_arr[1]), temp[0].time_arr[2]);
            if (last_access_time_new > last_access_time)
            {
                LIT_G;
                delay(1);
                QUENCH_G;

                Timing_data_short ts;
                ts.rid = temp[0].robot_ID;
                ts.time[0] = temp[0].time_arr[0];
                ts.time[1] = temp[0].time_arr[1];
                ts.time[2] = temp[0].time_arr[2];

                Timing_buffer.push(ts);

                last_access_time = last_access_time_new;
            }
        }
    }
}

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
    // struct LED_timing_t
    // {
    //     int64_t t_start;
    //     int64_t duration;
    //     int64_t period;
    //     float intensity_on = 1.0F;
    //     float intensity_off = 0.0F;
    // };

    // /**
    //  * @brief relative angle of LED
    //  */
    // constexpr float LED_angle_offset = 0.88187194F;

    // struct LED_callback_args
    // {
    //     bool LED_state;                      // current state of LED, 0 (initial state) for off, 1 for on
    //     LED_timing_t Last_LED_timing;        // last timing info used by the callback
    //     int64_t Last_trig_time;              // last time we updated the trigger info
    //     int64_t Next_trig_time;              // when will we trig next time
    //     int LED_num;                         // 0,1,2 for R,G,B respectively
    //     Circbuffer<LED_timing_t, 5> *buffer; // timing info buffer
    //     esp_timer_handle_t handle;           // callback handler
    // };

    // LED_callback_args callback_args_1, callback_args_2, callback_args_3;
    // Circbuffer<LED_timing_t, 5> timing_buf_1, timing_buf_2, timing_buf_3;

    // /**
    //  * @brief a task that switch on and off LED based on robot's position
    //  *
    //  * @param void* v_arg arguments
    //  */
    // void IRAM_ATTR LED_callback(void *v_arg)
    // {
    //     uint64_t t_now = esp_timer_get_time();

    //     LED_callback_args *arg = (LED_callback_args *)v_arg;

    //     // how long should we delay before next switch
    //     int64_t next_time;

    //     // this is for indication of XY position feedback.
    //     // if currently LED is on, then we should turn it off, and also setup the
    //     // time when it should be on again. Let's use green LED only here.
    //     if (arg->LED_state)
    //     {
    //         LED_set(arg->LED_num, arg->Last_LED_timing.intensity_off);
    //         arg->LED_state = false;

    //         // get localization data
    //         LED_timing_t temp = arg->buffer->peek_tail();
    //         // now this temp is used, update Last_LED_timing
    //         arg->Last_LED_timing = temp;

    //         // update time
    //         // make sure that we won't trigger again in 1/4 cycles
    //         next_time = ((temp.t_start - (t_now % temp.period) - (temp.period >> 2)) % temp.period) + (temp.period >> 2);
    //     }
    //     else
    //     {
    //         LED_set(arg->LED_num, arg->Last_LED_timing.intensity_on);
    //         arg->LED_state = true;
    //         next_time = arg->Last_LED_timing.duration;
    //     }

    //     // reset timer
    //     arg->Last_trig_time = t_now;
    //     arg->Next_trig_time += next_time;

    //     // not sure if this is necessary, but we will add it anyways, it's not gonna influence anything.
    //     next_time = arg->Next_trig_time - esp_timer_get_time();
    //     next_time = (next_time < Delay_min_time) ? Delay_min_time : next_time;

    //     esp_timer_stop(arg->handle);
    //     esp_timer_start_once(arg->handle, next_time);
    // }

    /**
     * @brief relative angle of motor
     */
    constexpr float Motor_angle_offset = 0.174533F;

    /**
     * @brief minimum delay time, if lower than this, we trigger after this.
     */
    constexpr int64_t Delay_min_time = 500LL;

    /**
     * @brief maximum delay time, if higher than this, we trigger after this.
     */
    constexpr int64_t Delay_max_time = 30000LL;

    struct Motor_timing_t
    {
        int64_t t_start;          // when we should switch the motor to spd_high, absolute time.
        int64_t period;           // period of rotation
        uint16_t spd_low = 0.0F;  // speed low end
        uint16_t spd_high = 0.0F; // speed high end

        float thrust_angle = 0.0F; // at which angle the thrust is high
    };

    struct Motor_control_info
    {
        uint32_t Motor_state = 0;                          // current state of motor
        Motor_timing_t Current_motor_target;               // current motor control target
        Circbuffer<Motor_timing_t, 5> Motor_timing_buffer; // timing info buffer
        esp_timer_handle_t timer_handle;                   // timer handler
    };

    Motor_control_info Motor_control_info_opt, Motor_control_info_EKF;

    int64_t Last_position_update_time = 0;
    int64_t Reach_target_speed_time = 0;
    bool Feedback_enabled = true;

    // pre-control actions
    // 0 for [0,speed1), 1 for [speed1,speed2), 2 for [speed2,inf), 3 for other actions
    int control_on = 0;

    /**
     * @brief a task that turn motor based on robot's position
     *
     * @param void* v_arg arguments
     */
    void IRAM_ATTR Motor_callback(void *arg)
    {
        // pointer to the correct command slot
        static Motor_control_info *command_ptr = &Motor_control_info_opt;

        int64_t t_now = esp_timer_get_time();

        // for arg -> Motor_state
        // state = 0 -> before pulse
        // state = 1 -> in pulse
        // state = 2 -> after pulse

        // whether we've already switched to the EKF based controller already
        static bool not_switched = true;

        // store timing for state 2
        static int64_t timing_temp = 0;

        // thrust to be set, 0 -> low, 1 -> high
        static bool to_set_thrust = 0;

        // measured speed
        // 0 is low, 1 is high
        static uint16_t meas_spd[2];

        // how long should we delay before next switch
        int64_t next_time;

        Motor_timing_t temp, lmt;

        // state = 0 -> before pulse
        // state = 1 -> in pulse
        // state = 2 -> after pulse
        // always cycle through these three
        switch (command_ptr->Motor_state)
        {
        case 0: // if was 0, the next time period should be in pulse
            to_set_thrust = !to_set_thrust;
            next_time = command_ptr->Current_motor_target.period / 2;
            command_ptr->Motor_state = 1;
            break;

        case 1: // if was 1, the next time period should be after pulse
            to_set_thrust = !to_set_thrust;
            next_time = timing_temp;
            command_ptr->Motor_state = 2;
            break;

        case 2: // if was 2, the next time period should be new before pulse
            // get latest motor timing data

            // lmt is short for last motor target
            lmt = command_ptr->Current_motor_target;

            // update next round's info
            if (Feedback_enabled)
            {
                // switch to EKF for input if necessary
                if (not_switched && control_on == 3)
                {
                    not_switched = false;
                    command_ptr = &Motor_control_info_EKF;
                    Motor_control_info_EKF.timer_handle = Motor_control_info_opt.timer_handle;
                }

                temp = command_ptr->Motor_timing_buffer.peek_tail();
                // now this temp is used, update Last_LED_timing
                command_ptr->Current_motor_target = temp;
            }
            else
            {
                temp = command_ptr->Current_motor_target;
            }

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
            command_ptr->Motor_state = 0;
            break;

        default:
            break;
        }

        // if started and not ended, we record the FB info
        if (Reach_target_speed_time && !Motor::Get_brake_status())
        {
            LED_set(1, to_set_thrust);

            Motor::Set_speed(to_set_thrust ? command_ptr->Current_motor_target.spd_high : command_ptr->Current_motor_target.spd_low);
           // Motor_buffer.push({to_set_thrust ? command_ptr->Current_motor_target.spd_high : command_ptr->Current_motor_target.spd_low, uint32_t(t_now)});

            // if was 2, we push to storage
            if (command_ptr->Motor_state == 0)
            {
                // when to_set_thrust is 1, we set the motor to spd_low, but this is actually when the speed is the highest.
                // Motor_buffer.push(Motor_info_1{lmt.spd_low, lmt.spd_high, meas_spd[0], meas_spd[1], lmt.thrust_angle, static_cast<uint32_t>(t_now)});
                EKF::Motor_info_t tempinfo;
                tempinfo.spd_low = lmt.spd_low;
                tempinfo.spd_high = lmt.spd_high;
                tempinfo.spd_FB_low = meas_spd[0];
                tempinfo.spd_FB_high = meas_spd[1];
                tempinfo.thrust_angle = lmt.thrust_angle;
                tempinfo.stop_time = t_now;

                EKF::Push_to_motor_buffer(tempinfo);
                EKF::Notify_Localization_Task();
            }
            // if was 0 or 1, we measure the speed and store them
            else
            {
                meas_spd[to_set_thrust] = Motor::Measure_speed();
            }
        }

        // not sure if this is necessary, but we will add it anyways, it's not gonna influence anything.
        next_time -= esp_timer_get_time() - t_now;
        next_time = clip(next_time, Delay_min_time, Delay_max_time);

        esp_timer_stop(command_ptr->timer_handle);
        esp_timer_start_once(command_ptr->timer_handle, next_time);
    }
} // anonymous namespace

namespace
{
    // true for speed 1, false for speed 2
    bool motor_state = false;
    uint8_t motor_spd_1 = 0;
    uint8_t motor_spd_2 = 0;

    char ToHEX(uint8_t val)
    {
        return (val >= 10) ? 'A' + val - 10 : '0' + val;
    }

    struct motor_spd
    {
        uint32_t spd_1;
        uint32_t spd_2;
    };

    constexpr size_t motor_spd_buffer_max_size = 5;
    Circbuffer<motor_spd, motor_spd_buffer_max_size + 5> motor_spd_buffer;

    void IRAM_ATTR Motor_test_callback(void *pvParameters)
    {
        if (motor_state)
        {
            DEBUG_C(setbit(DEBUG_PIN_2));
            Motor::Set_speed(motor_spd_1);
            motor_spd_buffer.push(motor_spd{Motor::Measure_speed(), 0U});
        }
        else
        {
            DEBUG_C(clrbit(DEBUG_PIN_2));
            Motor::Set_speed(motor_spd_2);
            if (motor_spd_buffer.n_elem)
            {
                motor_spd_buffer.peek_tail().spd_2 = Motor::Measure_speed();
            }
        }
        motor_state = !motor_state;
    }
}

// test input mode can switch between serial I/O and RX I/O
// 0 -> Serial
// 1 -> RX
// 2 -> Auto scan
#define TEST_INPUT_MODE 2

void Motor_test_task(void *pvParameters)
{
    // launch task to change motor speed
    const esp_timer_create_args_t oneshot_timer_callback_args_1 = {
        .callback = &Motor_test_callback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "test"};

    esp_timer_handle_t tm_handle;
    esp_timer_create(&oneshot_timer_callback_args_1, &tm_handle);

    constexpr uint64_t motor_spd_switch_time = 20000;
    esp_timer_start_periodic(tm_handle, motor_spd_switch_time);

// change speed based on inputs
// Serial input
#if TEST_INPUT_MODE == 0
    while (true)
    {
        if (Serial.available())
        {
            vTaskDelay(10);
            char c = Serial.read();

            uint16_t tval1, tval2, reg, val;
            String hexString;

            switch (c)
            {
            // thrust value
            case 't':
                tval1 = Serial.parseInt();
                tval2 = Serial.parseInt();

                tval2 = (tval2 == 0) ? tval1 : tval2;

                if (tval1 >= 256 || tval2 >= 256)
                {
                    Serial.println("Invalid thrust!");
                    break;
                }

                Serial.print("Thrust => ");
                Serial.print(tval1);
                Serial.print(" , ");
                Serial.println(tval2);

                motor_spd_1 = tval1;
                motor_spd_2 = tval2;
                break;

            // register value
            case 'r':
                reg = Serial.parseInt();

                hexString = Serial.readStringUntil('\n');  // Read the HEX number from the serial port and convert it to a string
                val = strtol(hexString.c_str(), NULL, 16); // Convert the HEX string to an integer

                if (reg == 0)
                {
                    Serial.println("Reset!");
                    esp_restart();
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

                    Motor::Config_register(reg, val);
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

        if (motor_spd_buffer.n_elem > 1)
        {
            auto temp = motor_spd_buffer.pop();
            Serial.print(temp.spd_1);
            Serial.print(",");
            Serial.println(temp.spd_2);
        }

        vTaskDelay(10);
    }
// RX input
#elif TEST_INPUT_MODE == 1
    static bool just_start = 1;

    // get data
    auto buf_1 = IR::RX::Get_msg_buffer_by_type(1);
    auto buf_2 = IR::RX::Get_msg_buffer_by_type(2);

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

#elif TEST_INPUT_MODE == 2
    int64_t last_update_time = esp_timer_get_time();
    constexpr int64_t update_interval = 4000000LL;

    constexpr uint16_t tstart = 3;
    constexpr uint16_t tend = 24; // 25;
    constexpr uint16_t tstep = 3;
    uint16_t tval1 = tstart, tval2 = tstart - tstep;

    bool print_enabled = false;

    while (true)
    {
        if (esp_timer_get_time() - last_update_time >= update_interval)
        {
            last_update_time += update_interval;

            if (tval1 != 0)
            {
                // if (tval1 >= tend)
                // {
                //     tval1 = tval2 = 0;
                //     Motor::Active_brake();
                //     esp_timer_stop(tm_handle);
                //     print_enabled = true;
                //     while (Serial.available())
                //     {
                //         Serial.read();
                //     }
                // }
                // else
                // {
                //     tval1 = tval2 = tval1 + 1;
                // }

                if (tval2 >= tend)
                {
                    // stop if ended
                    if (tval1 >= tend)
                    {
                        tval1 = tval2 = 0;
                        Motor::Active_brake();
                        esp_timer_stop(tm_handle);
                    }
                    else
                    {
                        tval1 += tstep;
                        tval2 = tval1;
                    }
                }
                else
                {
                    tval2 += tstep;
                }

                motor_spd_1 = tval1;
                motor_spd_2 = tval2;

                Serial.print("---- Thrust => ");
                Serial.print(tval1);
                Serial.print(" , ");
                Serial.print(tval2);
                Serial.print(" ----\n");
            }
        }

        // if (print_enabled && Serial.available())
        // {
        //     while (Serial.available())
        //     {
        //         Serial.read();
        //     }

        //     while (motor_spd_buffer.n_elem > 1)
        //     {
        //         auto temp = motor_spd_buffer.pop();
        //         Serial.print(temp.spd_1);
        //         Serial.print(",");
        //         Serial.println(temp.spd_2);
        //     }
        // }

        while (motor_spd_buffer.n_elem > 1)
        {
            auto temp = motor_spd_buffer.pop();
            Serial.print(temp.spd_1);
            Serial.print(",");
            Serial.println(temp.spd_2);
        }

        vTaskDelay(10);
    }
#endif
}

// I_comp is shared across multiple
float I_comp[3] = {0.0F, 0.0F, 0.0F};

void Motor_control_task_opt(void *pvParameters)
{
    const float cos_rot = cos(P_rot - K_rot);
    const float sin_rot = sin(P_rot - K_rot);

    // position data is valid for 1s
    constexpr int64_t Position_expire_time = 1000000LL;

    bool ISR_started = false;

    // PID components
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
            if (IR::TX::TX_enabled())
            {
                IR::TX::Add_to_schedule(4, {std::bit_cast<uint16_t>((int16_t)(filt_pos_0.x)), std::bit_cast<uint16_t>((int16_t)(filt_pos_0.y)), std::bit_cast<uint16_t>((int16_t)(filt_pos_0.z))}, 2);
            }
        }

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

        case 2:
            if (esp_timer_get_time() - Reach_target_speed_time > 10000000LL)
            {
                // LIT_R;
                T_switch_EKF = esp_timer_get_time();

                control_on = 3;
            }
            break;

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

        if (integration_on && control_on != 3)
        {
            t_coef = float(pos_0.time - pos_1.time) / 2.0e6F;
            I_comp[0] += t_coef * (pos_0.x + pos_1.x - 2.0F * target_point[0]);
            I_comp[1] += t_coef * (pos_0.y + pos_1.y - 2.0F * target_point[1]);
            I_comp[2] += t_coef * (pos_0.z + pos_1.z - 2.0F * target_point[2]);

            // add a constraint to horizontal I comp magnitude
            float imag = norm(I_comp[0], I_comp[1]);
            float imag_mult = clip(imag, 0.0F, I_XY_range) / (imag + 1.0e-6F);
            I_comp[0] *= imag_mult;
            I_comp[1] *= imag_mult;
        }

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

        // static float FB_filtered = 0.0F;
        // static uint32_t FB_count = 0U;
        // if (control_on == 2 && esp_timer_get_time() - Reach_target_speed_time >= 8000000LL)
        // {
        //     FB_filtered += FB_val[2];
        //     FB_count++;
        // }

        // if (control_on == 3)
        // {
        //     if (esp_timer_get_time() - T_switch_EKF <= 500000LL)
        //     {
        //         FB_val[0] = 0.0F;
        //         FB_val[1] = -8.0F;
        //         FB_val[2] = FB_filtered / FB_count + 0.3F;
        //     }
        //     else
        //     {
        //         FB_val[0] = 0.0F;
        //         FB_val[1] = 8.0F;
        //         FB_val[2] = FB_filtered / FB_count + 0.3F;
        //     }
        // }

        // all computation has been finished till now
        // from now on it's the boring setup interrupt part

        // cap the duration!
        constexpr int64_t Duration_max_time = 25000;

        // compute timing for callback_3 that use LED to indicate feedback value
        Motor_timing_t Callback_dat_m;

        // each cycle start with low->high
        Callback_dat_m.period = pos_0.rotation_time;
        Callback_dat_m.t_start = int64_t((atan2f(FB_val[1], FB_val[0]) + K_rot - Motor_angle_offset - pos_0.angle_0 + 10.5F * M_PI) / pos_0.angular_velocity);

        // FB[0~2] is F_a, Delta F, and angle.
        if (fabs(FB_val[1]) <= 1.0e-5f && fabs(FB_val[0]) <= 1.0e-5f)
        {
            Callback_dat_m.thrust_angle = 0.0f;
        }
        else
        {
            Callback_dat_m.thrust_angle = atan2f(FB_val[1], FB_val[0]);
        }

        // clip add_val to make sure that the average is still the average and the values will not exceed limits.
        // similar for motor settings.

        // It would be better if the thrust can be computed in the interrupt so we can carry on the residue to make things more accurate, but for now let's just compute it here.

        // min and max speed command
        constexpr uint32_t spd_max = 25U, spd_min = 7U;

        uint32_t spd_high, spd_low, spd_diff;
        uint32_t spd_avg_2 = clip(uint32_t(ceil(FB_val[2] * 1.0625F + 10.0F)), spd_min * 2, spd_max * 2);
        spd_low = spd_avg_2 / 2;
        spd_high = spd_avg_2 - spd_low;

        spd_diff = spd_high - spd_low;
        spd_diff = min(min(uint32_t(round((1.0625F * norm(FB_val[0], FB_val[1]) - spd_diff) / 2.0F)), spd_max - spd_high), spd_low - spd_min);

        Callback_dat_m.spd_low = spd_low - spd_diff;
        Callback_dat_m.spd_high = spd_high + spd_diff;

        // push inside buffer
        Motor_control_info_opt.Motor_timing_buffer.push(Callback_dat_m);

        // check if first time, if so, start ISR
        if (!ISR_started)
        {
            ISR_started = true;

            // Callback/ISR 3
            Motor_control_info_opt.Current_motor_target = Callback_dat_m;
            Motor_control_info_opt.Motor_state = 0;

            const esp_timer_create_args_t oneshot_timer_callback_args_m = {
                .callback = &Motor_callback,
                .arg = nullptr,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "motor"};

            esp_timer_create(&oneshot_timer_callback_args_m, &(Motor_control_info_opt.timer_handle));
            esp_timer_start_once(Motor_control_info_opt.timer_handle, 50U);
        }

        // how long before/after the trigger time should we prevent triggering
        // in us
        constexpr uint64_t No_operation_time = 200ULL;
        int64_t t_delay;
    }
}

void Motor_control_task_EKF(void *pvParameters)
{
    constexpr float EKF_scaling = 10.0f;

    // reset I component indicator
    bool reset_I = true;

    // PID components
    int64_t last_I_time = 0.0f;
    float last_P_comp[3] = {0.0F, 0.0F, 0.0F};

    // float I_comp[3] = {0.0F, 0.0F, 0.0F};
    float P_comp[3] = {0.0F, 0.0F, 0.0F};
    float D_comp[3] = {0.0F, 0.0F, 0.0F};
    float A_comp[3] = {0.0F, 0.0F, 0.0F};
    float FB_val[3] = {0.0F, 0.0F, 0.0F};

    while (true)
    {
        // wait till next EKF update is done
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Last_position_update_time = esp_timer_get_time();

        // get angular velocity
        auto ang_v = EKF::Get_last_angular_velocity();
        int64_t t_predict = esp_timer_get_time() + int64_t(3.0F * 360.0F / ang_v * 1.0e6F);
        // get state estimation for 1.5 cycles later
        auto st = EKF::Get_state(t_predict, EKF::drone_mass, nullptr);
        // use angular velocity in predicted data
        // note that the slight difference in angular velocity won't influence the prediction by much
        // but could have an impact on rotation control
        ang_v = st.state[EKF::state_para::omega];

        int64_t rotation_period = int64_t(360.0F / ang_v * 1.0e6F);

        P_comp[0] = EKF_scaling * st.state[EKF::state_para::x] - target_point[0];
        P_comp[1] = EKF_scaling * st.state[EKF::state_para::y] - target_point[1];
        P_comp[2] = EKF_scaling * st.state[EKF::state_para::z] - target_point[2];

        // add a constraint to horizontal P comp magnitude
        float pmag = norm(P_comp[0], P_comp[1]);
        float pmag_mult = clip(pmag, 0.0F, P_XY_range) / (pmag + 1.0e-6F);
        P_comp[0] *= pmag_mult;
        P_comp[1] *= pmag_mult;

        D_comp[0] = EKF_scaling * st.state[EKF::state_para::v_x];
        D_comp[1] = EKF_scaling * st.state[EKF::state_para::v_y];
        D_comp[2] = EKF_scaling * st.state[EKF::state_para::v_z];

        A_comp[0] = st.state[EKF::state_para::gamma_x] * float(M_PI) / 180.0f * 9800.0f;
        A_comp[1] = st.state[EKF::state_para::gamma_y] * float(M_PI) / 180.0f * 9800.0f;

        if (control_on == 3)
        {
            float t_coef = float(t_predict - last_I_time) / 2.0e6F;

            I_comp[0] += t_coef * (P_comp[0] + last_P_comp[0]);
            I_comp[1] += t_coef * (P_comp[1] + last_P_comp[1]);
            I_comp[2] += t_coef * (P_comp[2] + last_P_comp[2]);

            // add a constraint to horizontal I comp magnitude
            float imag = norm(I_comp[0], I_comp[1]);
            float imag_mult = clip(imag, 0.0F, I_XY_range) / (imag + 1.0e-6F);
            I_comp[0] *= imag_mult;
            I_comp[1] *= imag_mult;
        }

        last_P_comp[0] = P_comp[0];
        last_P_comp[1] = P_comp[1];
        last_P_comp[2] = P_comp[2];
        last_I_time = t_predict;

        // construct X,Y,Z control law
        FB_val[0] = -K_P_XY * P_comp[0] - K_D_XY * D_comp[0] - K_A_XY * A_comp[0] - K_I_XY * I_comp[0];
        FB_val[1] = -K_P_XY * P_comp[1] - K_D_XY * D_comp[1] - K_A_XY * A_comp[1] - K_I_XY * I_comp[1];
        FB_val[2] = -K_P_Z * P_comp[2] - K_D_Z * D_comp[2] - K_I_Z * I_comp[2] + Robot_mass;

        // if (norm(EKF_scaling * st.state[EKF::state_para::x] - target_point[0], EKF_scaling * st.state[EKF::state_para::y] - target_point[1]) > 200.0F)
        // {
        //     FB_val[0] = -P_comp[0];
        //     FB_val[1] = -P_comp[1];
        // }

        // compute timing for callback_3 that use LED to indicate feedback value
        Motor_timing_t Callback_dat_m;

        // each cycle start with low->high
        Callback_dat_m.period = rotation_period;
        Callback_dat_m.t_start = t_predict + int64_t((atan2f(FB_val[1], FB_val[0]) + K_rot - Motor_angle_offset - st.state[EKF::state_para::theta] * float(M_PI) / 180.0f + 10.5F * M_PI) / (ang_v * float(M_PI) / 180.0f) * 1.0e6f);

        // FB[0~2] is F_a, Delta F, and angle.
        if (fabs(FB_val[1]) <= 1.0e-5f && fabs(FB_val[0]) <= 1.0e-5f)
        {
            Callback_dat_m.thrust_angle = 0.0f;
        }
        else
        {
            Callback_dat_m.thrust_angle = atan2f(FB_val[1], FB_val[0]);
        }

        // clip add_val to make sure that the average is still the average and the values will not exceed limits.
        // similar for motor settings.

        // It would be better if the thrust can be computed in the interrupt so we can carry on the residue to make things more accurate, but for now let's just compute it here.

        // min and max speed command
        constexpr uint32_t spd_max = 25U, spd_min = 7U;

        uint32_t spd_high, spd_low, spd_diff;
        uint32_t spd_avg_2 = clip(uint32_t(ceil(FB_val[2] * 1.0625F + 10.0F)), spd_min * 2, spd_max * 2);
        spd_low = spd_avg_2 / 2;
        spd_high = spd_avg_2 - spd_low;

        spd_diff = spd_high - spd_low;
        spd_diff = min(min(uint32_t(round((1.0625F * norm(FB_val[0], FB_val[1]) - spd_diff) / 2.0F)), spd_max - spd_high), spd_low - spd_min);

        Callback_dat_m.spd_low = spd_low - spd_diff;
        Callback_dat_m.spd_high = spd_high + spd_diff;

        // push inside buffer
        Motor_control_info_EKF.Motor_timing_buffer.push(Callback_dat_m);

        // // register Callback information for printing
        // Callback_dat_m.period = esp_timer_get_time();
        // Motor_timing_buffer.push(Callback_dat_m);
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
            else if (t_now - Last_position_update_time >= Power_low_threshold)
            {
                Feedback_enabled = false;

                Motor_control_info_opt.Current_motor_target.spd_low = 15U;
                Motor_control_info_opt.Current_motor_target.spd_high = 15U;
                Motor_control_info_opt.Current_motor_target.thrust_angle = 0.0F;

                Motor_control_info_EKF.Current_motor_target.spd_low = 15U;
                Motor_control_info_EKF.Current_motor_target.spd_high = 15U;
                Motor_control_info_EKF.Current_motor_target.thrust_angle = 0.0F;

                Motor::Set_thrust(Power_low_value * Robot_mass);
            }
            else
            {
                Feedback_enabled = true;
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