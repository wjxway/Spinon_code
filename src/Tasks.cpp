#include "Tasks.hpp"
#include "RobotDefs.hpp"
#include "Utilities/FastIO.hpp"
#include "Utilities/FastMath.hpp"
#include "Utilities/DebugDefs.hpp"
#include "Utilities/FeedTheDog.hpp"
#include "Utilities/Circbuffer.hpp"
#include "IrCommunication/IrRX.hpp"
// #include "MotorCtrl/MotorCtrl.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <math.h>

// add the definition of bit_cast (a pretty unsafe version) if cpp version is
// old.
#if __cplusplus < 202002L
namespace std
{
    // bit cast, but a really unsafe version.
    // I don't know why some macros cannot work here, so it's up to the user to
    // make sure that it can work
    template <class To, class From>
    To bit_cast(const From &src) noexcept
    {
        To dst;
        memcpy(&dst, &src, sizeof(To));
        return dst;
    }
}
#endif

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

void LED_off_task(void *pvParameters)
{
    TickType_t prev_wake_time = xTaskGetTickCount();

    while (1)
    {
        // still need to feed the dog
        Feed_the_dog();

        // turn off led if they haven't been refreshed for a while
        QUENCH_R;
        QUENCH_G;
        QUENCH_B;

        vTaskDelayUntil(&prev_wake_time, pdMS_TO_TICKS(1));
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

void Send_message_task(void *pvParameters)
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
            msg_1 = IR::RX::Get_latest_msg_by_type(1);
            msg_4 = IR::RX::Get_latest_msg_by_type(4);
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
            // print latest msg of type 1
            temp += "\nLatest type 1 message is sent by robot " + std::to_string(msg_1.robot_ID) + " : " + n2hexstr(msg_1.content[0]);
            temp += "\n    last received at: " + std::to_string(msg_1.finish_reception_time);
        }
        else
        {
            temp += "\nNo message of type 1 received.";
        }

        if (msg_4.content_length)
        {
            // print first robot's latest msg of type 4
            temp += "\nLatest type 4 message is sent by robot " + std::to_string(msg_4.robot_ID) + " and has length of: " + std::to_string(msg_4.content_length) + "\n The contents are: ";

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
                temp += "\n" + std::to_string(tarr[i].robot_ID) + "@" + std::to_string(tarr[i].emitter_pos) + " : " + std::to_string(tarr[i].time_arr[0]) + "," + std::to_string(tarr[i].time_arr[1]) + "," + std::to_string(tarr[i].time_arr[2]);
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

// type of message that carries localization data
constexpr uint32_t Localization_Msg_type = 4;

// the angular standard deviation (of relative angle measurements)
constexpr float Angle_error = 1.0f / 180.0f * M_PI;

// error of relative angle (this is not the same as Angle_error, because we
// use this for angle difference between robots, and this will be influenced by
// the rotation speed variance.)
constexpr float Relative_angle_error = 4.0f / 180.0f * M_PI;

// tilting angle of center emitter, 1 / tangent value.
// it is 25deg right now, so we should put 1 / tan(25deg) here.
constexpr float Tilting_angle_multiplyer = 2.14450692051f;

// distance between left and right receiver
constexpr float Left_right_distance = 51.562f;
// just a constant
constexpr float Distance_error_mult = Angle_error / Left_right_distance;
/**
 * @brief compute distance error based on the distance
 *
 * @param distance distance expectation
 * @return constexpr uint16_t error estimation
 */
constexpr float Distance_error(const float distance)
{
    return Distance_error_mult * distance * distance;
}

/**
 * @brief compute elevation error based on the distance and elevation
 *
 * @param distance distance expectation
 * @param elevation elevation expectation
 * @return constexpr uint32_t error estimation
 */
constexpr float Elevation_error(const float distance, const float elevation)
{
    return (Tilting_angle_multiplyer * Angle_error * (distance + elevation * elevation / distance) + abs(elevation) * Distance_error_mult * distance);
}

// information needed to compute the position and orientation of this robot.
// Position, distance and elevation are all in unit of mm, angle in unit of rad.
// Note that here we assume that the position is accurate!
typedef struct
{
    float pos[3];   // position of reference robot
    float dist;     // horizontal distance between robots
    float dist_err; // error of dist
    float elev;     // vertical distance (h of this - h of reference)
    float elev_err; // error of elev
    float angle;    // angle computed by 2*Pi*(time%T_0)/T_0.
    float angle_err;
} Relative_position_data;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    float angle_0;
    uint64_t rotation_time; // used to pre-treat the uint64_t to prevent clipping.
    float angular_velocity;
} Position_data;

/**
 * @brief round an angle to [-Pi, Pi) range
 *
 * @param v input angle
 * @return float value rounded to [-Pi, Pi) range
 */
float Angle_diff(const float v) noexcept
{
    float v1 = v / float(2.0f * M_PI);
    v1 = v1 - math::fast::floor(v1 + 0.5f);
    return v1 * float(2.0f * M_PI);
}

constexpr float square(const float x) noexcept
{
    return x * x;
}

// /**
//  * @brief Compute the error between estimated state and measurements
//  *
//  * @param pos estimated positon
//  * @param data measurements
//  * @param data_len length of measurements
//  * @return float error
//  *
//  * @note here we only compute the error induced by horizontal position and
//  * rotation, because the error introduced by elevation is trivially computable.
//  */
// float Localization_error(const Position_data pos, Relative_position_data *const data, const size_t data_len)
// {
//     float error = 0;
//
//     for (size_t i = 0; i < data_len; i++)
//     {
//         // add distance error
//         float dx = data[i].pos[0] - pos.x, dy = data[i].pos[1] - pos.y;
//         error += square(sqrtf(dx * dx + dy * dy) - data[i].dist) / square(data[i].dist_err);
//
//         // add angular error
//         float da = Angle_diff(atan2f(dy, dx) - data[i].angle - pos.angle_0) / data[i].angle_err;
//         error += da * da;
//     }
//
//     return error;
// }

/**
 * @brief Execute localization based on relative measurement data
 *
 * @param data input data array
 * @param data_len length of data array
 */
Position_data Execute_localization(Relative_position_data *const data, const size_t data_len)
{
    Position_data temp;

    // determine the height, because it's independent from others, it's simple.
    float deno = 0.0f, nume = 0.0f;
    for (size_t i = 0; i < data_len; i++)
    {
        float c = 1.0f / (data[i].elev_err * data[i].elev_err);
        deno += c;
        nume += data[i].elev * c;
    }
    temp.z = nume / deno;

    // estimated {X,Y}
    float x = 0.0f, y = 0.0f, theta = 0.0f;

    // determine starting {X,Y} point by averaging all beacon position
    for (size_t i = 0; i < data_len; i++)
    {
        x += data[i].pos[0];
        y += data[i].pos[1];
    }
    x /= data_len;
    y /= data_len;

    // maximum iterations we do
    constexpr size_t Max_Position_Iterations = 10;
    // when step size dropped below this, we quit!
    constexpr float Error_tolerance = 1.0f;
    // determine the rest iteratively, max 10 times
    for (size_t step = 0; step < Max_Position_Iterations; step++)
    {
        // determine the angle first based on the X,Y we have.
        // assuming angle error are the same.
        // angsum is the summation of all angles between [-Pi, Pi), ang1sum is
        // the summation [0, 2Pi)
        // ang2sum is the summation of square of all angles ...
        float angsum = 0.0f, ang2sum = 0.0f, ang1sum = 0.0f, ang12sum = 0.0f;
        for (size_t i = 0; i < data_len; i++)
        {
            float ang = Angle_diff(atan2f(data[i].pos[1] - y, data[i].pos[0] - x) - data[i].angle);
            float ang1 = (ang < 0) ? (ang + 2.0f * float(M_PI)) : ang;
            angsum += ang;
            ang2sum += square(ang);
            ang1sum += ang1;
            ang12sum += square(ang1);
        }
        // compute variance * (data_len - 1)
        ang2sum = ang2sum - square(angsum) / float(data_len);
        ang12sum = ang12sum - square(ang1sum) / float(data_len);

        // the optimal angle is the smaller one of these two
        theta = ((ang2sum < ang12sum) ? angsum : ang1sum) / float(data_len);

        // now let's work on X,Y based on the rule that the optimal point should be at Inverse[Hessian].Grad(f)
        // we have an simple assumption of Hessian matrix, a constant diagonal matrix: a I
        // how much should we move this time and the Hessian value
        float Deltax = 0.0f, Deltay = 0.0f, Hessian = 0.0f;

        for (size_t i = 0; i < data_len; i++)
        {
            float dx = data[i].pos[0] - x, dy = data[i].pos[1] - y;
            float dr = sqrtf(square(dx) + square(dy));
            float angdiff = Angle_diff(atan2f(dy, dx) - theta - data[i].angle);

            float isq1 = 1.0f / square(data[i].dist_err), isq2 = 1.0f / square(data[i].dist_err);

            Hessian += 1.0f / square(data[i].angle_err * dr) + isq2;

            Deltax += 2.0f * dx * (1 - data[i].dist / dr) * isq1 - 2.0f * dy * angdiff * isq2;
            Deltay += 2.0f * dy * (1 - data[i].dist / dr) * isq1 + 2.0f * dx * angdiff * isq2;
        }
        x += Deltax / Hessian;
        y += Deltay / Hessian;
        if (abs(Deltax) <= Error_tolerance * Hessian && abs(Deltax) <= Error_tolerance * Hessian)
            break;
    }
    temp.x = x;
    temp.y = y;
    temp.angle_0 = theta;

    return temp;
}

/**
 * @brief a stack of positions
 *
 * @note because element is filled before we increment tail, it's safe to access
 * tail at any time.
 */
Circbuffer<Position_data, 5> Position_stack;

/**
 * @brief trigger timer for TX_ISR
 */
hw_timer_t *Trigger_timer;

/**
 * @brief a task that switch on and off LED based on robot's position
 */
void IRAM_ATTR FB_LED_ISR()
{

    DEBUG_C(setbit(DEBUG_PIN_1));

    // reset timer
    timerRestart(Trigger_timer);
    timerAlarmWrite(Trigger_timer, 1000000000ul, false);
    timerAlarmEnable(Trigger_timer);

    DEBUG_C(clrbit(DEBUG_PIN_1));
}

void FB_LED_Init()
{
    // config timer to transmit TX once in a while
    // we are using timer 3 to prevent confiction
    // timer ticks 1MHz
    Trigger_timer = timerBegin(IR_TX_trigger_timer_channel, 80, true);
    // add timer interrupt
    timerAttachInterrupt(Trigger_timer, &FB_LED_ISR, true);
    // trigger interrupt after some random time
    timerAlarmWrite(Trigger_timer, 1000000000ul, false);
    // timerAlarmWrite(Trigger_timer, 60, true);
    timerAlarmEnable(Trigger_timer);
}

void Localization_simple(void *pvParameters)
{
    // max history time we fetch data from. should be slightly higher than 2 rotations.
    // please use higher limit for this number.
    constexpr uint64_t Max_history_time = 130000ul;

    TickType_t prev_wake_time = xTaskGetTickCount();
    while (1)
    {
        // wait upfront, so we can use continue like return.
        vTaskDelayUntil(&prev_wake_time, pdMS_TO_TICKS(100));

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
                pos_list.push_back(IR::RX::Get_latest_msg_by_bot(robot_ID_list[i], Localization_Msg_type));
            }

            time_count = IR::RX::Get_timing_data(time_list, Max_history_time);
        } while (curr_flag != IR::RX::Get_io_flag());

        // processing

        // let's exploit the timing_valid_Q here to store the information about
        // whether we've processed the second newest transmission. 0 ->
        // processed, 1 -> not processed. We know that it's always 1 when we
        // retrive them anyways.

        // if finished=1, then directly quit because we've already scan through
        // last two rotations.
        bool finished = 0;
        // last useful robot's index, needs to be within the last rotation.
        size_t last_index = 0;
        // something used to coarsely estimate the rotation speed by averaging
        // the timing difference between seeing the same robot in two
        // consecutive rounds.
        uint64_t rotation_time = 0;
        uint32_t rotation_count = 0;
        // compute rotation speed.
        for (size_t i = 1; i < time_count; i++)
        {
            // iterate till either the previous message, or end before the last
            // rotation
            for (size_t j = 0; j < ((last_index) ? last_index : (i - 1)); j++)
            {
                // check if this a second-oldest message
                if (time_list[i].robot_ID == time_list[j].robot_ID)
                {
                    // if more than second oldest, break the whole cycle,
                    // because we've already run for two cycles.
                    if (!time_list[j].timing_valid_Q)
                    {
                        finished = 1;
                    }
                    // if exactly the second oldest, then use it to compute the
                    // rotation speed.
                    else
                    {
                        // if first time we encounter a duplicate, then we
                        // record this, and from now on we only care about
                        // robots smaller than this count.
                        if (!last_index)
                        {
                            last_index = i;
                        }
                        // compute rotation time by averaging the timing
                        // difference of left and right channel.
                        // the reason why we didn't check for validity is
                        // because:
                        // 1. when we add to the timing_buffer, we've already
                        // made sure that two consecutive timing cannot be too
                        // short.
                        // 2. when taking data out of buffer, we never take
                        // message too old.
                        rotation_time += ((time_list[j].time_arr[1] - time_list[i].time_arr[1] + time_list[j].time_arr[2] - time_list[i].time_arr[2]) >> 1);
                        rotation_count++;
                        // j's second latest round has been recorded.
                        time_list[j].timing_valid_Q = 0;
                    }
                    // end finding j whenever a solution is found.
                    break;
                }
            }

            // if already finished, directly break.
            if (finished)
                break;
        }

        // compute rotation speed
        // if we cannot determine the rotation speed, just quit this round!
        if (!rotation_count)
        {
            DEBUG_C(Serial.println("Cannot determine the rotation speed."));
            continue;
        }
        // if we can, compute the rotation speed.
        float angular_velocity = 2.0f * float(M_PI) * float(rotation_count) / float(rotation_time);
        // also record single round time.
        rotation_time /= rotation_count;

        // the minimum tolerable timing is 5 degrees
        uint64_t Min_valid_time = rotation_time / 72;
        // the maximum tolerable timing is 60 degrees
        uint64_t Max_valid_time = rotation_time / 6;

        // now we collect the data useful for localization.

        // data directly used for localization
        Relative_position_data Loc_data[last_index];
        // how many elements in Loc_data. Note that this is not necessarily
        // equivalent to last_index, because some might not have position info
        // up to date.
        size_t Loc_data_count = 0;

        // add to Loc_data
        for (size_t i = 0; i < last_index; i++)
        {
            // position found?
            bool position_found = 0;
            // robot ID
            uint32_t rid = time_list[i].robot_ID;
            // search for robot ID in pos_list and check for its validity
            for (auto &msg : pos_list)
            {
                // find robot and update content
                if (msg.robot_ID == rid)
                {
                    Loc_data[Loc_data_count].pos[0] = std::bit_cast<int16_t>(msg.content[0]);
                    Loc_data[Loc_data_count].pos[1] = std::bit_cast<int16_t>(msg.content[1]);
                    Loc_data[Loc_data_count].pos[2] = std::bit_cast<int16_t>(msg.content[2]);
                    position_found = 1;
                    break;
                }
            }
            // if we found the position, setup the data
            if (position_found)
            {
                // check if timing data is reasonable
                uint64_t t_diff = time_list[i].time_arr[1] - time_list[i].time_arr[2];
                // might need to change the sign here!
                if (time_list[i].time_arr[1] > time_list[i].time_arr[2] && t_diff > Min_valid_time && t_diff < Max_valid_time)
                {
                    Loc_data[Loc_data_count].angle = float(((time_list[i].time_arr[1] + time_list[i].time_arr[2]) >> 1) % rotation_time) * angular_velocity;
                    Loc_data[Loc_data_count].angle_err = Relative_angle_error;
                    Loc_data[Loc_data_count].dist = (Left_right_distance / (2.0f * sin(float(time_list[i].time_arr[1] - time_list[i].time_arr[2]) * angular_velocity * 0.5f)));
                    Loc_data[Loc_data_count].dist_err = Distance_error(Loc_data[Loc_data_count].dist);
                    Loc_data[Loc_data_count].elev = Loc_data[Loc_data_count].dist * tan((time_list[i].time_arr[0] - ((time_list[i].time_arr[1] + time_list[i].time_arr[2]) >> 1)) * angular_velocity * Tilting_angle_multiplyer);
                    Loc_data[Loc_data_count].elev_err = Elevation_error(Loc_data[Loc_data_count].dist, Loc_data[Loc_data_count].elev);

                    Loc_data_count++;
                }
            }
        }

        if (Loc_data_count > 1)
        {
            Position_data res = Execute_localization(Loc_data, Loc_data_count);
            res.rotation_time = rotation_time;
            res.angular_velocity = angular_velocity;
            // do something with the res, add it to a queue or something else.
            // we can setup the interrupt here and let it turn on/off the lights.
            Position_stack.push(res);
        }
    }
}
