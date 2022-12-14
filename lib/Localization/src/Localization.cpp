#include "Localization.hpp"
#include "Arduino.h"
#include <RobotDefs.hpp>
#include <Utilities.hpp>
#include <iostream>
#include <string>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

using math::fast::square;

namespace IR
{
    namespace Localization
    {
        namespace
        {

            // type of message that carries localization data
            uint32_t Localization_Msg_type = 4;

            /**
             * @brief the minimum tolerable timing is 6 degrees, corresponding
             * to a distance of ~50cm
             */
            constexpr float Min_valid_angle = 6.0F / 180.0F * M_PI;
            /**
             * @brief the maximum tolerable timing is 60 degrees, corresponding
             * to two robots touching each other.
             */
            constexpr float Max_valid_angle = 60.0F / 180.0F * M_PI;

            /**
             * @brief the angular standard deviation (of relative angle measurements)
             *
             * @note this is mostly determined by the randomness of when we
             * received a message, which is related to the consecutive TX time
             * and the geometry.
             */
            constexpr float Angle_error = 0.8F / 180.0F * M_PI;

            /**
             * @brief error of relative angle
             *
             * @note this is not the same as Angle_error, because we use this
             * for angle difference between robots, and this will be influenced
             * by the rotation speed variance.
             */
            constexpr float Relative_angle_error = 3.0F / 180.0F * M_PI;

            /**
             * @brief tilting angle of center emitter, 1 / tangent value. It is
             * 25deg right now, so we should put 1 / tan(25deg) here.
             */
            constexpr float Tilting_angle_multiplyer = 2.14450692051F;

            /**
             * @brief distance between left and right receiver in mm
             */
            constexpr float Left_right_distance = 51.562F;

            /**
             * @brief just a helper constant
             */
            constexpr float Distance_error_mult = Angle_error / Left_right_distance;

            /**
             * @brief The position error we are giving to the position per us.
             *        Let's assume that the movement speed is ~5cm/s.
             *
             * @note For now, we assume that the error increase rate is the same for all
             * three directions
             */
            constexpr float Error_increase_rate = 50e-6F;

            /**
             * @brief after how long will we stop to use the old localization
             * data as initial estimation.
             */
            constexpr int64_t Position_expire_time = 1000000L;

            /**
             * @brief max history time we fetch data from. should be slightly
             * higher than 2 rotations. please use higher limit for this number.
             */
            constexpr int64_t Relative_measurements_expire_time = 130000L;

            /**
             * @brief relative position information needed to compute the
             * position and orientation of this robot. Position, distance and
             * elevation are all in unit of mm, angle in unit of rad. Note that
             * here we assume that the position is accurate!
             */
            struct Relative_position_data
            {
                uint32_t Robot_ID; // ID of robot, should be redundant
                float pos[3];      // position of reference robot
                float dist;        // horizontal distance between robots
                float dist_err;    // error of dist
                float elev;        // vertical distance (h of this - h of reference)
                float elev_err;    // error of elev
                float angle;       // angle computed by 2*Pi*(time%T_0)/T_0.
                float angle_err;   // error of angle
            };

            /**
             * @brief round an angle to [-Pi, Pi) range
             *
             * @param v input angle
             * @return float value rounded to [-Pi, Pi) range
             */
            float Angle_diff(const float v) noexcept
            {
                float v1 = v / float(M_TWOPI);
                v1 = v1 - math::fast::floor(v1 + 0.5F);
                return v1 * float(M_TWOPI);
            }

            constexpr size_t Position_stack_length = 10U;

            /**
             * @brief a queue to store history of localization data
             */
            Circbuffer<Position_data, Position_stack_length> Position_stack;

            /**
             * @brief a queue to store history of filtered localization data
             */
            Circbuffer<Position_data, Position_stack_length> Filtered_position_stack;

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
             * @brief just a helper function to determine the average of angle.
             *
             * @param data input relative measurements data
             * @param x estimated x position
             * @param y estimated y position
             * @return float average angle
             *
             * @note angle average is not the simple average, but need to take the 2 Pi
             * cycle into consideration
             */
            float Angle_average(const std::vector<Relative_position_data> &data, const float x, const float y)
            {
                size_t data_len = data.size();
                // determine the angle first based on the X,Y we have.
                // assuming angle error are the same.
                // angsum is the summation of all angles between [-Pi, Pi), ang1sum is
                // the summation [0, 2Pi)
                // ang2sum is the summation of square of all angles ...
                float angsum = 0.0F;
                float ang2sum = 0.0F;
                float ang1sum = 0.0F;
                float ang12sum = 0.0F;
                for (size_t i = 0; i < data_len; i++)
                {
                    float ang = Angle_diff(atan2f(data[i].pos[1] - y, data[i].pos[0] - x) - data[i].angle);
                    float ang1 = (ang < 0) ? (ang + 2.0F * float(M_PI)) : ang;
                    angsum += ang;
                    ang2sum += square(ang);
                    ang1sum += ang1;
                    ang12sum += square(ang1);
                }
                // compute variance * (data_len - 1)
                ang2sum = ang2sum - square(angsum) / float(data_len);
                ang12sum = ang12sum - square(ang1sum) / float(data_len);

                // the optimal angle is the smaller one of these two
                return ((ang2sum < ang12sum) ? angsum : ang1sum) / float(data_len);
            }

            /**
             * @brief Execute localization based on relative measurement data
             *
             * @param data relative measurement data vector
             * @param x_start where shall we start in optimization (x)
             * @param y_start where shall we start in optimization (y)
             */
            Position_data Execute_localization(const std::vector<Relative_position_data> &data, const float x_start, const float y_start)
            {
                const size_t data_len = data.size();

                Position_data temp;

                // determine the height, because it's independent from others, it's simple.
                // deno is 1e-10 to prevent 1/0.
                float deno = 1e-10F;
                float nume = 0.0F;
                for (size_t i = 0; i < data_len; i++)
                {
                    // only take into account of those with elevation measurements.
                    if (data[i].dist_err != 0.0F)
                    {
                        float c = 1.0F / square(data[i].elev_err);
                        deno += c;
                        nume += data[i].elev * c;
                    }
                }
                temp.z = nume / deno;
                // variance of this measurement
                temp.var_z = 1.0F / deno;

                // estimated {X,Y}
                // starting from the last position
                // or you can start from the average of all beacon positions if you wish...
                // might even give you better stability.
                float x = x_start;
                float y = y_start;
                float theta;

                // // determine starting {X,Y} point by averaging all beacon position
                // for (size_t i = 0; i < data_len; i++)
                // {
                //     x += data[i].pos[0];
                //     y += data[i].pos[1];
                // }
                // x /= data_len;
                // y /= data_len;

                // maximum iterations we do
                constexpr size_t Max_Position_Iterations = 10;
                // when step size dropped below this, we quit!
                constexpr float Error_tolerance = 0.5F;

                // determine the rest iteratively, max Max_Position_Iterations times
                for (size_t step = 0; step < Max_Position_Iterations; step++)
                {
                    // compute data average based on current position
                    theta = Angle_average(data, x, y);

                    // now let's work on X,Y based on the rule that the optimal point should
                    // be at Inverse[Hessian].Grad(f) we have an simple assumption of
                    // Hessian matrix, a constant diagonal matrix: a I how much should we
                    // move this time and the Hessian value note that in this step, we only
                    // compute the error lead by measurements with distance.
                    float Deltax = 0.0F;
                    float Deltay = 0.0F;
                    float Hessian = 0.0F;

                    for (size_t i = 0; i < data_len; i++)
                    {
                        float dx = data[i].pos[0] - x;
                        float dy = data[i].pos[1] - y;
                        float dr = sqrtf(square(dx) + square(dy));
                        float angdiff = Angle_diff(atan2f(dy, dx) - theta - data[i].angle);

                        // angle related terms
                        float isq2 = 1.0F / square(data[i].angle_err * dr);

                        Hessian += isq2;

                        Deltax += -dy * angdiff * isq2;
                        Deltay += dx * angdiff * isq2;

                        // distance related terms, only exist when distance measurements are
                        // valid
                        if (data[i].dist_err != 0.0F)
                        {
                            float isq1 = 1.0F / square(data[i].dist_err);

                            Hessian += isq1;

                            Deltax += dx * (1 - data[i].dist / dr) * isq1;
                            Deltay += dy * (1 - data[i].dist / dr) * isq1;
                        }
                    }
                    x += 2.0F * Deltax / Hessian;
                    y += 2.0F * Deltay / Hessian;
                    if (abs(Deltax) <= Error_tolerance * Hessian &&
                        abs(Deltay) <= Error_tolerance * Hessian)
                    {
                        break;
                    }
                }
                // this is the final x and y, but we need to compute angle and hessian based
                // on this updated measurements
                temp.x = x;
                temp.y = y;
                temp.angle_0 = Angle_average(data, x, y);

                // keep an record of the last Hessian, so we have an estimate of how
                // accurate is this localization result. 1/Hessian would be a good
                // estimation of the variance of measurement.
                float Hessian = 0.0F;
                for (size_t i = 0; i < data_len; i++)
                {
                    Hessian += 1.0F / (square(data[i].angle_err) * (square(data[i].pos[0] - x) + square(data[i].pos[1] - y)));

                    // distance related terms, only exist when distance measurements are
                    // valid
                    if (data[i].dist_err != 0.0F)
                    {
                        Hessian += 1.0F / square(data[i].dist_err);
                    }
                }
                temp.var_xy = 1.0F / Hessian;

                temp.rotation_time = 0L;
                temp.angular_velocity = 0.0F;
                temp.time = 0L;

                return temp;
            }

            // /**
            //  * @brief a stack of positions and relative measurements
            //  */
            // typedef struct
            // {
            //     Position_data pos_dat;
            //     Relative_position_data rel_pos_dat[3];
            // } All_PD;
            // Circbuffer<All_PD, 300> Position_stack;
            //
            // void Print_data_task(void *pvParameters)
            // {
            //     while (1)
            //     {
            //         vTaskDelay(50);
            //         if (Serial.available())
            //         {
            //             delay(10);
            //             // deplete serial buffer
            //             while (Serial.available())
            //             {
            //                 Serial.read();
            //             }
            //
            //             Serial.println("\nLocalization result:");
            //             while(Position_stack.n_elem)
            //             {
            //                 auto temp = Position_stack.pop();
            //                 std::string v = "";
            //                 v.reserve(1000);
            //
            //                 v += std::string("x: ") + std::to_string(temp.pos_dat.x) + ", y: " + std::to_string(temp.pos_dat.y) + ", z: " + std::to_string(temp.pos_dat.z) + ", omega: " + std::to_string(1000000.0f * temp.pos_dat.angular_velocity) + "\n";
            //
            //                 for (size_t j = 0; j < 3; j++)
            //                 {
            //                     v += std::string("  --ID: ") + std::to_string(temp.rel_pos_dat[j].Robot_ID) + ", x0: " + std::to_string(temp.rel_pos_dat[j].pos[0]) + ", y0: " + std::to_string(temp.rel_pos_dat[j].pos[1]) + ", z0: " + std::to_string(temp.rel_pos_dat[j].pos[2]) + ", dist: " + std::to_string(temp.rel_pos_dat[j].dist) + ", dist_err: " + std::to_string(temp.rel_pos_dat[j].dist_err) + ", elev: " + std::to_string(temp.rel_pos_dat[j].elev) + ", elev_err: " + std::to_string(temp.rel_pos_dat[j].elev_err) + ", ang: " + std::to_string(temp.rel_pos_dat[j].angle) + ", ang_err: " + std::to_string(temp.rel_pos_dat[j].angle_err) + "\n";
            //                 }
            //
            //                 v += "\n";
            //
            //                 Serial.println(v.c_str());
            //             }
            //         }
            //     }
            // }

            /**
             * @brief a vector of all tasks to be notified when new data is processed.
             */
            std::vector<TaskHandle_t> TaskHandle_list;

            /**
             * @brief which type of trigger will localization task use? Could be
             * one of Trigger_based or Time_based. When is trigger based,
             * Localization will automatically run immediately after each data
             * update is finished. When is time based, Localization will run on
             * its own schedule and the period of execution is determined by
             * parameter period.
             */
            Trigger_type Localization_trig_type;
            /**
             * @brief Localization trigger period in ms
             */
            uint32_t Localization_trigger_period;

            /**
             * @brief task handle to Localization task
             */
            TaskHandle_t Localization_task_handle;

            /**
             * @brief A task that updates the localization result buffer and filtered
             * localization result buffer
             *
             * @param pvParameters
             *
             * @note some considerations about whether to include angular velocity as an
             * input for this function: from the program design perspective, this task
             * should be a task that simply reads in the relative measurements and then
             * output an estimation of the absolute position, and should not have redundant
             * information. However, an estimation of the angular velocity might be able to
             * help us identify the error of angle measurements (especially during
             * acceleration), or help us better understand how we should update the angle
             * estimation based on previous angle estimations. Yet, another reason why this
             * info might not be as helpful is because the angle estimation here should be
             * already sufficiently accurate as long as position estimation is accurate, and
             * the shift / error in position / angular velocity might just be too large so
             * that taking into consideration of the past measurement might not be very
             * useful afterall because the shift in angle during one rotation would be much
             * higher than the error of individual result.
             *
             * @note We use Kalmann filter to estimate the position and error right now,
             * with the assumption that the robot is static, and is subject to velocity
             * disturbances of normal distribution. The reason why we assume the position,
             * instead of the velocity, is a constant is partially because our localization
             * only gets position info per round, but the measurements are taken throughout
             * a time of one round. This means that there is an inherent measurement error
             * induced by velocity, and regardless of the model, it can only be accurate
             * when the robot is static. So, why not use the simplest model? However, there
             * is something we can slightly modify here: because the time step is not fixed,
             * and we know that between two updates, the velocity error will probably be in
             * the same direction. we can set the normal distribution's variance to be
             * proportional to T^2 instead of T. Well, because we are using a fixed position
             * model, Kalmann filter is just a glorified way of saying we are doing best
             * estimation based on multiple measurements.
             */
            void Localization_task(void *pvParameters)
            {
                uint32_t last_flag = 0;
                TickType_t prev_wake_time = xTaskGetTickCount();
                while (true)
                {
                    // check trigger type to determine how we should unblock this task.
                    if (Localization_trig_type == Trigger_type::Time_based)
                    {
                        // if this is time based, wait!
                        vTaskDelayUntil(&prev_wake_time, pdMS_TO_TICKS(Localization_trigger_period));
                    }
                    else
                    {
                        // if trigger based, wait for trigger signal.
                        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                    }

                    uint32_t curr_flag = RX::Get_io_flag();

                    // if the data has never been updated, just skip this round.
                    if (curr_flag == last_flag)
                    {
                        continue;
                    }

                    // neighboring robots
                    uint32_t robot_count;
                    uint32_t robot_ID_list[RX::Max_robots_simultaneous];

                    // neighboring robots' position
                    std::vector<RX::Parsed_msg_completed> pos_list;

                    // timing info
                    uint32_t time_count;
                    RX::Msg_timing_t time_list[RX::Timing_buffer_history_size];

                    // get data
                    do
                    {
                        curr_flag = RX::Get_io_flag();

                        robot_count = RX::Get_neighboring_robots_ID(robot_ID_list, 0);

                        pos_list.reserve(robot_count);
                        for (size_t i = 0; i < robot_count; i++)
                        {
                            pos_list.push_back(RX::Get_latest_msg_by_bot(robot_ID_list[i], Localization_Msg_type));
                        }

                        time_count = RX::Get_timing_data(time_list, Relative_measurements_expire_time);
                    } while (curr_flag != RX::Get_io_flag());

                    // some quick tests to verify we have enough data, if not, directly
                    // continue. we need at least 2 robots and 3 time counts to
                    // determine rotation speed and position
                    if (robot_count < 2 || time_count < 3)
                    {
                        // DEBUG_C(Serial.println("Not sufficent timing data!");)
                        continue;
                    }

                    // processing

                    // let's exploit the timing_valid_Q here to store the information about
                    // whether we've processed the second newest transmission. 0 ->
                    // processed, 1 -> not processed. We know that it's always 1 when we
                    // retrive them anyways.

                    // if finished=1, then directly quit because we've already scan through
                    // last two rotations.
                    bool finished = false;
                    // last useful robot's index, needs to be within the last rotation.
                    size_t last_index = 0U;
                    // something used to coarsely estimate the rotation speed by averaging
                    // the timing difference between seeing the same robot in two
                    // consecutive rounds.
                    int64_t rotation_time = 0L;
                    uint32_t rotation_count = 0U;
                    // compute rotation speed.
                    for (size_t i = 1U; i < time_count; i++)
                    {
                        // iterate till either the previous message, or end before the last
                        // rotation
                        for (size_t j = 0U; j < ((last_index) ? last_index : (i - 1)); j++)
                        {
                            // check if this a second-oldest message
                            if (time_list[i].robot_ID == time_list[j].robot_ID)
                            {
                                // if more than second oldest, break the whole cycle,
                                // because we've already run for two cycles.
                                if (!time_list[j].timing_valid_Q)
                                {
                                    finished = true;
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
                                    time_list[j].timing_valid_Q = 0U;
                                }
                                // end finding j whenever a solution is found.
                                break;
                            }
                        }

                        // if already finished, directly break.
                        if (finished)
                        {
                            break;
                        }
                    }

                    // compute rotation speed
                    // if we cannot determine the rotation speed, just quit this round!
                    if (!rotation_count)
                    {
                        // DEBUG_C(Serial.println("Cannot determine the rotation speed."));
                        continue;
                    }
                    // record single round time.
                    rotation_time /= rotation_count;

                    // if we can, compute the rotation speed.
                    float angular_velocity = float(M_TWOPI) / float(rotation_time);

                    // now we collect the data useful for localization.

                    // data directly used for localization
                    std::vector<Relative_position_data> Loc_data;
                    Loc_data.reserve(last_index);

                    // latest time info
                    int64_t last_message_time = 0L;

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
                                Loc_data.push_back(Relative_position_data{});
                                Relative_position_data &this_Loc_data = Loc_data.back();

                                this_Loc_data.Robot_ID = rid;
                                this_Loc_data.pos[0] = std::bit_cast<int16_t>(msg.content[0]);
                                this_Loc_data.pos[1] = std::bit_cast<int16_t>(msg.content[1]);
                                this_Loc_data.pos[2] = std::bit_cast<int16_t>(msg.content[2]);

                                position_found = 1;
                                break;
                            }
                        }
                        // if we found the position, setup the data
                        if (position_found)
                        {
                            Relative_position_data &this_Loc_data = Loc_data.back();
                            // check if timing data is reasonable
                            float ang_diff = float(int64_t(time_list[i].time_arr[1] - time_list[i].time_arr[2])) * angular_velocity + RX::Left_right_angle_offset;
                            // because the robot is rotating anti-clockwise when viewed from
                            // top, then left receiver will see the emitter first. thus the
                            // first reception time of the right receiver should be larger
                            // than the left.
                            // if ang_diff>=0 and ang_diff < Max_valid_angle, which is basic
                            // for sanity, we record its angle info. but not necessarily
                            // distance info.
                            if (ang_diff > 0 && ang_diff < Max_valid_angle)
                            {
                                int64_t avg_time = ((time_list[i].time_arr[1] + time_list[i].time_arr[2]) >> 1);

                                // update last_message_time if the new one is later
                                if (avg_time - last_message_time > 0)
                                {
                                    last_message_time = avg_time;
                                }

                                this_Loc_data.angle = float(avg_time % rotation_time) * angular_velocity;
                                this_Loc_data.angle_err = Relative_angle_error;
                                // we only record the distance info if the distance could be
                                // usefully accurate (~ <50cm) you can adjust that threshold
                                // for using the distance information or not by adjusting
                                // the Min/Max valid angle
                                if (ang_diff > Min_valid_angle)
                                {
                                    this_Loc_data.dist = (Left_right_distance / (2.0F * sin(ang_diff * 0.5F)));
                                    this_Loc_data.dist_err = Distance_error(this_Loc_data.dist);
                                    this_Loc_data.elev = this_Loc_data.dist * tanf((time_list[i].time_arr[0] - avg_time) * angular_velocity * Tilting_angle_multiplyer + RX::Center_angle_offset);
                                    this_Loc_data.elev_err = Elevation_error(this_Loc_data.dist, this_Loc_data.elev);
                                }
                                else
                                {
                                    this_Loc_data.dist = 0.0F;
                                    this_Loc_data.dist_err = 0.0F;
                                    this_Loc_data.elev = 0.0F;
                                    this_Loc_data.elev_err = 0.0F;
                                }
                            }
                            // if cannot pass sanity check, just delete this.
                            else
                            {
                                Loc_data.pop_back();
                            }
                        }
                    }

                    // we localize when there are at least 2 beacons
                    if (Loc_data.size() >= 2)
                    {
                        // new result
                        Position_data res;

                        // compute localization based purely on new data
                        // if there are previous results, use that for guess of
                        // new localization.
                        // 
                        // @note should we do this is open to debate, because we
                        // are almost sure that starting from the center will
                        // work, but previous results might be in unfavourable
                        // position.
                        if (Filtered_position_stack.n_elem && (last_message_time - Filtered_position_stack.peek_tail().time) < Position_expire_time)
                        {
                            // last filtered result
                            Position_data last = Filtered_position_stack.peek_tail();

                            res = Execute_localization(Loc_data, last.x, last.y);
                            // do not filter these two because they directly
                            // determines the angle offset, and the offset is
                            // computed by something like
                            // (T-[T/rotation_time])*angular_velocity
                            res.rotation_time = rotation_time;
                            res.angular_velocity = angular_velocity;
                            res.time = last_message_time;

                            // store data in the non-filtered stack
                            Position_stack.push(res);

                            // now we start to kalmann filter stuff
                            // new filtered result
                            Position_data filt;
                            // add some basic info
                            filt.rotation_time = rotation_time;
                            filt.angular_velocity = angular_velocity;
                            filt.time = last_message_time;

                            // determine how much variance should be added to
                            // the old measurement due to drifting of drone
                            float var_drift = square(float(res.time - last.time) * Error_increase_rate);

                            // determine z and var_z
                            float tmpv = 1.0F / (res.var_z + last.var_z + var_drift);
                            filt.z = (last.z * res.var_z + res.z * (last.var_z + var_drift)) * tmpv;
                            filt.var_z = res.var_z * (last.var_z + var_drift) * tmpv;

                            // similar for xy and var_xy
                            tmpv = 1.0F / (res.var_xy + last.var_xy + var_drift);
                            filt.x = (last.x * res.var_xy + res.x * (last.var_xy + var_drift)) * tmpv;
                            filt.y = (last.y * res.var_xy + res.y * (last.var_xy + var_drift)) * tmpv;
                            filt.var_xy = res.var_xy * (last.var_xy + var_drift) * tmpv;

                            // compute angle based on new xyz
                            filt.angle_0 = Angle_average(Loc_data, filt.x, filt.y);

                            // store in filtered stack
                            Filtered_position_stack.push(filt);
                        }
                        // if there are no previous results or they are all too old, we start from the average of all positions
                        else
                        {
                            float x0 = 0.0F;
                            float y0 = 0.0F;
                            for (auto &p : Loc_data)
                            {
                                x0 += p.pos[0];
                                y0 += p.pos[1];
                            }
                            x0 /= float(Loc_data.size());
                            y0 /= float(Loc_data.size());

                            res = Execute_localization(Loc_data, x0, y0);
                            // do not filter these two because they directly
                            // determines the angle offset, and the offset is
                            // computed by something like
                            // (T-[T/rotation_time])*angular_velocity
                            res.rotation_time = rotation_time;
                            res.angular_velocity = angular_velocity;
                            res.time = last_message_time;
                            // store data in the non-filtered stack
                            Position_stack.push(res);
                            // because we don't have available previous info to
                            // merge, we directly push into the filtered stack.
                            Filtered_position_stack.push(res);
                        }

                        vTaskSuspendAll();
                        // notify all tasks that should be notified
                        for (auto &hand : TaskHandle_list)
                        {
                            xTaskNotifyGive(hand);
                        }
                        xTaskResumeAll();
                    }
                }
            }
        } // anonymous namespace

        bool Init(const uint32_t loc_msg_type, const Trigger_type trig, const uint32_t period)
        {
            if (loc_msg_type > Single_transmission_msg_type && loc_msg_type <= Msg_type_max)
            {
                Localization_Msg_type = loc_msg_type;
                Localization_trig_type = trig;
                Localization_trigger_period = period;
            }
            else
            {
                DEBUG_C(
                    Serial.println("Invalid msg_type for transmitting robot's position!");
                    Serial.print("Valid range is [ ");
                    Serial.print(Single_transmission_msg_type + 1);
                    Serial.print(" , ");
                    Serial.print(Msg_type_max);
                    Serial.println(" ]");)

                return false;
            }

            auto task_status = xTaskCreatePinnedToCore(
                Localization_task,
                "Localization_task",
                50000,
                NULL,
                Localization_task_priority,
                &Localization_task_handle,
                0);

            if (task_status != pdTRUE)
            {
                DEBUG_C(Serial.println("Localization startup failed!"));
                return false;
            }

            // if it's trigger based, add trigger to Localization task
            if (trig == Trigger_type::Trigger_based)
            {
                RX::Add_RX_Notification(Localization_task_handle);
            }

            DEBUG_C(Serial.println("Localization setup finished!"));
            return true;
        }

        bool Add_Localization_Notification(const TaskHandle_t &handle)
        {
            vTaskSuspendAll();
            if (std::find(TaskHandle_list.begin(), TaskHandle_list.end(), handle) == TaskHandle_list.end())
            {
                TaskHandle_list.push_back(handle);
                xTaskResumeAll();
                return true;
            }
            xTaskResumeAll();
            return false;
        }

        bool Remove_Localization_Notification(const TaskHandle_t &handle)
        {
            vTaskSuspendAll();
            auto it = std::find(TaskHandle_list.begin(), TaskHandle_list.end(), handle);
            if (it != TaskHandle_list.end())
            {
                TaskHandle_list.erase(it);
                xTaskResumeAll();
                return true;
            }
            xTaskResumeAll();
            return false;
        }

        Position_data Get_position(const uint32_t age)
        {
            if (Position_stack.n_elem > age)
            {
                return Position_stack.peek_tail(age);
            }
            return Position_data{};
        }

        Position_data Get_filtered_position(const uint32_t age)
        {
            if (Filtered_position_stack.n_elem > age)
            {
                return Filtered_position_stack.peek_tail(age);
            }
            return Position_data{};
        }

        size_t Get_position_data_count(const int64_t history_time)
        {
            if (history_time == 0)
            {
                return Position_stack.n_elem;
            }

            uint32_t curr_flag;
            size_t count;

            do
            {
                curr_flag = Filtered_position_stack.io_flag;
                int64_t curr_time = esp_timer_get_time();

                for (count = 0; count < Position_stack.n_elem; count++)
                {
                    // break if time reached
                    if ((curr_time - Position_stack.peek_tail(count).time) > history_time)
                    {
                        break;
                    }
                }
            }
            // repeat if write task preempted this task
            while (Position_stack.io_flag != curr_flag);

            return count;
        }

        uint32_t Get_io_flag()
        {
            return Position_stack.io_flag;
        }
    } // namespace Localization
} // namespace IR
