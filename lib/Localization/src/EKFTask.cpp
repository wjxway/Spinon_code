#include "EKFTask.hpp"
#include "EKFCore.hpp"
#include <vector>
#include <IrRX.hpp>
#include <Utilities.hpp>

namespace EKF
{
    using namespace IR;

    namespace
    {
        /**
         * @brief type of message that carries localization data
         */
        uint32_t Localization_Msg_type = 4;

        /**
         * @brief task handle to Localization task
         */
        TaskHandle_t Localization_task_handle;

        /**
         * @brief a vector of all tasks to be notified when new data is processed.
         */
        std::vector<TaskHandle_t> TaskHandle_list;

        /**
         * @brief the minimum tolerable timing for distance measurements is
         * 6 degrees, corresponding to a distance of ~50cm
         */
        constexpr float Min_valid_angle = 6.0F;
        /**
         * @brief the maximum tolerable timing is 60 degrees, corresponding
         * to two robots touching each other.
         */
        constexpr float Max_valid_angle = 60.0F;

        /**
         * @brief Motor_data_buffer's max size
         */
        constexpr size_t Motor_data_buffer_size = 5U;
        /**
         * @brief buffer storing all motor feed forward and speed feedback data
         */
        Circbuffer<Motor_info_t, Motor_data_buffer_size> Motor_data_buffer;

        /**
         * @brief a copycat class for timing buffer
         */
        Circbuffer_copycat<IR::RX::Msg_timing_t, 24U> Timing_buffer_copycat;

        /**
         * @brief compute distance based on angle
         *
         * @param LR_angle difference between angle received by left and
         * right receiver.
         * @return constexpr float distance estimation in cm
         */
        float Distance_expectation(const float LR_angle)
        {
            return IR::RX::Distance_param_a / sin((LR_angle * float(DEG_TO_RAD) + IR::RX::Distance_param_b) / 2.0F) + IR::RX::Distance_param_c;
        }

        /**
         * @brief thrust vs. angular velocity
         *
         * @param w input angular velocity in rounds/sec
         * @return thrust in gf
         */
        float Thrust_function(float w)
        {
            return 0.0000595f * math::fast::square(w - 25.0f);
        }

        void Localization_task(void *pvParameters)
        {
            while (true)
            {
                // wait till trigger signal which is sent at the end of a rotation
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

                // fetch the motor thrust and feedback information
                auto motor_dat = Motor_data_buffer.pop();

                // construct actuation vector
                Act_vector act;

                float w_high = 1.0e6f / (12.0f * float(motor_dat.spd_FB_high));
                float w_low = 1.0e6f / (12.0f * float(motor_dat.spd_FB_low));

                float w_avg = (w_high + w_low) / 2.0f;
                float dw = sqrtf(0.0005f * ((motor_dat.spd_high - motor_dat.spd_low) + (math::fast::exp(-0.5f * float(motor_dat.spd_high - motor_dat.spd_low)) - 1.0f) * 2.0f)) * w_avg;
                float dt = (Thrust_function(w_avg + dw) - Thrust_function(w_avg - dw)) / 2;

                act.F_a = Thrust_function(w_avg);
                act.F_x = -dt * sin(motor_dat.thrust_angle);
                act.F_y = dt * cos(motor_dat.thrust_angle);
                // let set it to 0 for now.
                act.F_beta = 0.0f;

                // now start to process the timing buffer one by one
                // because this is executed when the previous round is just completed (or half a round later)
                // so we don't really care about when the position message is obtained. It won't vary much.
                // we also don't care if we have multiple messages in the same round.
                while (!Timing_buffer_copycat.Empty_Q())
                {
                    auto timing_dat = Timing_buffer_copycat.peek();

                    // break at first message exceeding the last motor cycle time
                    if (timing_dat.Get_avg_time() >= motor_dat.stop_time)
                    {
                        break;
                    }
                    else
                    {
                        Timing_buffer_copycat.pop();
                    }

                    Meas_vector meas;

                    uint32_t curr_flag;
                    // get position data
                    do
                    {
                        curr_flag = RX::Get_io_flag();
                        auto msg = RX::Get_latest_msg_by_bot(timing_dat.robot_ID, Localization_Msg_type);

                        if (msg.content_length)
                        {
                            meas.x_0 = std::bit_cast<int16_t>(msg.content[0]) * Position_scaling_factor;
                            meas.y_0 = std::bit_cast<int16_t>(msg.content[1]) * Position_scaling_factor;
                            meas.z_0 = std::bit_cast<int16_t>(msg.content[2]) * Position_scaling_factor;

                            // now we assume error of position is always 3mm
                            // could be replaced by message content
                            meas.sigma_0 = 0.1f;
                        }
                        else
                        {
                            curr_flag = 0xFFFF;
                            break;
                        }
                    } while (curr_flag != RX::Get_io_flag());

                    // if the sensed drone's location has not been received, continue.
                    if (curr_flag == 0xFFFF)
                    {
                        continue;
                    }

                    int64_t avg_time = timing_dat.Get_avg_time();
                    meas.time = avg_time;

                    // angular velocity in degree/s
                    float angular_velocity = Get_last_angular_velocity() * 1.0e-6;
                    // timing difference between left and right receiver.
                    float LR_diff = float(timing_dat.time_arr[1] - timing_dat.time_arr[2]) * angular_velocity;
                    // timing difference between center and average of LR.
                    float Cent_diff = float(timing_dat.time_arr[0] - avg_time) * angular_velocity;

                    float Compensated_LR_diff = LR_diff + IR::RX::LR_angle_compensation * Cent_diff;

                    // compensate for up/low emitter
                    if (timing_dat.emitter_pos == 0)
                    {
                        meas.z_0 += IR::detail::LED_elevation_diff;
                    }

                    // check if timing data is reasonable

                    // because the robot is rotating anti-clockwise when
                    // viewed from top, then left receiver will see the
                    // emitter first. thus the first reception time of
                    // the right receiver should be larger than the
                    // left. if ang_diff>=0 and ang_diff <
                    // Max_valid_angle, which is basic for sanity, we
                    // record its angle info. but not necessarily
                    // distance info.
                    if (Compensated_LR_diff > 0.0F && Compensated_LR_diff < Max_valid_angle)
                    {
                        // we only record the distance info if the
                        // distance could be usefully accurate (~ <42cm)
                        // you can adjust that threshold for using the
                        // distance information or not by adjusting the
                        // Min/Max valid angle
                        if (Compensated_LR_diff > Min_valid_angle)
                        {
                            meas.d = Distance_expectation(Compensated_LR_diff);
                            meas.phi = Cent_diff;
                        }

                        // construct and feed data to EKFCore
                        Iterate_state(act, meas);
                    }
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
    } // anonymous namespace

    bool Init(const uint32_t loc_msg_type)
    {
        if (loc_msg_type > detail::Single_transmission_msg_type && loc_msg_type <= detail::Msg_type_max)
        {
            Localization_Msg_type = loc_msg_type;
        }
        else
        {
            DEBUG_C(
                Serial.println("Invalid msg_type for transmitting robot's position!");
                Serial.print("Valid range is [ ");
                Serial.print(Single_transmission_msg_type + 1);
                Serial.print(" , ");
                Serial.print(Msg_type_max);
                Serial.println(" ]"));

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

    TaskHandle_t Get_Localization_Task_Handle()
    {
        return Localization_task_handle;
    }
} // EKF namespace