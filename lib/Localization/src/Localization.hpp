/**
 * @file Localization.hpp
 * @brief Localization task and relavent functions
 */
#ifndef LOCALIZATION_HPP__
#define LOCALIZATION_HPP__

#include "Arduino.h"
#include "IrRX.hpp"

/**
 * @brief should we be in calibration mode?
 * 
 * @note in calibration mode, we basically output all raw measurement data we
 * obtained through serial. (buffer might be required)
 */
// #define LOCALIZATION_CALIBRATION_MODE_ON 1

namespace IR
{
    namespace Localization
    {
        /**
         * @brief priority of Localization task, it should be marginally smaller
         * than Preprocess task, but still higher than the rest.
         */
        constexpr uint32_t Localization_task_priority = RX::Preprocess_task_priority - 1;

        /**
         * @brief type of trigger, whether time based or trigger based.
         */
        enum class Trigger_type
        {
            Time_based,   // task will run on its own schedule and the period of execution is determined by parameter period
            Trigger_based // task will automatically run immediately after each data update is finished.
        };

        /**
         * @brief container for absolute position information, note that
         * this includes the error estimation as well. Position are in unit
         * of mm, variance of position in unit of mm^2, angle in unit of
         * rad.
         */
        struct Position_data
        {
            float x;                // estimated x position
            float y;                // estimated y position
            float var_xy;           // variance of x,y position, sqrt(var)=error
            float z;                // estimated z position
            float var_z;            // variance of z position, sqrt(var)=error
            float angle_0;          // angle the robot's facing at k*rotation_time
            int64_t rotation_time;  // used to pre-treat the int64_t to prevent clipping in float.
            float angular_velocity; // angular velocity
            int64_t time;           // the time of data, which is the last measurement's time
            float mean_error_factor;// only useful for individual measurements. this is the average of ((measurement-estimation)/measurement_error)^2, which partially indicates how good/poor our data quality is.
        };

        /**
         * @brief initialize localization routine
         *
         * @param loc_msg_type which type of message is for transmitting
         * position of neighboring robots
         * @param trig trigger method, could be one of Trigger_based or
         * Time_based. When is trigger based, Localization will automatically
         * run immediately after each data update is finished. When is time
         * based, Localization will run on its own schedule and the period of
         * execution is determined by parameter period.
         * @param period only useful in time based mode, determines how long
         * will we execute localization once.
         * @return bool 0 -> successful, 1 -> unsuccessful
         *
         * @note should execute this first before doing anything else! I don't
         * check for this, but you have to.
         */
        bool Init(const uint32_t loc_msg_type = 4U, const Trigger_type trig = Trigger_type::Trigger_based, const uint32_t period = 50U);

        /**
         * @brief Notify this task when data is updated!
         *
         * @param handle Task handle to be added
         * @return bool true if successfully added, false if already exists.
         */
        bool Add_Localization_Notification(const TaskHandle_t &handle);

        /**
         * @brief No longer notify this task when data is updated!
         *
         * @param handle Task handle to be removed
         * @return bool true if successfully removed, false if never exist.
         */
        bool Remove_Localization_Notification(const TaskHandle_t &handle);

        /**
         * @brief Get the latest unfiltered position information
         *
         * @param age by default is 0 -> latest, 1 -> next latest, etc. max is
         * recent_msg_buffer_history_size - 1
         * @return Position_data
         *
         * @note return data will be all 0 if no position data is available.
         *
         * @warning is not super thread safe...
         */
        Position_data Get_position(const uint32_t age = 0);

        /**
         * @brief Get the latest filtered position information
         *
         * @param age by default is 0 -> latest, 1 -> next latest, etc. max is
         * recent_msg_buffer_history_size - 1
         * @return Position_data
         *
         * @note return data will be all 0 if no position data is available.
         *
         * @warning is not super thread safe...
         */
        Position_data Get_filtered_position(const uint32_t age = 0);

        /**
         * @brief How many data is available.
         *
         * @param history_time get only messages that has been seen within
         * history_time us. when is 0, then access all.
         * @return size_t number
         */
        size_t Get_position_data_count(const int64_t history_time = 0);

        /**
         * @brief get io_flag, can be used to implement one's own read/update routine.
         *        a standard way of using it is:
         *          uint32_t curr_flag;
         *          do
         *          {
         *              curr_flag = Get_io_flag();
         *              // read something
         *          }
         *          // repeat if write task preempted this task
         *          while (Get_io_flag() != curr_flag);
         *
         * @note io_flag can also be used to determine whether there's an new
         * update, to eliminate redundant data updates.
         */
        uint32_t Get_io_flag();
    } // namespace Localization
} // namespace IR
#endif