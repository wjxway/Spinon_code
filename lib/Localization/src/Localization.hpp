/**
 * @file Localization.hpp
 * @brief Localization task and relavent functions
 */
#ifndef LOCALIZATION_HPP__
#define LOCALIZATION_HPP__

#include "Arduino.h"
#include "IrRX.hpp"

namespace IR
{
    namespace Localization
    {
        /**
         * @brief priority of Localization task, it should be marginally smaller than Preprocess task, but still higher than the rest.
         */
        constexpr uint32_t Localization_task_priority = RX::Preprocess_task_priority - 1;

        enum class Trigger_type
        {
            Time_based,
            Trigger_based
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
    } // namespace Localization
} // namespace IR
#endif