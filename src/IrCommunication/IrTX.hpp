/**
 * @file IrTX.hpp
 * @brief transmit data interface
 */
#ifndef _IRTX_HPP_
#define _IRTX_HPP_

#include "Arduino.h"
#include "RMTMessageDefs.hpp"
#include "driver/rmt.h"
#include <vector>

namespace IR
{
    namespace TX
    {
        constexpr uint32_t TX_priority_max = detail::Msg_type_max;

        /**
         * @brief initialization of TX routine, including
         *        1. initialize TX RMT channels
         *        2. initialize TX ISR, timer and random number generator
         *        2. initialize scheduler and default TX task
         */
        void Init();

        /**
         * @brief remove a task of certain type from scheduler (no longer
         * transmit)
         *
         * @param type msg_type of data
         */
        void Remove_from_schedule(const uint32_t type);

        /**
         * @brief Create or reset data, then add to schedule.
         *
         * @param type msg_type of data
         * @param raw raw data, each uint32_t should contain Msg_content_bits of
         * data.
         * @param priority1 priority of this task
         * @param expiration_count how much individual transmissions before this
         * task is removed. note that it might take multiple individual
         * transmissions to complete one complete transmission.
         * @param period when no other task is presented, this data should be
         * transmitted once per how many transmissions.
         *
         * @warning calling this will ALWAYS override the existing data.
         */
        void Add_to_schedule(const uint32_t type, const std::vector<uint16_t> &raw, const uint32_t priority1, const int32_t expiration_count, const uint32_t period);
    }
}

#endif