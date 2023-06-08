/**
 * @file IrTX.hpp
 * @brief transmit data interface
 */
#ifndef IRTX_HPP__
#define IRTX_HPP__

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
         *        3. initialize scheduler and default TX task
         * 
         * @note should execute this first before doing anything else! I don't
         * check for this, but you have to.
         */
        void Init();

        /**
         * @brief remove a task of certain type from scheduler (no longer
         * transmit)
         *
         * @param type msg_type of data
         * 
         * @warning this function is thread safe, but not multi-core safe!
         */
        void Remove_from_schedule_safe(const uint32_t type);

        /**
         * @brief Create or reset data, then add to schedule.
         *
         * @param type msg_type of data
         * @param raw raw data, each uint16_t should contain Msg_content_bits of
         * data.
         * @param priority1 priority of this task
         * @param expiration_count how much individual transmissions before this
         * task is removed. note that it might take multiple individual
         * transmissions to complete one complete transmission. By default this
         * is -1, which means never expire.
         * @param period when no other task is presented, this data should be
         * transmitted once per how many transmissions. By default this is 1,
         * which means fire every time.
         *
         * @warning calling this will ALWAYS override the existing data.
         * @warning this function is thread safe, but not multi-core safe!
         */
        void Add_to_schedule(const uint32_t type, const std::vector<uint16_t> &raw, const uint32_t priority1, const int32_t expiration_count = -1, const uint32_t period = 1);
    } // namespace TX
} // namespace IR

#endif