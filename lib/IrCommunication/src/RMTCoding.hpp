/**
 * @file RMTCoding.hpp
 * @brief Encoding & decoding of RMT signal
 */
#ifndef _RMTCODING_HPP_
#define _RMTCODING_HPP_

#include "Arduino.h"
#include "soc/rmt_struct.h"

namespace IR
{
    namespace detail
    {
        /**
         * @brief Generate rmt item for emission based on input data
         *
         * @note For rmt length >32, change uint32_t to uint64_t or larger
         *
         * @param pointer Pointer to a pre-defined space of rmt item storage. The
         * elements inside the array are volatile.
         * @param data Input data in an integer form.
         *
         * @return true Successful!
         * @return false Unable to generate.
         *
         * @note Please manually edit the generated
         * rmt_item32_t[] object to control the initial delay and final pulse length. undefined
         * The default delay is 1 tick and default final pulse length is
         * RMT_ticks_num. Usually, for synchronization and distinguishing between
         * the upper and lower emitter, the lower emitter can directly use the
         * generated data, but the upper emitter should change the
         * pointer[0].duration0 to 2 undefined and
         * pointer[RMT_data_pulse_count].duration1 to 2*RMT_ticks_num undefined
         */
        bool Generate_RMT_item(rmt_item32_t *const pointer, const uint32_t data) noexcept;

        /**
         * @brief Parse rmt item to data.
         *
         * @note For rmt length >32, change uint32_t to uint64_t or larger
         *
         * @param pointer1 Pointer to raw rmt item storage. The elements inside the
         * array are volatile.
         * @param dataptr Pointer to the output space, the output is an integer.
         *
         * @return true Successful!
         * @return false Data invalid!
         *
         * @note each call takes ~2.2us
         */
        bool IRAM_ATTR Parse_RMT_item(volatile rmt_item32_t *const pointer, uint32_t *const dataptr) noexcept;
    }
}

#endif