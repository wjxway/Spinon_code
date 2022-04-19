// Encoding & decoding of RMT signal
#ifndef _RMTCODING_
#define _RMTCODING_
#pragma once

#include "Arduino.h"
#include "RobotDefs.hpp"
#include "driver/rmt.h"

// Because I implemented a HPF in the filter to deal with change in environmental light and irrelavent blinks,
// The data we are sending must have half low level and half high level, evenly distributed.
// A natural idea would be to use 01 to represent 0 and 10 to represent one. (This is also a commonly used IR protocol)
// The message should always start with 0, followed by the real data
// The message should be transmitted in LSB to MSB order:
// For example, if we want to transmit a signal: 1010, then the pulse sequence should be: 01 01 10 01 10

namespace detail
{
    /**
     * @brief The amount of data, in bits, that's transmitted in each RMT transmission.
     *
     * @note RMT length should be kept below 63 for convenience of processing (leaving some space for init code and probably ECC.)
     * When RMT length is larger than 63, it will occupy at least two RMT memory register block. That's inconvenient and requires much more processing.
     * I would suggest using rmt length from 16 to 32. dUsing rmt length >32 then you will need to modify the code here and there, changing uint32_t to uint64_t.
     */
    extern const uint32_t RMT_data_length;

    /**
     * @brief RMT clock division, rmt clock speed = 80MHz / RMT_clock_div
     * @note RMT_ticks_num * RMT_clock_div * 1/80MHz is the pulse width.
     */
    constexpr uint32_t RMT_clock_div = 5;

    /**
     * @brief Ticks count of each pulse
     * @note RMT_ticks_num * RMT_clock_div * 1/80MHz is the pulse width.
     */
    constexpr uint32_t RMT_ticks_num = 4;

    /**
     * @brief Ticks number that deviate from RMT_ticks_num by RMT_ticks_tol will still be accepted
     * @note There will be error in ticks number in reality, so we could give it some tolerance
     */
    constexpr uint32_t RMT_ticks_tol = 1;

    /**
     * @brief Number of bits per cycle/pulse
     * @note Please keep it at 2 for simplicity, making it larger won't improve the performance much further.
     */
    constexpr uint32_t Bit_per_cycle = 2;

    /**
     * @brief Padding before each cycle in ticks
     */
    constexpr uint32_t Pad_per_cycle = RMT_ticks_num;
}

/**
 * @brief Generate rmt item for emission based on input data
 *
 * @note For rmt length >32, change uint32_t to uint64_t or larger
 *
 * @param pointer Pointer to a pre-defined space of rmt item storage. The elements inside the array are volatile.
 * @param data Input data in an integer form.
 * @param ticks_delay How many ticks of delay before the data, for time sync.
 * @param ticks_final How many ticks for the final pulse in the data.
 *
 * @return true Successful!
 * @return false Unable to generate.
 */
bool Generate_RMT_item(rmt_item32_t *pointer, uint32_t data, uint32_t ticks_delay, uint32_t ticks_final);

/**
 * @brief Parse rmt item to data.
 *
 * @note For rmt length >32, change uint32_t to uint64_t or larger
 *
 * @param pointer1 Pointer to raw rmt item storage. The elements inside the array are volatile.
 * @param dataptr Pointer to the output space, the output is an integer.
 *
 * @return true Successful!
 * @return false Data invalid!
 * 
 * @note each call takes ~2.2us
 */
bool IRAM_ATTR Parse_RMT_item(volatile rmt_item32_t *pointer, uint32_t *dataptr);

#endif