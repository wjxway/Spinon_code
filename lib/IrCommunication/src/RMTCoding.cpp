#include "RMTCoding.hpp"
#include "RMTMessageDefs.hpp"
// #include "Utilities/DebugDefs.hpp"

// speed is crucial, no debug here!
#define NDEBUG

// modified 1+4ppm version
bool IR::detail::Generate_RMT_item(rmt_item32_t *const pointer, const uint32_t data) noexcept
{
    // delay and initial pulse
    pointer[0] = {{{1 + RMT_sync_ticks_num, 0, RMT_ticks_num, 1}}};

    // temp zero length
    uint32_t temp_len = 0;

    for (size_t i = 0; i < RMT_data_pulse_count; i++)
    {
        const uint32_t temp = (((data >> (i * Bit_per_cycle)) & ((1 << Bit_per_cycle) - 1)));
        temp_len += temp;
        pointer[i + 1] = {{{RMT_ticks_num * temp_len + Pad_per_cycle, 0, RMT_ticks_num, 1}}};
        temp_len = ((1 << Bit_per_cycle) - 1) - temp;
    }

    // termination
    pointer[RMT_data_pulse_count + 1] = pointer[RMT_data_pulse_count + 2] = {0};

    return true;
}

// modified 1+4ppm version
bool IRAM_ATTR IR::detail::Parse_RMT_item(volatile rmt_item32_t *const pointer, uint32_t *const dataptr) noexcept
{
    // just in case the idle_level isn't correct
    if (pointer[0].level1)
        return false;

    // val is the final uint32_t output
    // curr is the absolute starting time of this or last pulse.
    uint32_t val = 0, curr = 3;
    rmt_item32_t temp;

    for (size_t i = 0; i < RMT_data_pulse_count; i++)
    {
        temp.val = pointer[i].val;

        // discard if the message length is too short.
        if (!temp.duration1)
            return false;

        // // add pulse length and pulse gap to the absolute starting time of last pulse to get the start time of this pulse.
        // curr += 1 + ((temp.duration1 + RMT_ticks_tol - Pad_per_cycle) / RMT_ticks_num);

        // add pulse length and pulse gap to the absolute starting time of last pulse to get the start time of this pulse.
        // This version is much much better!
        curr += ((temp.duration0 + temp.duration1 + RMT_ticks_tol - Pad_per_cycle) >> RMT_ticks_shift);

        // discard if time gap between two messages are incorrect
        if (curr < (1 << Bit_per_cycle) || curr >= (2 << Bit_per_cycle))
            return false;

        curr -= (1 << Bit_per_cycle);
        val += (curr << (i * Bit_per_cycle));
    }

    // the final message's length
    // if it's larger than RMT_ticks_num+RMT_ticks_tol, then it's coming from the upper emitter, or else it's coming from the lower emitter.
    // record this information in the 32th bit.
    val += ((pointer[RMT_data_pulse_count].duration0 > RMT_ticks_num + RMT_ticks_tol) << (32 - 1));

    // discard if the message length is too long.
    if (pointer[RMT_data_pulse_count].duration1)
        return false;
    else
    {
        *dataptr = val;
        return true;
    }
}