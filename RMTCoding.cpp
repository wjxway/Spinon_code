#include "RMTCoding.hpp"

// speed is crucial, no debug here!
#define NDEBUG

using namespace detail;

extern const uint32_t detail::RMT_data_length;

bool Generate_RMT_item(rmt_item32_t *pointer, uint32_t data)
{
    // temp zero length
    uint32_t temp_len = 0, temp = 0;

    for (uint32_t i = 0; i < RMT_data_length / Bit_per_cycle; i++)
    {
        temp = ((data >> (i * Bit_per_cycle) & ((1 << Bit_per_cycle) - 1)));
        temp_len += temp;
        pointer[i] = {{{RMT_ticks_num, 1, RMT_ticks_num * temp_len + Pad_per_cycle, 0}}};
        temp_len = ((1 << Bit_per_cycle) - 1) - temp;
    }
    // termination indicator
    pointer[RMT_data_length / Bit_per_cycle] = {{{RMT_ticks_num, 1, 0, 0}}};
    pointer[RMT_data_length / Bit_per_cycle + 1] = {0};

    return true;
}

// modified 4ppm version
bool IRAM_ATTR Parse_RMT_item(volatile rmt_item32_t *pointer, uint32_t *dataptr)
{
    // just in case the idle_level isn't correct
    if (pointer[0].level1)
        return false;

    uint32_t val = 0, curr = 3;
    rmt_item32_t temp;

    for (int i = 0; i < RMT_data_length / Bit_per_cycle; i++)
    {
        temp.val = pointer[i].val;

        // discard if the message length is too short.
        if (!temp.duration1)
            return false;

        curr += 1 + ((temp.duration1 + RMT_ticks_tol - Pad_per_cycle) / RMT_ticks_num);

        // discard if time gap between two messages are incorrect
        if (curr < (1 << Bit_per_cycle) || curr >= (2 << Bit_per_cycle))
            return false;

        curr -= (1 << Bit_per_cycle);
        val += (curr << (i * Bit_per_cycle));
    }

    // discard if the message length is too long.
    if (pointer[RMT_data_length / Bit_per_cycle].duration1)
        return false;
    else
    {
        *dataptr = val;
        return true;
    }
}