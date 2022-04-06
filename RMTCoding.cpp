#include "RMTCoding.hpp"

// speed is crucial, no debug here!
#define NDEBUG

using namespace detail;

extern const uint32_t detail::RMT_data_length;

bool Generate_RMT_item(rmt_item32_t *pointer, uint32_t data)
{
    // pulse that represent 0 and 1.
    static rmt_item32_t item0 = {{{RMT_ticks_num, 0, RMT_ticks_num, 1}}};
    static rmt_item32_t item1 = {{{RMT_ticks_num, 1, RMT_ticks_num, 0}}};

    pointer[0] = item0;

    for (uint32_t i = 0; i < RMT_data_length; i++)
        pointer[i + 1] = ((data >> i) & 1) ? item1 : item0;

    // termination indicator
    pointer[RMT_data_length + 1] = {0};

    return true;
}

// fastest version
bool IRAM_ATTR Parse_RMT_item(volatile rmt_item32_t *pointer, uint32_t *dataptr)
{
    // just in case the idle_level isn't correct
    if (pointer[0].level1)
        return false;

    int32_t data = 0, val = 0, bit_pos = 1;
    rmt_item32_t temp;

    for (int32_t i = 0; i <= RMT_data_length; i++)
    {
        // buffer the data to prevent repetitive flash reads.
        temp.val = pointer[i].val;

        // check for pointer[0].level0
        // if it's zero then it means the signal has already end with some oscillation
        if (!(temp.level0))
            break;

        // high period
        if (temp.duration0 > RMT_ticks_num + RMT_ticks_tol)
        {
            if (val)
                return false;

            val = 1;
            data += bit_pos;
            bit_pos = bit_pos << 1;
        }
        // low period
        if (!temp.duration1)
            break;
        if (temp.duration1 > 2 * RMT_ticks_num - RMT_ticks_tol)
        {
            if (!val)
                return false;

            val = 0;
        }
        data += val * bit_pos;
        bit_pos = bit_pos << 1;
    }

    // check message length
    if (bit_pos == (1 << RMT_data_length))
    {
        *dataptr = data;
        return true;
    }
    else
        return false;
}