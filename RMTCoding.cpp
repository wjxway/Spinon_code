#include "RMTCoding.h"

using namespace detail;

extern const uint8_t detail::RMT_data_length;

bool Generate_RMT_item(rmt_item32_t *pointer, uint32_t data)
{
    // pulse that represent 0 and 1.
    static rmt_item32_t item0 = {{{RMT_ticks_num, 0, RMT_ticks_num, 1}}};
    static rmt_item32_t item1 = {{{RMT_ticks_num, 1, RMT_ticks_num, 0}}};

    pointer[0] = item0;

    for (uint16_t i = 0; i < RMT_data_length; i++)
        pointer[i + 1] = ((data >> i) & 1) ? item1 : item0;

    // termination indicator
    pointer[RMT_data_length + 1] = {0};

    return true;
}

bool Parse_RMT_item(volatile rmt_item32_t *pointer, uint32_t *dataptr)
{
    // reset data
    *dataptr = 0;

    // just in case the idle_level isn't correct
    if (pointer[0].level1)
        return false;

    int8_t val = 0, bit_pos = 0;
    uint32_t temp = 0;
    for (int i = 0; i <= RMT_data_length; i++)
    {
        // high period
        temp = pointer[i].duration0;
        // 1 tick long
        if (temp <= RMT_ticks_num + RMT_ticks_tol && temp >= RMT_ticks_num - RMT_ticks_tol)
        {
            val++;
            if (val == 2)
                *dataptr += (1 << (bit_pos++));
            else if (val != 1)
                return false;
        }
        // 2 ticks long
        else if (temp <= RMT_ticks_num * 2 + RMT_ticks_tol && temp >= RMT_ticks_num * 2 - RMT_ticks_tol)
        {
            val += 2;
            if (val == 2)
                *dataptr += (1 << (bit_pos++));
            else
                return false;
        }
        else
            return false;

        // low period
        // check termination
        if (temp = pointer[i].duration1)
        {
            // 1 tick long
            if (temp <= RMT_ticks_num + RMT_ticks_tol && temp >= RMT_ticks_num - RMT_ticks_tol)
            {
                val--;
                if (!val)
                {
                    bit_pos++;
                }
                else if (val != 1)
                    return false;
            }
            // 2 ticks long
            else if (temp <= (RMT_ticks_num + RMT_ticks_tol) * 2 && temp >= (RMT_ticks_num - RMT_ticks_tol) * 2)
            {
                val -= 2;
                if (val)
                    return false;
                else
                    bit_pos++;
            }
            else
                return false;
        }
        // if data ended, then break the loop.
        else
            break;
    }

    // check message length
    if (bit_pos == RMT_data_length)
        return true;
    else
        return false;
}