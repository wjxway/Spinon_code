#include "CRCCheck.hpp"
#include "..\RMTMessageDefs.hpp"

uint32_t crc4_itu(uint32_t data)
{
    for (int i = 0; i < IR::detail::Msg_content_bits; ++i)
        data = (data >> 1) ^ ((-(data & 1)) & 0xC);
    return data;
}

uint32_t crc8_maxim(const uint32_t *data, const uint32_t length)
{
    uint32_t i, length1 = length, crc = 0;
    while (length1--)
    {
        crc ^= *data++; // crc ^= *data; data++;
        for (i = 0; i < IR::detail::Msg_content_bits; i++)
            crc = (crc >> 1) ^ ((-(crc & 1)) & 0x8C);
    }
    return crc;
}

uint32_t crc12_cdma(const uint32_t *data, const uint32_t length)
{
    uint32_t i, length1 = length, crc = 0;
    while (length1--)
    {
        crc ^= *data++; // crc ^= *data; data++;
        for (i = 0; i < IR::detail::Msg_content_bits; i++)
            crc = (crc >> 1) ^ ((-(crc & 1)) & 0xC8F);
    }
    return crc;
}