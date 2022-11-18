#include "CRCCheck.hpp"
#include "../IrCommunication/RMTMessageDefs.hpp"

uint16_t crc4_itu(uint16_t data)
{
    for (int i = 0; i < IR::detail::Msg_content_bits; ++i)
        data = (data >> 1) ^ ((-(data & 1)) & 0xC);
    return data;
}

uint16_t crc8_maxim(const uint16_t *data, const uint16_t length)
{
    uint16_t i, length1 = length, crc = 0;
    while (length1--)
    {
        crc ^= *data++; // crc ^= *data; data++;
        for (i = 0; i < IR::detail::Msg_content_bits; i++)
            crc = (crc >> 1) ^ ((-(crc & 1)) & 0x8C);
    }
    return crc;
}

uint16_t crc12_cdma(const uint16_t *data, const uint16_t length)
{
    uint16_t i, length1 = length, crc = 0;
    while (length1--)
    {
        crc ^= *data++; // crc ^= *data; data++;
        for (i = 0; i < IR::detail::Msg_content_bits; i++)
            crc = (crc >> 1) ^ ((-(crc & 1)) & 0xC8F);
    }
    return crc;
}