#include "CRCCheck.hpp"
#include "../IrCommunication/RMTMessageDefs.hpp"

uint16_t crc4_itu(uint16_t data)
{
    for (size_t i = 0U; i < IR::detail::Msg_content_bits; ++i)
    {
        data = (data >> 1) ^ ((-(data & 0x1)) & 0xC);
    }
    return data;
}

uint16_t crc8_maxim(const uint16_t *data, const size_t length)
{
    uint16_t crc = 0;
    for (size_t j = length; j > 0U; j--)
    {
        crc ^= *data++;
        for (size_t i = 0U; i < IR::detail::Msg_content_bits; i++)
        {
            crc = (crc >> 1) ^ ((-(crc & 0x1)) & 0x8C);
        }
    }
    return crc;
}

uint16_t crc12_cdma(const uint16_t *data, const size_t length)
{
    uint16_t crc = 0;

    for (size_t j = length; j > 0U; j--)
    {
        crc ^= *data++;
        for (uint16_t i = 0U; i < IR::detail::Msg_content_bits; i++)
        {
            crc = (crc >> 1) ^ ((-(crc & 0x1)) & 0xC8F);
        }
    }

    return crc;
}
