#include "CRCCheck.hpp"

namespace
{
    // please synchronize it with IR::detail::Msg_content_bits as defined in
    // IrCommunication/RMTMessageDefs
    constexpr uint32_t Msg_content_bits = 16;
}

const uint16_t crc4_itu(uint16_t data) noexcept
{
    for (size_t i = 0U; i < Msg_content_bits; i++)
    {
        data = (data >> 1) ^ ((-(data & 1)) & 0xC);
    }
    return data;
}

const uint16_t crc8_maxim(const uint16_t *data, const size_t length) noexcept
{
    uint16_t crc = 0;
    for (size_t j = length; j > 0U; j--)
    {
        crc ^= *data++;
        for (size_t i = 0U; i < Msg_content_bits; i++)
        {
            crc = (crc >> 1) ^ ((-(crc & 0x1)) & 0x8C);
        }
    }
    return crc;
}

const uint16_t crc12_cdma(const uint16_t *data, const size_t length) noexcept
{
    uint16_t crc = 0;
    for (size_t j = length; j > 0U; j--)
    {
        crc ^= *data++;
        for (uint16_t i = 0U; i < Msg_content_bits; i++)
        {
            crc = (crc >> 1) ^ ((-(crc & 0x1)) & 0xC8F);
        }
    }
    return crc;
}