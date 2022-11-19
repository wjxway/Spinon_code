/**
 * @file CRCCheck.hpp
 * @brief compute CRC for data
 * @note should only be used by IrCommunication library.
 */
#ifndef _CRCCHECK_HPP_
#define _CRCCHECK_HPP_

#include <cstddef>
#include <cstdint>

/**
 * @brief CRC function using CRC-4/ITU standard, for usage in single
 * transmission messages
 *
 * @param uint32_t input data (single transmission).
 * @return crc4 result as a uint16_t.
 */
uint16_t crc4_itu(const uint16_t data) noexcept;

/**
 * @brief CRC function using CRC-8/Maxim standard
 *
 * @param data input data pointer, should be in uint16_t form, containing
 * Msg_content_bits data each.
 * @param length input data length.
 * @return crc8 result as a uint16_t
 */
uint16_t crc8_maxim(const uint16_t *const data, const size_t length) noexcept;

/**
 * @brief CRC function using CRC-12/CDMA2000 standard
 *
 * @param data input data pointer, should be in uint16_t form, containing
 * Msg_content_bits data each.
 * @param length input data length.
 * @return crc12 result as a uint16_t
 */
uint16_t crc12_cdma(const uint16_t *const data, const size_t length) noexcept;
#endif