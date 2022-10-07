// CRC check code
#ifndef _CRCCHECK_HPP_
#define _CRCCHECK_HPP_
#pragma once

#include "Arduino.h"

/**
 * @brief CRC function using CRC-4/ITU standard, for usage in single transmission messages
 *
 * @param uint32_t input data (single transmission).
 * @return crc4 result as a uint32_t.
 */
uint32_t crc4_itu(uint32_t data);

/**
 * @brief CRC function using CRC-8/Maxim standard
 *
 * @param data input data pointer, should be in uint32_t form, containing Msg_content_bits data each.
 * @param length input data length.
 * @return crc8 result as a uint32_t
 */
uint32_t crc8_maxim(const uint32_t *data, const uint32_t length);

/**
 * @brief CRC function using CRC-12/CDMA2000 standard
 *
 * @param data input data pointer, should be in uint32_t form, containing Msg_content_bits data each.
 * @param length input data length.
 * @return crc12 result as a uint32_t
 */
uint32_t crc12_cdma(const uint32_t *data, const uint32_t length);
#endif