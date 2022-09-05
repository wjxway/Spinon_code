// Encoding & decoding of RMT signal
#ifndef _RMTCODING_HPP_
#define _RMTCODING_HPP_
#pragma once

#include "Arduino.h"
#include "soc/rmt_struct.h"

/**
 * @brief Generate rmt item for emission based on input data
 *
 * @note For rmt length >32, change uint32_t to uint64_t or larger
 *
 * @param pointer Pointer to a pre-defined space of rmt item storage. The elements inside the array are volatile.
 * @param data Input data in an integer form.
 * @param ticks_delay How many ticks of delay before the data, for time sync.
 * @param ticks_final How many ticks for the final pulse in the data.
 *
 * @return true Successful!
 * @return false Unable to generate.
 */
bool Generate_RMT_item(rmt_item32_t *pointer, uint32_t data, uint32_t ticks_delay, uint32_t ticks_final);

/**
 * @brief Parse rmt item to data.
 *
 * @note For rmt length >32, change uint32_t to uint64_t or larger
 *
 * @param pointer1 Pointer to raw rmt item storage. The elements inside the array are volatile.
 * @param dataptr Pointer to the output space, the output is an integer.
 *
 * @return true Successful!
 * @return false Data invalid!
 * 
 * @note each call takes ~2.2us
 */
bool IRAM_ATTR Parse_RMT_item(volatile rmt_item32_t *pointer, uint32_t *dataptr);

#endif