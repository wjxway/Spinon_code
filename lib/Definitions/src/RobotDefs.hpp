/**
 * @file RobotDefs.hpp
 * @brief Global definition for robots.
 */
#ifndef _ROBOTDEFS_HPP_
#define _ROBOTDEFS_HPP_

#include <cstdint>

/**
 * @brief Robot's ID
 *
 * @note for valid range, please check RMTMessageDefs.hpp -> Robot_ID_bits
 */
const uint32_t This_robot_ID = 12u;

/**
 * @brief whether to enable led output
 */
#define LED_ENABLED 1
/**
 * @brief whether to enable debug
 *
 * @note this is linked to debug prints, debug pins, and debug leds
 */
#define DEBUG_ENABLED 1

/**
 * @brief number of receivers, by default is 3
 */
#define RMT_RX_CHANNEL_COUNT 3
/**
 * @brief number of emitters, by default is 2
 */
#define RMT_TX_CHANNEL_COUNT 2

namespace IR
{
    namespace RX
    {
        /**
         * @brief timing offset between left and right receiver channel
         *
         * @note The actual left-right delay is T_right - T_left + offset
         */
        constexpr int64_t Left_right_timing_offset = 650;

        /**
         * @brief timing offset between the average of left and right receiver
         * channel and the center receiver channel
         *
         * @note The actual center delay is T_center - T_avg + offset
         */
        constexpr int64_t Center_timing_offset = 0;
    }
}

#endif