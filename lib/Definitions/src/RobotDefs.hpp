/**
 * @file RobotDefs.hpp
 * @brief Global definition for robots.
 */
#ifndef ROBOTDEFS_HPP__
#define ROBOTDEFS_HPP__

#include <cstdint>

#define _USE_MATH_DEFINES
#include <cmath>

/**
 * @brief Robot's ID
 *
 * @note for valid range, please check RMTMessageDefs.hpp -> Robot_ID_bits
 */
const uint32_t This_robot_ID = 12U;

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

/**
 * @brief set to 1 to enter calibration mode where we output all raw data.
 */
#define LOCALIZATION_CALIBRATION_MODE 1

namespace IR
{
    namespace RX
    {
        /**
         * @brief angle offset between left and right receiver channel in rad
         *
         * @note The actual left-right delay is T_right - T_left + angle_offset
         * / angular_velocity
         * 
         * @note not useful in calibration mode
         */
        constexpr float Left_right_angle_offset = 2.75F / 180.0F * M_PI;

        /**
         * @brief angle offset between the average of left and right receiver
         * channel and the center receiver channel in rad
         *
         * @note The actual center delay is T_center - T_avg + angle_offset /
         * angular_velocity
         * 
         * @note not useful in calibration mode
         */
        constexpr float Center_angle_offset = -0.3F / 180.0F * M_PI;
    } // namespace RX
} // namespace IR

#endif