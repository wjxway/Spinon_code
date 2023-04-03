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
 * @brief robot's weight in grams
 */
// constexpr float Robot_mass = 18.9F;
constexpr float Robot_mass = 20.0F;

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
 * @brief should we be in calibration mode?
 *
 * @note in calibration mode, we basically output all raw measurement data we
 * obtained through serial. (buffer might be required)
 */
// #define LOCALIZATION_CALIBRATION_MODE 1

namespace IR
{
    namespace RX
    {
        /**
         * @brief angle offset between left and right receiver channel in rad
         *
         * @note The actual left-right angle is LR_diff + LR_angle_compensation
         * * Cent_diff
         *
         * @note not useful in calibration mode
         */
        constexpr float LR_angle_compensation = 0.0F;

        /**
         * @brief orientation angle offset.
         *
         * @note The actual orientation angle is angle +
         * Orientation_compensation * LR_diff
         *
         * @note not useful in calibration mode
         */
        constexpr float Orientation_compensation = 0.18F;
    } // namespace RX
} // namespace IR

#endif