/**
 * @file RobotDefs.hpp
 * @brief Global definition for robots.
 */
#ifndef ROBOTDEFS_HPP__
#define ROBOTDEFS_HPP__

#include <cstdint>

#define _USE_MATH_DEFINES
#include <cmath>

// this is a macro to define robot ID in this file (related to robot parameters tuning)
// should be removed and replaced by preferences.h for storing robot parameters
#define TRID 11

/**
 * @brief Robot's ID
 *
 * @note for valid range, please check RMTMessageDefs.hpp -> Robot_ID_bits
 */
const uint32_t This_robot_ID = TRID;

/**
 * @brief robot's weight in grams
 */
#if TRID == 11
constexpr float Robot_mass = 16.5F;
#else
constexpr float Robot_mass = 16.0F;
#endif

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
#if TRID == 11
        /**
         * @brief angle offset between left and right receiver channel in rad
         *
         * @note The actual left-right angle is LR_diff + LR_angle_compensation
         * * Cent_diff
         */
        constexpr float LR_angle_compensation = -0.03F;

        /**
         * @brief angle addition to measured elevation angle
         *
         * @note actual elevation angle is Cent_angle +
         * Elevation_angle_compensation
         */
        constexpr float Elevation_angle_compensation = -0.00174533F;

        /**
         * @brief three parameters used for distance estimation
         *
         * @note actual distance is Distance_param_a / sin((LR_angle +
         * Distance_param_b) / 2.0F) - Distance_param_c
         */
        constexpr float Distance_param_a = 39.0518F;
        constexpr float Distance_param_b = 0.0555407F;
        constexpr float Distance_param_c = -43.9161F;

        /**
         * @brief orientation angle offset.
         *
         * @note The actual orientation angle is angle +
         * Orientation_compensation * LR_diff
         *
         * @note not useful in calibration mode
         * @note pretty consistently related to geometry.
         */
        constexpr float Orientation_compensation = 0.18F;

#else
        // this is for drone 12
        /**
         * @brief angle offset between left and right receiver channel in rad
         *
         * @note The actual left-right angle is LR_diff + LR_angle_compensation
         * * Cent_diff
         */
        constexpr float LR_angle_compensation = 0.00F;

        /**
         * @brief angle addition to measured elevation angle
         *
         * @note actual elevation angle is Cent_angle +
         * Elevation_angle_compensation
         */
        constexpr float Elevation_angle_compensation = 0.0174533F;

        /**
         * @brief three parameters used for distance estimation
         *
         * @note actual distance is Distance_param_a / sin((LR_angle +
         * Distance_param_b) / 2.0F) - Distance_param_c
         */
        constexpr float Distance_param_a = 28.3525F;
        constexpr float Distance_param_b = 0.00402181F;
        constexpr float Distance_param_c = -18.7431F;

        /**
         * @brief orientation angle offset.
         *
         * @note The actual orientation angle is angle +
         * Orientation_compensation * LR_diff
         *
         * @note not useful in calibration mode
         * @note pretty consistently related to geometry.
         */
        constexpr float Orientation_compensation = 0.18F;
#endif
    } // namespace RX
} // namespace IR

#undef TRID
#endif