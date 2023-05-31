/**
 * @file RobotDefs.hpp
 * @brief Global definition for robots.
 */
#ifndef ROBOTDEFS_HPP__
#define ROBOTDEFS_HPP__

#define _USE_MATH_DEFINES
#include <cstdint>
#include <cmath>

/**
 * @brief whether to enable led output
 */
#define LED_ENABLED 1
/**
 * @brief whether to enable debug
 *
 * @note this is linked to debug prints, debug pins, and debug leds
 */
#define DEBUG_ENABLED 0

/**
 * @brief number of receivers, by default is 3
 */
#define RMT_RX_CHANNEL_COUNT 3
/**
 * @brief number of emitters, by default is 2
 */
#define RMT_TX_CHANNEL_COUNT 2

/**
 * @brief Robot's ID
 *
 * @note for valid range, please check RMTMessageDefs.hpp -> Robot_ID_bits
 */
extern uint32_t This_robot_ID;

/**
 * @brief robot's weight in grams
 */
extern float Robot_mass;

namespace IR
{
    namespace RX
    {
        /**
         * @brief angle offset between left and right receiver channel in rad
         *
         * @note The actual left-right angle is LR_diff + LR_angle_compensation
         * * Cent_diff
         */
        extern float LR_angle_compensation;
        /**
         * @brief angle addition to measured elevation angle
         *
         * @note actual elevation angle is Cent_angle +
         * Elevation_angle_compensation
         */
        extern float Elevation_angle_compensation;
        /**
         * @brief orientation angle offset.
         *
         * @note The actual orientation angle is angle +
         * Orientation_compensation * LR_diff
         *
         * @note not useful in calibration mode
         * @note pretty consistently related to geometry.
         */
        extern float Orientation_compensation;

        /**
         * @brief three parameters used for distance estimation
         *
         * @note actual distance is Distance_param_a / sin((LR_angle +
         * Distance_param_b) / 2.0F) - Distance_param_c
         */
        extern float Distance_param_a;
        extern float Distance_param_b;
        extern float Distance_param_c;
    } // namespace RX
} // namespace IR

/**
 * @brief call this function to initialize all global variables from NVM
 *
 * @return bool whether init is successful
 */
bool Init_global_parameters();

/**
 * @brief call this function to write global variables' values to NVM
 *
 * @param TRID This_robot_ID
 * @param Rmass Robot_mass
 * @param LRAcomp IR::RX::LR_angle_compensation
 * @param EAcomp IR::RX::Elevation_angle_compensation
 * @param Ocomp IR::RX::Orientation_compensation
 * @param DparamA IR::RX::Distance_param_a
 * @param DparamB IR::RX::Distance_param_b
 * @param DparamC IR::RX::Distance_param_c
 * @return bool whether init is successful
 */
bool Write_global_parameters(uint32_t TRID, float Rmass, float LRAcomp, float EAcomp, float Ocomp, float DparamA, float DparamB, float DparamC);
#endif