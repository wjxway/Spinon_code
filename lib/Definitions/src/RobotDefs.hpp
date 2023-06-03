/**
 * @file RobotDefs.hpp
 * @brief Global definition for robots.
 */
#ifndef ROBOTDEFS_HPP__
#define ROBOTDEFS_HPP__

#include <cstdint>

#define _USE_MATH_DEFINES
#include <cmath>

// DRONE_MODE = 0 -> RX+TX, spinning drone
// DRONE_MODE = 1 -> RX only, static message relay drone
#define DRONE_MODE 0

/**
 * @brief Robot's ID
 *
 * @note for valid range, please check RMTMessageDefs.hpp -> Robot_ID_bits
 */
const uint32_t This_robot_ID = 12U - DRONE_MODE;

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

#endif