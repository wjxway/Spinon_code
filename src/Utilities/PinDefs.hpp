/**
 * @file PinDefs.hpp
 * @brief global definition for pin and hardware resource mapping.
 */
#ifndef _PINDEFS_HPP_
#define _PINDEFS_HPP_
#pragma once

#include "../RobotDefs.hpp"
#include "hal/rmt_types.h"

// pins def
// For pins def, use #define for compatibility with FastIO lib.
// Usually you don't need these pins to be typed to write correct code,
// so no need for defining them using constexpr and bring more trouble with FastIO.

// RMT pins def
// If we enable both TX channnels (as in our real hardware)
// the convention will be:
//      RMT_TX_PIN_1 -> bottom emitter (shorter pulses at the end)
//      RMT_TX_PIN_2 -> top emitter (longer pulses at the end)
// Note that please make sure that when both can be seen, the top emitter will be seen FIRST.
#if RMT_TX_CHANNEL_COUNT >=1
#define RMT_TX_PIN_1 18
#endif
#if RMT_TX_CHANNEL_COUNT == 2
#define RMT_TX_PIN_2 10
#endif

// If we enable all three RX channnels (as in our real hardware)
// the convention will be:
//      RMT_RX_PIN_1 -> left emitter
//      RMT_RX_PIN_2 -> right emitter
//      RMT_RX_PIN_3 -> middle emitter
#if RMT_RX_CHANNEL_COUNT >=1
#define RMT_RX_PIN_1 22
#endif
#if RMT_RX_CHANNEL_COUNT >= 2
#define RMT_RX_PIN_2 19
#endif
#if RMT_RX_CHANNEL_COUNT == 3
#define RMT_RX_PIN_3 23
#endif

// // this is the old one
// #define RMT_OUT_PIN_1 4
// #define RMT_OUT_PIN_2 23
// #define RMT_IN_PIN_1 22
// #if RMT_RX_CHANNEL_COUNT >= 2
// #define RMT_IN_PIN_2 26
// #endif
// #if RMT_RX_CHANNEL_COUNT == 3
// #define RMT_IN_PIN_3 21


// test pins
#define DEBUG_PIN_1 4
#define DEBUG_PIN_2 9

// #define DEBUG_PIN_3 25

// R,G,B channels respectively
// pull low to lit
// when updating this, please also update DebugDefs
// because LED_PIN_R is >=32, it cannot be directly toggled using setbit and clrbit
#define LED_PIN_R 32
#define LED_PIN_G 33
#define LED_PIN_B 25

// analog input for polarized light sensor
#define PL_PIN 34

// motor pins
#define MOTOR_ALERT_PIN 26
#define MOTOR_SPD_FB_PIN 27
#define MOTOR_SDA_PIN 14
#define MOTOR_SCL_PIN 12
#define MOTOR_SPD_CTRL_PIN 13
#define MOTOR_BRAKE_PIN 15

// // this is the old one
// #define TEST_PIN 27
// #define TEST_PIN_2 23
// // R,G,B channels now redirected to external pins
// #define LED_PIN_1 18
// #define LED_PIN_2 19
// #define LED_PIN_3 16
// //#define LED_PIN_3 23

// // HSPI pins
// #define HSCK_PIN 14
// #define HMISO_PIN 12
// #define HMOSI_PIN 13
// #define HSS_PIN 15
// // VSPI pins
// #define VSCK 18
// #define VMISO 19
// #define VMOSI 23

// RMT channel and timer definitions
/**
 * @brief RMT TX channel 1 num (lower emitter)
 */
constexpr rmt_channel_t RMT_TX_channel_1 = RMT_CHANNEL_0;
/**
 * @brief RMT TX channel 2 num (upper emitter)
 */
constexpr rmt_channel_t RMT_TX_channel_2 = RMT_CHANNEL_1;
/**
 * @brief First RMT RX channel num (Highest priority)
 */
constexpr rmt_channel_t RMT_RX_channel_1 = RMT_CHANNEL_2;
#if RMT_RX_CHANNEL_COUNT >= 2
/**
 * @brief Second RMT RX channel num
 */
constexpr rmt_channel_t RMT_RX_channel_2 = RMT_CHANNEL_4;
#endif
#if RMT_RX_CHANNEL_COUNT == 3
/**
 * @brief Third RMT RX channel num (Lowest priority)
 */
constexpr rmt_channel_t RMT_RX_channel_3 = RMT_CHANNEL_6;
#endif

/**
 * @brief Timer channel used for IR TX trigger
 */
constexpr uint32_t IR_TX_trigger_timer_channel = 3u;

/**
 * @brief motor's i2c address
 */
constexpr uint8_t Motor_address = 0b0110010u;

/**
 * @brief Motor LEDC channel
 */
constexpr uint32_t Motor_LEDC_PWM_channel = 0u;

#endif