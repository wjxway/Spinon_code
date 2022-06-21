#ifndef _ROBOTDEFS_HPP_
#define _ROBOTDEFS_HPP_
#pragma once
#include "Arduino.h"

// whether to enable debug led & digital output
#define DEBUG_LED_ENABLED 0

// number of receivers
#define RMT_RX_CHANNEL_COUNT 3
// number of emitters
#define RMT_TX_CHANNEL_COUNT 2

// Robot's ID
constexpr uint32_t THIS_ROBOT_ID = 12;

// pins def
// For pins def, use #define for compatibility with FastIO lib.
// Usually you don't need these pins to be typed to write correct code,
// so no need for defining them using constexpr and bring more trouble with FastIO.
// RMT pins def
#define RMT_OUT_PIN_1 4
#define RMT_OUT_PIN_2 23
#define RMT_IN_PIN_1 22
#if RMT_RX_CHANNEL_COUNT >= 2
#define RMT_IN_PIN_2 26
#endif
#if RMT_RX_CHANNEL_COUNT == 3
#define RMT_IN_PIN_3 21
#endif

// test pins
#define TEST_PIN 16
#define TEST_PIN_2 16
// #define TEST_PIN 27
// #define TEST_PIN_2 23

// // R,G,B channels respectively
// #define LED_PIN_1 33
// #define LED_PIN_2 25
// #define LED_PIN_3 32

// R,G,B channels now redirected to external pins
#define LED_PIN_1 18
#define LED_PIN_2 19
#define LED_PIN_3 16
//#define LED_PIN_3 23

//HSPI pins
#define HSCK_PIN 14
#define HMISO_PIN 12
#define HMOSI_PIN 13
#define HSS_PIN 15

// //VSPI pins
// #define VSCK 18
// #define VMISO 19
// #define VMOSI 23

#endif