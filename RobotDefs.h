#ifndef _ROBOTDEFS_
#define _ROBOTDEFS_
#pragma once

#include "Arduino.h"
#include "driver/rmt.h"

// number of receivers
#define N_RECEIVERS 3
// whether to enable the emitter
#define EMITTER_ENABLED 1
// Robot's ID
constexpr uint32_t THIS_ROBOT_ID = 5;

// pins def
// For pins def, use #define for compatibility with FastIO lib.
// Usually you don't need these pins to be typed to write correct code,
// so no need for defining them using constexpr and bring more trouble with FastIO.
// RMT pins def
#define RMT_OUT 25
#define RMT_IN_1 26
#if N_RECEIVERS >= 2
#define RMT_IN_2 27
#endif
#if N_RECEIVERS == 3
#define RMT_IN_3 32
#endif

// test pins
#define TEST_PIN 16

#define LED_PIN_1 17
#define LED_PIN_2 18
#define LED_PIN_3 19

// //HSPI pins
// #define HSCK 14
// #define HMISO 12
// #define HMOSI 13
// #define HSS 15

// //VSPI pins
// #define VSCK 18
// #define VMISO 19
// #define VMOSI 23

#endif