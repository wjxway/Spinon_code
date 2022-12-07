/**
 * @file DebugDefs.hpp
 * @brief macros used for debugging
 */
#ifndef DEBUGDEFS_HPP__
#define DEBUGDEFS_HPP__

#include "Arduino.h"
#include "FastIO.hpp"
#include <RobotDefs.hpp>

// wrap DEBUG_C around all debug code.
#if DEBUG_ENABLED
#define DEBUG_C(c) c
#else
#define DEBUG_C(c) 0
#endif

// lED defs should be updated when 
#if LED_ENABLED
#define LIT_R (GPIO.out1_w1tc.data = 1)
#define QUENCH_R (GPIO.out1_w1ts.data = 1)
#define LIT_G (GPIO.out1_w1tc.data = 2)
#define QUENCH_G (GPIO.out1_w1ts.data = 2)
#define LIT_B clrbit(LED_PIN_B)
#define QUENCH_B setbit(LED_PIN_B)
#else
#define LIT_R 0
#define QUENCH_R 0
#define LIT_G 0
#define QUENCH_G 0
#define LIT_B 0
#define QUENCH_B 0
#endif

#endif