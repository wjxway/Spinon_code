#ifndef _PRINTS_HPP_
#define _PRINTS_HPP_
#pragma once
#include "Arduino.h"

// The only intent of this file is to make printing simpler with macros

/**
 * @brief Whether to enable printing upon RMT_RX_TX class initialization or other initialization processes.
 *        if this macro exists, then initialization prints are enabled. 
 */
#define INITIALIZATION_PRINT_ENABLED 1

/**
 * @brief Whether to enable printing debug contents
 *        if this macro exists, then debug prints are enabled. 
 */
#define DEBUG_PRINT_ENABLED 0

#if INITIALIZATION_PRINT_ENABLED
#define INIT_print(content) Serial.print(content)
#define INIT_println(content) Serial.println(content)
#else
#define INIT_print(content) 0
#define INIT_println(content) 0
#endif

#if DEBUG_PRINT_ENABLED
#define DEBUG_print(content) Serial.print(content)
#define DEBUG_println(content) Serial.println(content)
#else
#define DEBUG_print(content) 0
#define DEBUG_println(content) 0
#endif

#endif