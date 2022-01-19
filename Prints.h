#ifndef _PRINTS_
#define _PRINTS_
#pragma once
#include "Arduino.h"

// The only intent of this file is to make printing simpler with macros

/**
 * @brief Whether to enable printing upon RMT_RX_TX class initialization or other initialization processes.
 *        if this macro exists, then initialization prints are enabled. 
 */
#define _INITIALIZATION_PRINT_ENABLE_ 1

/**
 * @brief Whether to enable printing debug contents
 *        if this macro exists, then debug prints are enabled. 
 */
// #define _DEBUG_PRINT_ENABLE_ 1

#ifdef _INITIALIZATION_PRINT_ENABLE_
#define INIT_print(content) Serial.print(content)
#define INIT_println(content) Serial.println(content)
#else
#define INIT_print(content) 0
#define INIT_println(content) 0
#endif

#ifdef _DEBUG_PRINT_ENABLE_
#define DEBUG_print(content) Serial.print(content)
#define DEBUG_println(content) Serial.println(content)
#else
#define DEBUG_print(content) 0
#define DEBUG_println(content) 0
#endif

#endif