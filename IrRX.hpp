// Encoding & decoding of RMT signal
#ifndef _IRRX_HPP_
#define _IRRX_HPP_
#pragma once

#include "Arduino.h"
#include "Utilities/Circbuffer.hpp"
#include "RobotDefs.hpp"
#include "RMTMessageDefs.hpp"
#include "driver/rmt.h"
#include <vector>

namespace IR
{
    namespace RX
    {
        /**
         * @brief initialize RX routine, including:
         *        1. initialize RX RMT channels
         *        2. initialize RX ISR
         *        3. initialize preprocess data structures, routine, and timer
         */
        void Init();

        /**
         * @brief RX ISR that handles input RMT transmissions and store them into a buffer.
         */
        void IRAM_ATTR RX_ISR();

        /**
         * @brief get raw uint32_t data from buffer(data comes from RX_ISR) and organize and store all parsed data packet.
         *        this function shall be called at ~100Hz rate (just not very frequently).
         */
        void Preprocess();

        /**
         * @brief analyze data organized by RX_preprocess.
         *        output to two buffers, one data buffer, and one relative position buffer.
         */
        void Analysis();

        /**
         * @brief execute localization based on relative position and data buffers
         */
        void Localize();

        /**
         * @brief get current absolute position
         */
        void Get_position();
        
        /**
         * @brief a bunch of helper functions like get all relative position, get timing, etc...
         */
    }
}

#endif