/**
 * @file IrSense.hpp
 * @brief transmit data and sense interface
 * 
 * @warning this is still in development and is not compatible with original IrTX library
 */
#ifndef IRSENSE_HPP__
#define IRSENSE_HPP__

#include "Arduino.h"
#include <vector>
#include <RobotDefs.hpp>

namespace IR
{
    namespace Sense
    {
        void Init();

        /**
         * @brief Transmit a message and return the RSSI
         * 
         * @return std::array<int32_t,RMT_RX_CHANNEL_COUNT> RSSI strength
         * 
         * @warning **Experimental** calling this function will transmit a pulse
         * of type 0 containing message 0, and return the RSSI of the very first
         * pulse. This function is not compatible with IrCommunication interface
         * now!
         */
        std::array<int16_t,RMT_RX_CHANNEL_COUNT> Transmit_and_sense(int32_t ticks_delay);
    } // namespace Sense
} // namespace IR

#endif