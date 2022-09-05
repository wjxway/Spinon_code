// Definitions for RMT messages
#ifndef _RMTMESSAGEDEFS_HPP_
#define _RMTMESSAGEDEFS_HPP_
#pragma once

#include "Utilities/PinDefs.hpp"

// Because I implemented a HPF in the filter to deal with change in environmental light and irrelavent blinks,
// The data we are sending must have a fixed portion of low level and a fixed portion of high level, evenly distributed.
// A natural idea would be to use Manchester encoding. (This is also a commonly used IR protocol)
// But the problem with such method is that pulses can be merged into a single larger pulse, and each pulse contains only one bit of info.
// This will bring difficulty to the parsing process.

// So what I do right now is use a modified 4ppm method (let's call it 1+4ppm) where a unit of empty space is added between two 4ppm cycle.
// Now pulses won't merge and each pulse represent two bits. an additional benifit is that the power consumption is lower too due to smaller duty.
//   0 -> 00001
//   1 -> 00010
//   2 -> 00100
//   3 -> 01000
// To identify the top and bottom emitter while allowing them to be transmitted at the same time (I don't want to use up the multiplexing ability!)
// I make the last pulse to be variable in length, and the emitter that will always be seen first (by convention, the top one) will have longer pulse.
// So that if the drone can see the top emitter or both, it will localize based on the top emitter's position.
// For example, a signal of 01100011 will be translated into:
//   01100011 -> 1|00010|00100|00001|01(1)
// where the first 1 is the starting signal, and 01(1) is the last two bits, which can be long or short depending on the emitter.

// Each raw message is in the following form:
// | ---robot_ID--- | ---msg_ID--- | ---Data or header info--- | (---ECC--- |)
// where ID is the robot's id, MSG_ID is the ID of message, when all msg_id sent from robot_id is received, then a transmission is complete.
// I think ECC bit is not necessary, because it is highly unlikely that the received signal is distorted in a way that
// result in a wrong signal instead of a bad signal.
// I will test that in the reality before deciding whether to add in the ECC bits.
//
// Messages with Msg_ID = x000.. is header message, which contains information about how many raw message would complete the data and probably some ECC.
//
// Note that this header file is for messages <=32 bits, if you want to use larger messages, change uint32_t to uint64_t.

// Implementation details are hide in this namespace to keep global clean
namespace detail
{
    // message content specifications

    /**
     * @brief number of bits for robot's id.
     */
    constexpr uint32_t Robot_ID_bits = 6;

    /**
     * @brief number of bits for message's id.
     * @note message ID on messages for the same data should have same initial bit,
     *       so that we can distinguish from different data groups sent by the same robot at different time.
     *       If the robot received a message with the other intial bit, it should discard all previous incomplete messages
     *       With the previous initial bit, because the data they contains are already obsolete.
     *       Should set to <= 7, for two reasons.
     *          1. With Msg_ID_bits <= 8, the ID has <= 7 bits, thus we can use a uint64_t to indicate whether we received all the data. Save space & time.
     *          2. The full data should be transmitted in <30 deg time, and with 0.5 deg / message, this means 2^6 messages. Adding up with the header bit, it's 7 bits.
     */
    constexpr uint32_t Msg_ID_bits = 6;

    /**
     * @brief number of BYTES for message's content.
     */
    constexpr uint32_t Msg_content_bytes = 2;

    /**
     * @brief number of bits for message's content.
     *
     * @note please make sure that this is a multiplier of 8 for easy conversion.
     *       If not, you will have to write your own function to convert an arbitrary data into a array of that many bits.
     *       And you will have to write your own ECC as well.
     */
    constexpr uint32_t Msg_content_bits = Msg_content_bytes * 8;

    /**
     * @brief The amount of data, in bits, that's transmitted in each RMT transmission.
     *
     * @note RMT length should be kept below 63 for convenience of processing (leaving some space for init code and probably ECC.)
     * When RMT length is larger than 63, it will occupy at least two RMT memory register block. That's inconvenient and requires much more processing.
     * I would suggest using rmt length from 16 to 32. dUsing rmt length >32 then you will need to modify the code here and there, changing uint32_t to uint64_t.
     *
     * @note Keep this an even number if using 4ppm encoding!
     *
     * @note for simplicity, make sure that this is <= 31, we are currently using the 32th bit to pass the receiver's position (top or bottom).
     */
    constexpr uint32_t RMT_data_length = Robot_ID_bits + Msg_ID_bits + Msg_content_bits;


    // data structure specification

    /**
     * @brief general structure of messages with <=32 bits.
     */
    typedef union
    {
        struct
        {
            uint32_t robot_ID : detail::Robot_ID_bits;
            uint32_t msg_ID_init : 1;
            uint32_t msg_ID : detail::Msg_ID_bits - 1;
            uint32_t content : detail::Msg_content_bits;
        };
        uint32_t raw;
    } Msg_t;

    /**
     * @brief general structure of message headers with <=32 bits.
     */
    typedef union
    {
        struct
        {
            uint32_t robot_ID : detail::Robot_ID_bits;
            uint32_t msg_ID_init : 1;
            uint32_t msg_ID : detail::Msg_ID_bits - 1;
            uint32_t msg_ID_len : detail::Msg_ID_bits - 1; // Msg_ID will be from 0 (the header) to Msg_ID_len.
            uint32_t CRC : detail::Msg_content_bits - detail::Msg_ID_bits + 1;
        };
        uint32_t raw;
    } Msg_header_t;

    /**
     * @brief a struct packed with data / receiver state / time of reception, which is all information contained in a transmission.
     *
     */
    struct Trans_info
    {
        uint32_t data;
        uint32_t receiver;
        uint64_t time;
    };


    // RMT timing specifications

    /**
     * @brief RMT clock division, rmt clock speed = 80MHz / RMT_clock_div
     * @note RMT_ticks_num * RMT_clock_div * 1/80MHz is the pulse width.
     */
    constexpr uint32_t RMT_clock_div = 2;

    /**
     * @brief Ticks count of each pulse
     * @note RMT_ticks_num * RMT_clock_div * 1/80MHz is the pulse width.
     */
    constexpr uint32_t RMT_ticks_num = 4;

    /**
     * @brief Ticks number that deviate from RMT_ticks_num by RMT_ticks_tol will still be accepted
     * @note There will be error in ticks number in reality, so we could give it some tolerance
     */
    constexpr uint32_t RMT_ticks_tol = 1;

    /**
     * @brief Number of bits per cycle/pulse
     * @note Please keep it at 2 for simplicity, making it larger won't improve the performance much further.
     */
    constexpr uint32_t Bit_per_cycle = 2;

    /**
     * @brief Padding before each cycle in ticks
     */
    constexpr uint32_t Pad_per_cycle = RMT_ticks_num;

    /**
     * @brief RMT data pulse count.
     *
     * @note RMT_data_pulse_count = RMT_data_length / Bit_per_cycle
     */
    constexpr uint32_t RMT_data_pulse_count = RMT_data_length / Bit_per_cycle;

    /**
     * @brief RMT TX data structure length (array of rmt_item32_t).
     *
     * @note RMT_TX_length = RMT_data_pulse_count + 2 because there will be a '0' header and a {0} as ending block
     */
    constexpr uint32_t RMT_TX_length = RMT_data_pulse_count + 2;


    // storage and buffer specifications

    /**
     * @brief maximum memory size for data pool.
     */
    constexpr uint32_t Msg_memory_size = ((1 << (Msg_ID_bits - 1)) - 1) * Msg_content_bytes;

    /**
     * @brief maximum number of robots that can communicate with a single robot in the same period of time.
     */
    constexpr uint32_t Max_robots_simultaneous = 8;

    /**
     * @brief Fragments pool's timing data's decay time. If nothing is added to the pool for this amount of time (in us),
     *        then the timing info should be discarded.
     *
     * @note A proper value for Timing_expire_time should be the time it takes for the robot to spin 0.8 rounds.
     */
    constexpr uint64_t Timing_expire_time = 60000;

    /**
     * @brief Fragments pool itself's decay time. If nothing is added to the pool for this amount of time (in us),
     *        then this robot has probably leaved the communication range and this pool should be re-purposed.
     *
     * @note Data_expire_time should be larger than Timing_expire_time.
     *       A proper value for Data_expire_time should be 1.5 times the minimum span between two consecutive TX_load(...)
     *       So that it's not too short, but still can prevent msg_ID_init from duplication.
     */
    constexpr uint64_t Data_expire_time = 1000000;

    /**
     * @brief RMT TX Trigger period in us
     */
    constexpr uint64_t RMT_TX_trigger_period = 50;

    /**
     * @brief how many messages are allowed to stay in the buffer
     */
    constexpr uint32_t Msg_buffer_max_size = 200;

    /**
     * @brief how frequently we read data from buffer and process it (in ms)
     *
     * @note because we have low duty cycle, we can rest for a while and let other tasks work...
     *       the signal's period is 100us, and there's seldom >3 emitters
     *       so in 5ms, there's at most 150 messages we need to process.
     */
    constexpr uint32_t Msg_process_period = 5;
}

#endif