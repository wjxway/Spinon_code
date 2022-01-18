// Encoding & decoding of RMT signal
#ifndef _DATATRANSMISSION_
#define _DATATRANSMISSION_
#pragma once

#include "Arduino.h"
#include "RMTCoding.h"
#include <vector>

// Each raw message is in the following form:
// | ---Robot_ID--- | ---Msg_ID--- | ---Data--- | (---ECC--- |)
// where ID is the robot's id, MSG_ID is the ID of message, when all msg_id sent from robot_id is received, then a transmission is complete.
// I think ECC bit is not necessary, because it is highly unlikely that the received signal is distorted in a way that
// result in a wrong signal instead of a bad signal.
// I will test that in the reality before deciding whether to add in the ECC bits.
//
// Messages with Msg_ID = x000.. is header message, which contains information about how many raw message would complete the data and probably some ECC.
//
// Note that this header file is for messages <=32 bits, if you want to use larger messages, change uint32_t to uint64_t.

namespace detail
{
    /**
     * @brief number of bits for robot's id.
     */
    constexpr uint8_t Robot_ID_bits = 4;

    /**
     * @brief number of bits for message's id.
     * @note message ID on messages for the same data should have same initial bit,
     *       so that we can distinguish from different data groups sent by the same robot at different time.
     *       If the robot received a message with the other intial bit, it should discard all previous incomplete messages
     *       With the previous initial bit, because the data they contains are already obsolete.
     *       Should set to <=7, for two reasons.
     *          1. With Msg_ID_bits <= 7 we can use a uint64_t to indicate whether we received all the data. Save space & time.
     *          2. The full data should be transmitted in <30 deg time, and with 0.5 deg / message, this means 2^6 messages. Adding up with the header bit, it's 7 bits.
     */
    constexpr uint8_t Msg_ID_bits = 7;

    /**
     * @brief number of BYTES for message's content.
     */
    constexpr uint8_t Msg_content_bytes = 2;

    /**
     * @brief maximum memory size for data pool.
     */
    constexpr uint32_t Msg_memory_size = ((1 << (Msg_ID_bits - 1)) - 1) * Msg_content_bytes;

    /**
     * @brief number of bits for message's content.
     *
     * @note please make sure that this is a multiplier of 8 for easy conversion.
     *       If not, you will have to write your own function to convert an arbitrary data into a array of that many bits.
     *       And you will have to write your own ECC as well.
     */
    constexpr uint8_t Msg_content_bits = Msg_content_bytes * 8;

    /**
     * @brief The amount of data, in bits, that's transmitted in each RMT transmission.
     *
     * @note RMT length should be kept below 63 for convenience of processing (leaving some space for init code and probably ECC.)
     * When RMT length is larger than 63, it will occupy at least two RMT memory register block. That's inconvenient and requires much more processing.
     * I would suggest using rmt length from 16 to 32. dUsing rmt length >32 then you will need to modify the code here and there, changing uint32_t to uint64_t.
     */
    extern const uint8_t RMT_data_length;

    /**
     * @brief RMT TX sequnce length.
     *
     * @note RMT_TX_length = RMT_data_length + 2 because there will be a '0' header and a ending block
     */
    const uint8_t RMT_TX_length = RMT_data_length + 2;

    /**
     * @brief Fragments pool's timing data's decay time. If nothing is added to the pool for this amount of time (in us),
     *        then the timing info should be discarded.
     */
    constexpr uint64_t Timing_expire_time = 100000;

    /**
     * @brief maximum number of robots that can communicate with a single robot in the same period of time.
     */
    constexpr uint32_t Max_robots_simultaneous = 5;

    /**
     * @brief Fragments pool itself's decay time. If nothing is added to the pool for this amount of time (in us),
     *        then this robot has probably leaved the communication range and this pool should be re-purposed.
     */
    constexpr uint64_t Data_expire_time = 2000000;

    /**
     * @brief RMT TX channel num
     */
    constexpr rmt_channel_t RMT_TX_CHANNEL = RMT_CHANNEL_0;
    /**
     * @brief First RMT RX channel num (Highest priority)
     */
    constexpr rmt_channel_t RMT_RX_CHANNEL_1 = RMT_CHANNEL_2;
#if RMT_RX_CHANNEL_COUNT >= 2
    /**
     * @brief Second RMT RX channel num
     */
    constexpr rmt_channel_t RMT_RX_CHANNEL_2 = RMT_CHANNEL_4;
#endif
#if RMT_RX_CHANNEL_COUNT == 3
    /**
     * @brief Third RMT RX channel num (Lowest priority)
     */
    constexpr rmt_channel_t RMT_RX_CHANNEL_3 = RMT_CHANNEL_6;
#endif

    /**
     * @brief Timer channel used for RMT TX trigger
     */
    constexpr uint8_t RMT_TX_TRIGGER_TIMER_CHANNEL = 3;

    /**
     * @brief RMT TX Trigger period in us
     */
    constexpr uint64_t RMT_TX_TRIGGER_PERIOD = 500;
}

/**
 * @brief general structure of messages with <=32 bits.
 */
typedef union
{
    struct
    {
        uint32_t Robot_ID : detail::Robot_ID_bits;
        uint32_t Msg_ID_init : 1;
        uint32_t Msg_ID : detail::Msg_ID_bits - 1;
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
        uint32_t Robot_ID : detail::Robot_ID_bits;
        uint32_t Msg_ID_init : 1;
        uint32_t Msg_ID : detail::Msg_ID_bits - 1;
        uint32_t Msg_ID_len : detail::Msg_ID_bits - 1; // Msg_ID will be from 0 (the header) to Msg_ID_len.
        uint32_t CRC : detail::Msg_content_bits - detail::Msg_ID_bits + 1;
    };
    uint32_t raw;
} Msg_header_t;

/**
 * @brief CRC function using CRC-8/Maxim standard
 *
 * @param data input data pointer, should be in uint8_t form.
 * @param length input data length.
 * @return crc8 result as a int.
 *
 * @note copied from to https://github.com/whik/crc-lib-c
 *       could change to other crc functions for less collision chance when the message is longer.
 */
uint8_t crc8_maxim(uint8_t *data, uint16_t length);

/**
 * @brief RMT TX preperation class. Used to process signal emissions from a single robot ID.
 */
class RMT_TX_prep
{
public:
    // robot id
    uint32_t robot_ID = 0;
    // msg id initial
    uint32_t msg_id_initial = 0;

    // output data
    std::vector<rmt_item32_t> output;
    // output data position pointer
    uint32_t output_pos = 0;
    // output ready flag
    // 0 for all set, 1 for updating the header, 2 for updating the content.
    volatile uint16_t output_ready_flag = 0;
    volatile uint16_t reader_count_flag = 0;

    /**
     * @brief Construct a new RMT_TX_prep object.
     *
     * @param rid Robot's ID, please make sure it's smaller than 2^Robot_ID_bits, I will not check.
     */
    RMT_TX_prep(uint32_t rid) : robot_ID(rid)
    {
    }

    /**
     * @brief Load raw data and convert them into rmt_item32_t objects.
     *
     * @param raw_1 Raw data's pointer.
     * @param raw_length_1 Raw data's length in bytes.
     *
     * @return true TX load successful!
     * @return false TX load failed!
     *
     * @note msg_id_initial will automatically change every time you call this function.
     */
    bool TX_load(std::vector<uint8_t> raw);

    /**
     * @brief Get start pointer for TX. should transmit exactly RMT_TX_length items.
     *
     * @return rmt_item32_t* the start pointer.
     */
    rmt_item32_t *Get_TX_pointer();
};

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

/**
 * @brief A class for storing and processing data fragments received by RMT RX.
 */
class RX_data_fragments_pool
{
public:
    /**
     * @brief Pool of data fragments.
     */
    uint8_t pool[detail::Msg_memory_size] = {0};

    /**
     * @brief When each receiver received its first message from this robot.
     */
    uint64_t first_message_time[RMT_RX_CHANNEL_COUNT] = {0};

    /**
     * @brief When the last message in this pool is received.
     */
    uint64_t last_message_time = 0;

    /**
     * @brief Add element to the pool, do sanity checks, reset the pool when necessary.
     *
     * @param data Input data.
     * @param receiver receviers' id. from LSB to MSB, 0 to RMT_RX_CHANNEL_COUNT-1 bits, if the corresponding bit is 1 then means this receiver received the data as well.
     * @param time Time the data arrives.
     * @return bool Whether the whole data stream has been completed.
     */
    bool Add_element(Trans_info info);

    /**
     * @brief check if time data is valid.
     *
     * @return A boolean value indicating if all timing is ready. No sanity checks though
     */
    bool Time_valid_q();

    /**
     * @brief check if data is valid. Just here for compatability / unification of interface.
     *
     * @return A boolean value indicating if all data is ready.
     */
    bool Data_valid_q();

    /**
     * @brief Get the usable size of pool
     *
     * @return uint32_t Msg_max * Msg_content_bytes.
     */
    uint32_t size();

    /**
     * @brief Get the Msg_max object
     *
     * @return uint32_t Msg_max that is the max number of messages.
     *         Msg_max * Msg_content_bytes is the actual size of the pool.
     */
    uint32_t get_Msg_max();

private:
    /**
     * @brief How many messages should there be in this pool, indicated by the header message.
     */
    uint32_t Msg_max = 0;

    /**
     * @brief Number of messages in this pool. excluding header message!
     */
    uint32_t Msg_count = 0;

    /**
     * @brief Pool filling status.
     */
    uint64_t filling_status = 0;

    /**
     * @brief Which of the first_message_time list is filled. If time_ready_flag == (1<<RMT_RX_CHANNEL_COUNT)-1, then timing data is ready.
     */
    uint32_t time_ready_status = 0;

    /**
     * @brief Initial bit of messages in this pool.
     */
    uint32_t Msg_ID_init = 0;

    /**
     * @brief CRC data.
     */
    uint8_t CRC = 0;

    /**
     * @brief Data valid indicator.
     */
    bool data_valid_flag = false;

    /**
     * @brief reset the whole pool. Then setup the first data.
     *
     * @param data Input data.
     * @param receiver recevier's id. from 0 to RMT_RX_CHANNEL_COUNT-1
     * @param time Time the data arrives.
     */
    void reset_pool(Trans_info info);

    /**
     * @brief Reset the whole pool to its initial state.
     *        used by RMT_RX_prep when a pool is obsolete.
     */
    void reset_pool();

    // RMT_RX_prep is its friend and can acess Reset_all
    friend class RMT_RX_prep;
};

/**
 * @brief RMT RX preperation class.
 *
 */
class RMT_RX_prep
{
public:
    /**
     * @brief All pools storing data fragments sent by different robots.
     */
    RX_data_fragments_pool pools[detail::Max_robots_simultaneous] = {};

    /**
     * @brief Parse rmt item to usable data and add it to data pool for additional processing.
     *
     * @param pointer Pointer to raw rmt item storage.
     * @return true The corresponding pool has been completed!
     * @return false More data is needed.
     */
    bool Parse_RX(Trans_info info);

    /**
     * @brief Get the pointer to a single pool with certain ID. if it's not there, then return NULL
     *
     * @param robot_id The ID of the robot you want to know about
     *
     * @return a pointer to the corresponding RX_data_fragments_pool object, or NULL.
     */
    RX_data_fragments_pool *Get_pool(uint32_t robot_id);

    /**
     * @brief Count the number of active neighbors
     *
     * @return uint32_t the number of active neighbors.
     */
    uint32_t Count_neighbors();

    /**
     * @brief Write the IDs of active neighbors
     *
     * @return std::vector<uint32_t> neighbors' ID.
     */
    std::vector<uint32_t> Get_neighbors_ID();

    /**
     * @brief Delete obsolete pools.
     */
    void Delete_obsolete();

private:
    /**
     * @brief List of robot IDs.
     *
     * @note For simplicity, here we will assume robots all have id>0, so id==0 means there's nothing.
     *       We can always change that later, but for now, it's a neat trick.
     */
    uint32_t robot_id_list[detail::Max_robots_simultaneous] = {0};
};

class RMT_RX_TX final
{
public:
    /**
     * @brief initialization of RMT peripheral
     *
     * @return bool indicating whether the initialization is successful
     */
    static bool RMT_Init();

    /**
     * @brief TX timer interrupt handler
     */
    static void IRAM_ATTR RMT_TX_trigger();

    /**
     * @brief RX interrupt handler
     */
    static void IRAM_ATTR RMT_RX_ISR_handler(void *arg);

    /**
     * @brief RX trigger timer pointer
     */
    static hw_timer_t *RMT_TX_trigger_timer;

    /**
     * @brief RX pointer
     */
    static RMT_RX_prep *RX_prep;

    /**
     * @brief TX pointer
     */
    static RMT_TX_prep *TX_prep;

    /**
     * @brief This is a utility class, so there shouldn't be a constructor.
     *        Functions and values should be called directly via RMT_RX_TX::func() or RMT_RX_TX::val
     */
    RMT_RX_TX() = delete;

private:
    /**
     * @brief nothing but a handler used to clean up the ISR later
     */
    static rmt_isr_handle_t xHandler;
};

#endif