// Encoding & decoding of RMT signal
#ifndef _DATATRANSMISSION_
#define _DATATRANSMISSION_

#pragma once
#include "Arduino.h"
#include "RMTCoding.h"
#include <vector>

// // print debug content or not
// #define _DEBUG_PRINT_ 1

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
constexpr uint64_t Timing_expire_time = 500000;

/**
 * @brief maximum number of robots that can communicate with a single robot in the same period of time.
 */
constexpr uint32_t Max_robots_simultaneous = 20;

/**
 * @brief Fragments pool itself's decay time. If nothing is added to the pool for this amount of time (in us),
 *        then this robot has probably leaved the communication range and this pool should be re-purposed.
 */
constexpr uint64_t Data_expire_time = 2000000;

/**
 * @brief number of receivers
 */
#define N_RECEIVERS 2

/**
 * @brief general structure of messages with <=32 bits.
 */
typedef union
{
    struct
    {
        uint32_t Robot_ID : Robot_ID_bits;
        uint32_t Msg_ID_init : 1;
        uint32_t Msg_ID : Msg_ID_bits - 1;
        uint32_t content : Msg_content_bits;
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
        uint32_t Robot_ID : Robot_ID_bits;
        uint32_t Msg_ID_init : 1;
        uint32_t Msg_ID : Msg_ID_bits - 1;
        uint32_t Msg_ID_len : Msg_ID_bits - 1; // Msg_ID will be from 0 (the header) to Msg_ID_len.
        uint32_t CRC : Msg_content_bits - Msg_ID_bits + 1;
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
 * @brief convert a small uint8_t array to uint32_t.
 *
 * @param pointer start pointer for data.
 * @return uint32_t content field.
 */
inline uint32_t Construct_content(uint8_t *pointer);

/**
 * @brief convert a uint32_t to a small uint8_t array.
 *
 * @param data input uint32_t data.
 * @param pointer fill data starting from this pointer.
 */
inline void Deconstruct_content(uint32_t data, uint8_t *pointer);

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
    // 1 for ready, 0 for not ready. skip an emission cycle when output is not ready.
    // use spin-lock mechanism to prevent simultaneous access of output.
    bool output_ready_flag = 1;

    /**
     * @brief Construct a new RMT_TX_prep object.
     *
     * @param rid Robot's ID, please make sure it's smaller than 2^Robot_ID_bits, I will not check.
     */
    RMT_TX_prep(uint32_t rid) : robot_ID(rid) {}

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
     *
     * @note Should check output_ready_flag before calling this function.
     *       output_pos will automatically change every time you call this function.
     */
    rmt_item32_t *Get_TX_pointer();
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
    uint8_t pool[Msg_memory_size] = {0};

    /**
     * @brief When each receiver received its first message from this robot.
     */
    uint64_t first_message_time[N_RECEIVERS] = {0};

    /**
     * @brief When the last message in this pool is received.
     */
    uint64_t last_message_time = 0;

    /**
     * @brief Add element to the pool, do sanity checks, reset the pool when necessary.
     *
     * @param data Input data.
     * @param receiver receviers' id. from LSB to MSB, 0 to N_RECEIVERS-1 bits, if the corresponding bit is 1 then means this receiver received the data as well.
     * @param time Time the data arrives.
     * @return true The whole data stream has been completed.
     * @return false Still need more data.
     */
    bool Add_element(uint32_t data, uint32_t receiver, uint64_t time);

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
     * @return uint32_t Msg_max which is the max number of messages.
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
     * @brief Which of the first_message_time list is filled. If time_ready_flag == (1<<N_RECEIVERS)-1, then timing data is ready.
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
     * @param receiver recevier's id. from 0 to N_RECEIVERS-1
     * @param time Time the data arrives.
     */
    void reset_pool(uint32_t data, uint32_t receiver, uint64_t time);

    /**
     * @brief Reset the whole pool to its initial state.
     *        used by RMT_RX_prep when a pool is obsolete.
     */
    void Reset_all();

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
     *        Stored in a unordered map for faster fetch based on Robot_ID
     */
    RX_data_fragments_pool pools[Max_robots_simultaneous] = {};

    /**
     * @brief list of robot ids.
     * 
     * @note For simplicity, here we will assume robots all have id>0, so id==0 means there's nothing.
     *       We can always change that later, but for now, it's a neat trick.
     */
    uint32_t robot_id_list[Max_robots_simultaneous]={0};

    /**
     * @brief Try to fetch the data coming from a single robot.
     * 
     * @param robot_id robot's id.
     * @param data_ptr will write the start pointer of data array, data_ptr, to this address
     * @param data_len will write data's length to this address
     * @return bool which represent whether the robot exists and the operation is successful.
     */
    bool Get_data(uint32_t robot_id, uint8_t** data_ptr, uint32_t* data_len);

    /**
     * @brief Try to fetch the timing data coming from a single robot.
     * 
     * @param robot_id robot's id.
     * @param data_ptr will write the start pointer of timing array, data_ptr, to this address
     * @return bool which represent whether the robot exists and the operation is successful.
     */
    bool Get_time(uint32_t robot_id, uint8_t** time_ptr);

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
     * @brief Parse rmt item to usable data and add it to data pool for additional processing.
     *
     * @param pointer Pointer to raw rmt item storage.
     * @return true Successful!
     * @return false Data invalid!
     */
    bool Parse_RX(volatile rmt_item32_t *pointer);

    /**
     * @brief Delete obsolete pools.
     */
    void Delete_obsolete();
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
    static void IRAM_ATTR rmt_isr_handler(void *arg);

    /**
     * @brief RX
     */
    static RMT_RX_prep RX_prep;

    /**
     * @brief TX
     */
    static RMT_TX_prep TX_prep;

    // This is a utility class, so there shouldn't be a constructor.
    // Functions and values should be called directly via RMT_RX_TX::func() or RMT_RX_TX::val
    RMT_RX_TX() = delete;
};

#endif