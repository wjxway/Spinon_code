// Encoding & decoding of RMT signal
#ifndef _DATATRANSMISSION_HPP_
#define _DATATRANSMISSION_HPP_
#pragma once

#include "Arduino.h"
#include "RMTCoding.hpp"
#include "Circbuffer.hpp"
#include <vector>

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
    extern const uint32_t RMT_data_length;

    /**
     * @brief RMT TX sequnce length.
     *
     * @note RMT_TX_length = RMT_data_length + 2 because there will be a '0' header and a ending block
     */
    const uint32_t RMT_TX_length = RMT_data_length + 2;

    /**
     * @brief Fragments pool's timing data's decay time. If nothing is added to the pool for this amount of time (in us),
     *        then the timing info should be discarded.
     *
     * @note A proper value for Timing_expire_time should be the time it takes for the robot to spin 0.8 rounds.
     */
    constexpr uint64_t Timing_expire_time = 60000;

    /**
     * @brief maximum number of robots that can communicate with a single robot in the same period of time.
     */
    constexpr uint32_t Max_robots_simultaneous = 5;

    /**
     * @brief Fragments pool itself's decay time. If nothing is added to the pool for this amount of time (in us),
     *        then this robot has probably leaved the communication range and this pool should be re-purposed.
     *
     * @note Data_expire_time should be larger than Timing_expire_time.
     *       A proper value for Data_expire_time should be 1.5 times the minimum span between two consecutive TX_load(...)
     *       So that it's not too short, but still can prevent msg_ID_init from duplication.
     */
    constexpr uint64_t Data_expire_time = 2000000;

    /**
     * @brief RMT TX channel num
     */
    constexpr rmt_channel_t RMT_TX_channel_1 = RMT_CHANNEL_0;

    /**
     * @brief RMT TX channel 2 num
     */
    constexpr rmt_channel_t RMT_TX_channel_2 = RMT_CHANNEL_1;

    /**
     * @brief First RMT RX channel num (Highest priority)
     */
    constexpr rmt_channel_t RMT_RX_channel_1 = RMT_CHANNEL_2;
#if RMT_RX_CHANNEL_COUNT >= 2
    /**
     * @brief Second RMT RX channel num
     */
    constexpr rmt_channel_t RMT_RX_channel_2 = RMT_CHANNEL_4;
#endif
#if RMT_RX_CHANNEL_COUNT == 3
    /**
     * @brief Third RMT RX channel num (Lowest priority)
     */
    constexpr rmt_channel_t RMT_RX_channel_3 = RMT_CHANNEL_6;
#endif

    /**
     * @brief Timer channel used for RMT TX trigger
     */
    constexpr uint32_t RMT_TX_trigger_timer_channel = 3;

    /**
     * @brief RMT TX Trigger period in us
     */
    constexpr uint64_t RMT_TX_trigger_period = 50;

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

    /**
     * @brief how many messages are allowed to stay in the buffer
     */
    constexpr uint32_t Msg_buffer_max_size = 200;

    /**
     * @brief how frequently we read data from buffer and process it
     *
     * @note because we have low duty cycle, we can rest for a while and let other tasks work...
     *       the signal's period is 100us, and there's seldom >3 emitters
     *       so in 5ms, there's at most 150 messages we need to process.
     */
    constexpr uint32_t Msg_process_period = 5;
}

/**
 * @brief RMT TX preperation class. Used to process signal emissions from a single robot ID.
 *
 * @note This class is thread-safe for output data acessing
 *       This class should only be responsible for preperation of data
 *       but not hardware level interaction!
 *       This seperation allows us to implement collision avoidance easier in the future.
 */
class RMT_TX_prep
{
public:
    /**
     * @brief robot's ID
     */
    uint32_t robot_ID = 0;
    /**
     * @brief msg id initial
     */
    uint32_t msg_ID_init = 0;

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
     * @param ticks_delay How many ticks of delay before the data, for time sync.
     * @param ticks_final How many ticks for the final pulse in the data.
     *
     * @return true TX load successful!
     * @return false TX load failed!
     *
     * @note msg_ID_init will automatically change every time you call this function.
     */
    bool TX_load(std::vector<uint8_t> raw, uint32_t ticks_delay, uint32_t ticks_final);

    /**
     * @brief Get start pointer for TX. should transmit exactly RMT_TX_length items.
     *        After calling this function and finished with the data transfer,
     *        you should explicitly and externally reset the reader lock using Reset_reader_lock()
     *
     * @return rmt_item32_t* the start pointer.
     */
    rmt_item32_t *Get_TX_pointer();

    /**
     * @brief reset the reader's lock once so data could be updated as normal.
     *        This is just a semaphore theme, not a MUTEX, so be careful not to call it twice!
     */
    void Reset_reader_lock();

private:
    /**
     * @brief output rmt_item32_t data vector
     */
    std::vector<rmt_item32_t> output;
    /**
     * @brief output data position pointer
     */
    uint32_t output_pos = 0;

    // the implementation of the read-write lock
    // use spinlocks and flags to prevent added timing associated with FreeRTOS
    // though less obvious and more unsafe to some extent.

    /**
     * @brief output ready flag
     *
     * @note 0 for all set, 1 for updating the header, 2 for updating the content.
     */
    volatile uint32_t edit_state_flag = 0;
    /**
     * @brief number of readers
     *
     * @note should lock write operation when there's >=1 reader
     */
    volatile uint32_t reader_count_flag = 0;
};

/**
 * @brief A class for storing and processing data fragments received by RMT RX.
 */
class RX_data_fragments_pool
{
public:
    /**
     * @brief corresponding robot's id
     */
    uint32_t robot_id = 0;

    /**
     * @brief Pool of data fragments.
     */
    uint8_t pool[detail::Msg_memory_size] = {0};

    /**
     * @brief When each receiver received its first message from this robot.
     *
     * @note first_message_time[RMT_RX_CHANNEL_COUNT] indicates whether the message is sent by the top emitter or the bottom one.
     */
    uint64_t first_message_time[RMT_RX_CHANNEL_COUNT + 1] = {};

    /**
     * @brief When the last message in this pool is received.
     */
    uint64_t last_RX_time = 0;

    /**
     * @brief Add element to the pool, do sanity checks, reset the pool when necessary.
     *
     * @param info complete information of transmission
     * @return bool Whether the whole data stream has been completed.
     */
    bool Add_element(detail::Trans_info info);

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
    uint32_t msg_ID_max = 0;

    /**
     * @brief Number of messages in this pool. excluding header message!
     */
    uint32_t msg_count = 0;

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
    uint32_t msg_ID_init = 0;

    /**
     * @brief CRC data.
     */
    uint32_t CRC = 0;

    /**
     * @brief Data valid indicator.
     */
    bool data_valid_flag = false;

    /**
     * @brief reset the data-related contents in pool. Then setup the first data.
     *
     * @param info complete information of transmission
     */
    void Reset_pool_data(detail::Trans_info info);

    /**
     * @brief Reset the whole pool to its initial state.
     *        used by RMT_RX_prep when a pool is obsolete.
     */
    void Reset_pool_all();

    // RMT_RX_prep is its friend and can acess Reset_all
    friend class RMT_RX_prep;
};

/**
 * @brief RMT RX preperation class.
 *
 * @note this class should only be responsible for preperation of data
 *       but not hardware level interaction!
 *       This seperation allows us to implement collision avoidance easier in the future.
 */
class RMT_RX_prep
{
public:
    /**
     * @brief message buffer
     *
     * @note messages received is first decoded by interrupt on core 0
     *       and then stored in this buffer
     *       before it could be put in place by core 1. (ECC / place into corresponding pool / other actions)
     */
    Circbuffer<detail::Trans_info> msg_buffer{detail::Msg_buffer_max_size};

    /**
     * @brief All pools storing data fragments sent by different robots.
     */
    RX_data_fragments_pool pools[detail::Max_robots_simultaneous] = {};

    /**
     * @brief Parse rmt item to usable data and add it to data pool for additional processing.
     *
     * @param info complete information of transmission.
     * @return true The corresponding pool has been completed!
     * @return false More data is needed.
     */
    bool Parse_RX(detail::Trans_info info);

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

/**
 * @brief a FreeRTOS queue for storing timing data
 */
extern xQueueHandle RMT_RX_time_queue;

/**
 * @brief a FreeRTOS queue for storing real data
 */
extern xQueueHandle RMT_RX_data_queue;

class RMT_RX_TX final
{
public:
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
    static RMT_TX_prep *TX_prep_1;

    /**
     * @brief TX pointer
     */
    static RMT_TX_prep *TX_prep_2;

    /**
     * @brief last time when a message is received
     */
    static volatile uint64_t last_RX_time;

    /**
     * @brief last time when a message is sent
     */
    static volatile uint64_t last_TX_time;

    /**
     * @brief Initialization of RMT peripheral, should be called FIRST! Will automatically start RX, but not TX.
     *
     * @return bool indicating whether the initialization is successful
     *
     * @note You can start TX with RMT_TX_resume(), after loading your own data (default data is {0,0}).
     */
    static bool RMT_init();

    /**
     * @brief TX timer interrupt handler
     */
    static void IRAM_ATTR RMT_TX_trigger();

    /**
     * @brief RX interrupt handler
     */
    static void IRAM_ATTR RMT_RX_ISR_handler(void *arg);

    /**
     * @brief RX processor
     *
     * @note this process will process data in the buffer, which is originally generated by ISR handler
     *       this process should run on core 1.
     */
    static void RX_process_task(void *parameters);

    /**
     * @brief Add some time to the current timer. Use when preventing interference with other robots.
     *
     * @param dt time in ticks to add
     */
    static bool RMT_TX_add_time(uint64_t dt);

    /**
     * @brief Resume RMT_TX, note that this won't re-setup the timer.
     */
    static bool RMT_TX_resume();

    /**
     * @brief Pause RMT_TX, note that this won't stop or free the timer.
     */
    static bool RMT_TX_pause();

    /**
     * @brief Resume RMT_TX, note that this won't re-setup the timer.
     */
    static bool RMT_RX_resume();

    /**
     * @brief Pause RMT_TX, note that this won't stop or free the timer.
     */
    static bool RMT_RX_pause();

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