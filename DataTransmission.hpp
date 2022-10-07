// transmission of RMT signal
#ifndef _DATATRANSMISSION_HPP_
#define _DATATRANSMISSION_HPP_
#pragma once

#include "Arduino.h"
#include "Utilities\Circbuffer.hpp"
#include "RobotDefs.hpp"
#include "RMTMessageDefs.hpp"
#include "driver/rmt.h"
#include <vector>

namespace IR
{
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
        uint8_t data_pool[detail::Msg_memory_size] = {0};

        /**
         * @brief When each receiver received its first message from this robot.
         */
        uint64_t first_message_time[RMT_RX_CHANNEL_COUNT] = {0};

        /**
         * @brief When was the the last first_message_time.
         *
         * @note this time should be cauculated based on 1~3 timing in first_message_time being replaced in this update.
         *       but the exact method of calculation depends on the number of RX channels.
         */
        uint64_t prev_first_message_time = 0;

        /**
         * @brief When the last message in this pool is received.
         *
         * @note note the difference between last_RX_time and prev_first_message_time.
         *       last_RX_time records the last time ANY message is received,
         *       and is used to determine whether the incoming message should be considered as
         *       first message in this round or completely new message from the same robot.
         *       It's also used to determine whether the pool is obsolete and should be deleted.
         *       While prev_first_message_time records the time this robot first see this other robot in the previous round of rotation,
         *       and is used to determine the rotation speed of robot.
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

}

#endif