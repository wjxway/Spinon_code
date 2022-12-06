/**
 * @file IrRX.hpp
 * @brief receive and process data & timing interface.
 */
#ifndef _IRRX_HPP_
#define _IRRX_HPP_

#include "Arduino.h"
#include "../Utilities/Circbuffer.hpp"
#include "../RobotDefs.hpp"
#include "RMTMessageDefs.hpp"
#include <array>

/**
 * @brief if we should turn on LED based on message received?
 */
// #define MSG_LED_ON 1

namespace IR
{
    using namespace detail;

    namespace RX
    {
        // constants
        /**
         * @brief how frequently will preprocess task be triggered, in ms.
         */
        constexpr uint32_t Preprocess_trigger_period = 30;

        /**
         * @brief priority of Preprocess task, note that all other user tasks
         * based on RX should have lower priority than this!
         */
        constexpr uint32_t Preprocess_task_priority = 10;

        /**
         * @brief timing data's valid time
         *
         * @note A proper value for Timing_expire_time should be the time it
         * takes for the robot to spin ~0.2 rounds, take lower limit, but should
         * be larger than a few times of RMT_TX_trigger_period_max.
         */
        constexpr uint64_t Timing_expire_time = 10000;

        /**
         * @brief data's valid time
         * @note Data_expire_time should be larger than Timing_expire_time.
         * A proper value for Data_expire_time should be 1.5 times the minimum
         * span between two consecutive TX_load(...) So that it's not too short,
         * but still can prevent msg_ID_init from duplication.
         */
        constexpr uint64_t Data_expire_time = 1000000;

        /**
         * @brief how many messages are allowed to stay in the buffer
         */
        constexpr uint32_t Raw_msg_buffer_size = 200;

        /**
         * @brief maximum number of robots that can communicate with a single
         * robot in the same period of time.
         */
        constexpr uint32_t Max_robots_simultaneous = 8;

        /**
         * @brief maximum memory size for data.
         */
        constexpr uint32_t Msg_memory_size = ((1 << (Msg_ID_bits - 1)) - 1);

        /**
         * @brief how many messages will we save per robot per type
         */
        constexpr uint32_t Msg_buffer_history_size = 5;

        /**
         * @brief how many messages will we save in the recent message buffer
         */
        constexpr uint32_t Recent_msg_buffer_history_size = 20;

        /**
         * @brief how many messages will we save in the timing message buffer
         */
        constexpr uint32_t Timing_buffer_history_size = 3 * Max_robots_simultaneous;

        /**
         * @brief Parsed_msg_completed can be shorter because it's always
         * finished so we can get rid of some metadata.
         */
        typedef struct
        {
            /**
             * @brief type of msg, determines what is inside the union,
             * msg_single or msg_multiple
             */
            uint32_t msg_type;

            /**
             * @brief robot's ID
             */
            uint32_t robot_ID;

            /**
             * @brief time of finish reception
             */
            uint64_t finish_reception_time;

            /**
             * @brief length of message in uint16_t
             */
            uint32_t content_length;

            /**
             * @brief content of message
             */
            uint16_t content[Msg_memory_size];
        } Parsed_msg_completed;

        /**
         * @brief timing structure
         */
        typedef struct
        {
            uint32_t robot_ID;
            /**
             * @brief time for different channels
             */
            uint64_t time_arr[RMT_RX_CHANNEL_COUNT];
            /**
             * @brief bit i represents whether message received is sent by
             * top/bottom emitter.
             *        0 -> top, 1 -> bottom
             * Note that we only care about which emitter we are getting message
             * from if the message is received by the center receiver, because
             * that's the only receiver related to elevation sensing.
             */
            uint32_t emitter_pos = 0;
            /**
             * @brief helper for quickly identifying the occupation state of
             * time_arr. if bit i=1, then time_arr[i] is occupied undefined
             */
            uint32_t receiver = 0;
            /**
             * @brief byte channel represents whether timing for channel i is valid.
             */
            uint32_t timing_valid_Q = 0;
        } Msg_timing_t;

        /**
         * @brief initialize RX routine, including:
         *        1. initialize RX RMT channels
         *        2. initialize RX ISR
         *        3. initialize preprocess data structures, routine, and timer
         *
         * @return whether init is successful
         */
        bool Init();

        /**
         * @brief get io_flag, can be used to implement one's own read routine.
         *        a standard way of using it is:
         *          uint32_t curr_flag;
         *          do
         *          {
         *              curr_flag = io_flag;
         *              // read something
         *          }
         *          // repeat if write task preempted this task
         *          while (io_flag != curr_flag);
         */
        uint32_t Get_io_flag();

        /**
         * @brief get the age th latest message by id, type.
         *
         * @param robot_ID message from which robot?
         * @param msg_type which type of message?
         * @param age by default is 0 -> latest, 1 -> next latest, etc. max is
         * msg_buffer_history_size - 2
         * @return Parsed_msg_completed corresponding data
         *
         * @note this will return the rank'th **completed** message!
         * @note please check if data exists by checking the length of data, if
         * 0 then there's no returned message.
         */
        Parsed_msg_completed Get_latest_msg_by_bot(const uint32_t robot_ID, const uint32_t msg_type, const uint32_t age = 0);

        /**
         * @brief get the age th latest message
         *
         * @param msg_type which type of message?
         * @param age by default is 0 -> latest, 1 -> next latest, etc. max is
         * recent_msg_buffer_history_size - 1
         * @return Parsed_msg_completed corresponding data
         * @note please check if data exists by checking the length of data, if
         * 0 then there's no returned message.
         */
        Parsed_msg_completed Get_latest_msg_by_type(const uint32_t msg_type, const uint32_t age = 0);

        /**
         * @brief get a queue of certain type of message which you can save as a
         * stream and pop() as you wish without worrying about influencing other
         * tasks.
         *
         * @param msg_type which type of message?
         * @return
         * Circbuffer_copycat<Parsed_msg_completed,recent_msg_buffer_history_size>
         * the 'stream' circbuffer.
         */
        Circbuffer_copycat<Parsed_msg_completed, Recent_msg_buffer_history_size> Get_msg_buffer_by_type(const uint32_t msg_type);

        // /**
        //  * @brief get the age th latest timing data
        //  *
        //  * @param age by default is 0 -> latest, 1 -> next latest, etc.
        //  * max is Timing_buffer_history_size - 1
        //  * @return Msg_timing_t corresponding data
        //  *
        //  * @note Don't know why anyone would want this...
        //  */
        // Msg_timing_t Get_timing_data(uint32_t age = 0);

        /**
         * @brief copy all timing data into an array, **from newest to oldest**.
         *
         * @param start starting pointer of array.
         * @param history_time get only the robots that has been seen within
         * history_time us. when is 0, then access all. Note that here we use
         * only time_arr[0] for timing.
         * @return uint32_t length of Msg_timing_t array
         */
        uint32_t Get_timing_data(Msg_timing_t *const start, const uint64_t history_time=0);

        /**
         * @brief get all neighboring robot's ID
         *
         * @param start starting pointer with type uint32_t, when == nullptr,
         * then only return the number of robot.
         * @param history_time get only the robots that has been seen within
         * history_time us. when is 0, then access all.
         * @return uint32_t how many robots are there?
         */
        uint32_t Get_neighboring_robots_ID(uint32_t *const start, const uint64_t history_time = 0);

        /**
         * @brief get the last time any message is received, regardless of correctness.
         * 
         * @return last RX time
         */
        uint64_t Get_last_RX_time();
    }
}

#endif