// Encoding & decoding of RMT signal
#ifndef _IRRX_HPP_
#define _IRRX_HPP_
#pragma once

#include "Arduino.h"
#include "Utilities/Circbuffer.hpp"
#include "RobotDefs.hpp"
#include "RMTMessageDefs.hpp"
#include "driver/rmt.h"
#include <array>
#include <vector>

namespace IR
{
    using namespace detail;

    namespace RX
    {
        // constants
        /**
         * @brief timing data's valid time
         *
         * @note A proper value for Timing_expire_time should be the time it takes for the robot to spin 1 rounds.
         */
        constexpr uint64_t Timing_expire_time = 60000;

        /**
         * @brief data's valid time
         * @note Data_expire_time should be larger than Timing_expire_time.
         *       A proper value for Data_expire_time should be 1.5 times the minimum span between two consecutive TX_load(...)
         *       So that it's not too short, but still can prevent msg_ID_init from duplication.
         */
        constexpr uint64_t Data_expire_time = 1000000;

        /**
         * @brief how many messages are allowed to stay in the buffer
         */
        constexpr uint32_t Raw_msg_buffer_size = 200;

        /**
         * @brief how frequently we read data from buffer and process it (in ms)
         *
         * @note because we have low duty cycle, we can rest for a while and let other tasks work...
         *       the signal's period is 100us, and there's seldom >3 emitters
         *       so in 5ms, there's at most 150 messages we need to process.
         */
        constexpr uint32_t Msg_process_period = 5;

        /**
         * @brief maximum number of robots that can communicate with a single robot in the same period of time.
         */
        constexpr uint32_t Max_robots_simultaneous = 8;

        /**
         * @brief maximum memory size for data.
         */
        constexpr uint32_t Msg_memory_size = ((1 << (Msg_ID_bits - 1)) - 1) * Msg_content_bytes;

        /**
         * @brief how many messages will we save per robot per type
         */
        constexpr uint32_t msg_buffer_history_size = 5;

        /**
         * @brief how many messages will we save in the recent message buffer
         */
        constexpr uint32_t recent_msg_buffer_history_size = 20;

        /**
         * @brief how many messages will we save in the timing message buffer
         */
        constexpr uint32_t timing_buffer_history_size = 3 * Max_robots_simultaneous;

        // struct definitions
        /**
         * @brief a general class for storing parsed messages
         *        it has subclass Parsed_msg_single, Parsed_msg_multiple.
         *        it is used by msg_buffer and recent_msg_buffer.
         *        you can get the validity of msg, length of msg, the starting pointer for msg, and last time its received.
         *        but except for these, there might be other relavent information inside the data structure,
         *        those are for me to construct it, not for you to read directly!
         */
        class Parsed_msg
        {
        public:
            /**
             * @brief last time this message is received.
             */
            uint64_t last_reception_time;
            /**
             * @brief get how many elements are there in the content
             *
             * @return uint32_t number of uint16_t, *2 is the number of bytes
             */
            virtual inline uint32_t Get_content_length() = 0;
            /**
             * @brief get the starting pointer of the content
             *
             * @return uint16_t* starting pointer
             */
            virtual inline uint16_t *Get_content_pointer() = 0;
            /**
             * @brief whether the content is valid right now
             *
             * @return bool validity
             */
            virtual inline bool Content_valid_Q() = 0;

            friend void Preprocess();

        protected:
            /**
             * @brief Initial bit of messages in this pool.
             */
            uint32_t msg_ID_init = 0;
        };

        class Parsed_msg_single : public Parsed_msg
        {
        public:
            uint16_t content;
            inline uint32_t Get_content_length()
            {
                return 1;
            }
            inline uint16_t *Get_content_pointer()
            {
                return &content;
            }
            inline bool Content_valid_Q()
            {
                return 1;
            }
            friend void Preprocess();
        };

        class Parsed_msg_multiple : public Parsed_msg
        {
        public:
            uint16_t content[Msg_memory_size];
            inline uint32_t Get_content_length()
            {
                return msg_ID_max;
            }
            inline uint16_t *Get_content_pointer()
            {
                return content;
            }
            inline bool Content_valid_Q()
            {
                return content_valid_flag;
            }
            friend void Preprocess();

        private:
            /**
             * @brief Add element to the pool, do sanity checks, reset the pool when necessary.
             *
             * @param info complete information of transmission
             * @return uint32_t there are two useful bits, bit 0 and bit 1.
             *         bit 0 indicates whether we should open a new structure for the new inquiry. 1 for yes.
             *         bit 1 indicates whether we just finished a complete message.
             * 
             * @note to call this function's or Add_header is determined by the Preprocess
             */
            uint32_t Add_content(Trans_info info);

            /**
             * @brief Add header to the pool, do sanity checks, reset the pool when necessary.
             *
             * @param info complete information of transmission
             * @return uint32_t there are two useful bits, bit 0 and bit 1.
             *         bit 0 indicates whether we should open a new structure for the new inquiry. 1 for yes.
             *         bit 1 indicates whether we just finished a complete message.
             */
            uint32_t Add_header(Trans_info info);

            /**
             * @brief How many messages should there be in this pool, indicated by the header message, excluding header message.
             */
            uint32_t msg_ID_max = 0;

            /**
             * @brief Number of messages in this pool. including header message! so when full, we have msg_count = msg_ID_max + 1.
             */
            uint32_t msg_count = 0;

            /**
             * @brief Pool filling status. bit i in filling status indicates whether message with msg_id i is filled.
             * 
             * @note header is bit 0. first message is bit 1.
             */
            uint32_t filling_status = 0;

            /**
             * @brief CRC data.
             */
            uint32_t CRC = 0;

            /**
             * @brief Data valid indicator.
             */
            bool content_valid_flag = false;
        };

        /**
         * @brief Parsed_msg_completed can be shorter because it's always finished so we can get rid of some metadata.
         */
        typedef struct
        {
            /**
             * @brief type of msg, determines what is inside the union, msg_single or msg_multiple
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
             * @brief byte channel represents whether timing for channel i is sent by top/bottom emitter.
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
         */
        void Init();

        /**
         * @brief RX ISR that handles input RMT transmissions and store them into a buffer.
         */
        void IRAM_ATTR RX_ISR();

        /**
         * @brief get raw uint32_t data from buffer(data comes from RX_ISR) and organize and store all parsed data packet.
         *        this function shall be called at ~100Hz rate (just not very frequently).
         *
         * @note I know that this function is too long, but it's still pretty structured and clear so I am not gonna split it into multiple functions. 
         * @note This task should not be preempted by other user tasks!!! or else read and write of buffer might get messy.
         */
        void Preprocess();

        /**
         * @brief get current absolute position
         */
        void Calc_position_task(void *pvParameters);

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
        inline uint32_t Get_io_flag();

        /**
         * @brief get the age th latest message by id, type.
         *
         * @param robot_ID message from which robot?
         * @param msg_type which type of message?
         * @param age by default is 0 -> latest, 1 -> next latest, etc. max is msg_buffer_history_size - 2
         * @return Parsed_msg* the pointer to a Parsed_msg object. you can use relavent functions to access its content.
         *
         * @note this will return the rank'th **completed** message! 
         * @note please check if data exists by checking the timing information of the returned result. if it's 0 then no message.
         */
        Parsed_msg_completed Get_latest_msg_by_bot(uint32_t robot_ID, uint32_t msg_type, uint32_t age = 0);

        /**
         * @brief get the age th latest message
         *
         * @param msg_type which type of message?
         * @param age by default is 0 -> latest, 1 -> next latest, etc. max is recent_msg_buffer_history_size - 1
         * @return Parsed_msg_completed corresponding data
         * @note please check if data exists by checking the timing information of the returned result. if it's 0 then no message.
         */
        Parsed_msg_completed Get_latest_msg_by_type(uint32_t msg_type, uint32_t age = 0);

        /**
         * @brief get a queue of certain type of message which you can save as a stream and pop() as you wish without worrying about influencing other tasks.
         * 
         * @param msg_type which type of message?
         * @return Circbuffer_copycat<Parsed_msg_completed,recent_msg_buffer_history_size> the 'stream' circbuffer.
         */
        inline Circbuffer_copycat<Parsed_msg_completed,recent_msg_buffer_history_size> Get_msg_buffer_by_type(uint32_t msg_type);

        // /**
        //  * @brief get the age th latest timing data
        //  *
        //  * @param age by default is 0 -> latest, 1 -> next latest, etc. max is timing_buffer_history_size - 1
        //  * @return Msg_timing_t corresponding data
        //  * 
        //  * @note Don't know why anyone would want this...
        //  */
        // Msg_timing_t Get_timing_data(uint32_t age = 0);

        /**
         * @brief copy all timing data into an array, **from newest to oldest**.
         *
         * @param start starting pointer of array.
         * @return uint32_t length of Msg_timing_t array
         */
        uint32_t Copy_timing_data(Msg_timing_t *start);
    }
}

#endif