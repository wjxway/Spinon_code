#include "IrRX.hpp"
#include "RMTCoding.hpp"
#include "RMTMessageDefs.hpp"
#include "Utilities/FastIO.hpp"
#include "Utilities/DebugDefs.hpp"
#include "Utilities/CRCCheck.hpp"
#include "Utilities/Circbuffer.hpp"
#include <array>
#include <vector>

namespace IR
{
    using namespace detail;

    namespace RX
    {
        // global variables (especially buffers)
        // in total we have 4 buffers (can add more in the future depending on need)
        // 1. raw_msg_buffer    : the only one that's populate by ISR, stores individual, unparsed messages. Read by preprocess task.
        //
        // 2.0 msg_buffer_dict  : dictionary for msg_buffer, msg_buffer_dict[robot_id] -> dict_id.
        // 2.1 msg_buffer       : stores data in the form of msg_buffer[dict_id][msg_type][n] gives the nth oldest message of msg_type sent by robot_id.
        //                        This buffer only provide information of content and last reception time, but not anything related to localization like emitter position or receiver position.
        // 3. recent_msg_buffer : stores the most recent message that finished reception.
        //                        This buffer only provide information of content and last reception time, but not anything related to localization like emitter position or receiver position.
        // 4. timing_buffer     : stores the most recent first message in this rotation from different robots.
        //                        This buffer only provide information of first message's timing and robot ID, but not data contents.
        //
        // I cannot think of any end user scenario where people need the raw data and cannot use a combination of buffer 2~4.
        //
        // To deal with concurrency, we require the priority of Preprocess task to always be higher than the user tasks.
        // then we know that write task can never be preempted by read task, but read task can be preempted by write task.
        // we also know that read is much faster than write, and write will occur at a fixed, non frequent interval.
        // Now, we add a io_flag and increment it evertime we write, and in read, do:
        //      do
        //      {
        //          curr_flag=io_flag;
        //          // read stuffs
        //      } while(curr_flag!=io_flag)
        // so if the read is preempted by write, we will read again.
        namespace
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
            constexpr uint32_t timing_buffer_history_size = 5 * Max_robots_simultaneous;

            // variable definitions
            /**
             * @brief time of last message reception
             */
            uint64_t last_RX_time = 0;

            /**
             * @brief a circular buffer that stores all incoming, unprocessed raw messages and their meta data.
             *        Once it's full, it will start to override the oldest data.
             */
            Circbuffer<Trans_info, Raw_msg_buffer_size> raw_msg_buffer;

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
                virtual uint32_t Get_content_length() = 0;
                /**
                 * @brief get the starting pointer of the content
                 *
                 * @return uint16_t* starting pointer
                 */
                virtual uint16_t *Get_content_pointer() = 0;
                /**
                 * @brief whether the content is valid right now
                 *
                 * @return bool validity
                 */
                virtual bool Content_valid_Q() = 0;

            private:
                /**
                 * @brief Initial bit of messages in this pool.
                 */
                uint32_t msg_ID_init = 0;
            };

            class Parsed_msg_single : public Parsed_msg
            {
            public:
                uint16_t content;
                uint32_t Get_content_length()
                {
                    return 1;
                }
                uint16_t *Get_content_pointer()
                {
                    return &content;
                }
                bool Content_valid_Q()
                {
                    return 1;
                }
            };

            class Parsed_msg_multiple : public Parsed_msg
            {
            public:
                uint16_t content[Msg_memory_size];
                uint32_t Get_content_length()
                {
                    return msg_ID_max;
                }
                uint16_t *Get_content_pointer()
                {
                    return content;
                }
                bool Content_valid_Q()
                {
                    return content_valid_flag;
                }

            private:
                /**
                 * @brief Add element to the pool, do sanity checks, reset the pool when necessary.
                 *
                 * @param info complete information of transmission
                 * @return bool Whether the whole data stream has been completed.
                 */
                bool Add_element(detail::Trans_info info);

                /**
                 * @brief How many messages should there be in this pool, indicated by the header message.
                 */
                uint32_t msg_ID_max = 0;

                /**
                 * @brief Number of messages in this pool. excluding header message!
                 */
                uint32_t msg_count = 0;

                /**
                 * @brief Pool filling status. bit i in filling status indicates whether message with msg_id i is filled.
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

            class Robot_RX
            {
            public:
                std::array<Circbuffer<Parsed_msg_single, msg_buffer_history_size>, Single_transmission_msg_type + 1> single_data;
                std::array<Circbuffer<Parsed_msg_multiple, msg_buffer_history_size>, Msg_type_max - Single_transmission_msg_type> multiple_dat;
                uint64_t last_reception_time;
            };

            /**
             * @brief index of all robots
             *        msg_buffer_dict[Robot_ID] gives you the position of Robot_ID in msg_buffer
             *        if it's -1, then Robot_ID do not exist in msg_buffer
             *        should be initialized all to -1
             */
            std::array<int32_t, 1 << Robot_ID_bits> msg_buffer_dict;

            /**
             * @brief message buffer that stores data in the form of msg_buffer[dict_id][msg_type][n] gives the nth oldest message of msg_type sent by robot_id.
             *        This buffer only provide information of content and last reception time, but not anything related to localization like emitter position or receiver position.
             */
            std::array<Robot_RX, Max_robots_simultaneous> msg_buffer;

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
                 * @brief length of message
                 */
                uint32_t content_length;

                /**
                 * @brief content of message
                 */
                uint16_t content[Msg_memory_size];
            } Parsed_msg_completed;

            /**
             * @brief recent_msg_buffer stores data that recently completed transmission.
             */
            Circbuffer<Parsed_msg_completed, recent_msg_buffer_history_size> recent_msg_buffer;

            /**
             * @brief
             */
            typedef struct
            {
                uint32_t robot_ID;
                uint64_t time_arr[RMT_RX_CHANNEL_COUNT];
                /**
                 * @brief byte channel represents whether timing for channel i is valid.
                 */
                uint32_t timing_valid_Q = 0;
            } Msg_timing_t;

            /**
             * @brief a buffer that stores timing information of
             *        'when the first message from each robot is received by each receiver in this revolution'.
             */
            Circbuffer<Msg_timing_t, timing_buffer_history_size> timing_buffer;

            /**
             * @brief flag that will increase when write. used to resolve concurrency (check notes at the beginning of this file).
             */
            uint32_t io_flag = 0;
        }

        void IRAM_ATTR RX_ISR()
        {
            // test start pulse
            // delayhigh100ns(TEST_PIN);
            // clrbit(TEST_PIN);

            // delay for a bit to make sure all RMT channels finished receiving
            // I assume a identical signal's delay won't vary by 100ns which is more than half the RMT pulse width.
            // It is highly unlikely that two valid yet different signal received within such a short time is different.
            // If each cycle is 250us, that's a 1/1000 chance that two valid yet different signal will collide,
            // and this still haven't take into consideration of geometric configuration.
            // delay100ns;
            delay100ns;
            delay100ns;

            // record time
            uint64_t rec_time;
            rec_time = esp_timer_get_time();

            // read RMT interrupt status.
            uint32_t intr_st = RMT.int_st.val;
            // interrupt state, with order modified so that its 0,1,2 bits represent interrupt state for ch0, ch1, ch2.
            uint32_t intr_st_1 = 0;

            // rmt item
            volatile rmt_item32_t *item_1, *item_2, *item_3;
            // RMT_RX_1, corresponding to RMT channel 2
            // This channel has the highest priority when >1 channels received the signal.
            // So make sure that this is the channel that received the signal later.
            // Because the channel receiving the signal later will have higher received intensity.
            if (intr_st & (1 << (1 + 3 * RMT_RX_channel_1)))
            {
                // change owner state, disable rx
                RMT.conf_ch[RMT_RX_channel_1].conf1.rx_en = 0;
                RMT.conf_ch[RMT_RX_channel_1].conf1.mem_owner = RMT_MEM_OWNER_TX;
                intr_st_1 += 1;

                // RMT storage pointer
                item_1 = RMTMEM.chan[RMT_RX_channel_1].data32;
            }
#if RMT_RX_CHANNEL_COUNT >= 2
            // RMT_RX_2, corresponding to RMT channel 4
            if (intr_st & (1 << (1 + 3 * RMT_RX_channel_2)))
            {
                // change owner state, disable rx
                RMT.conf_ch[RMT_RX_channel_2].conf1.rx_en = 0;
                RMT.conf_ch[RMT_RX_channel_2].conf1.mem_owner = RMT_MEM_OWNER_TX;
                intr_st_1 += 2;

                // RMT storage pointer
                item_2 = RMTMEM.chan[RMT_RX_channel_2].data32;
            }
#endif
#if RMT_RX_CHANNEL_COUNT == 3
            // RMT_RX_3, corresponding to RMT channel 6
            if (intr_st & (1 << (1 + 3 * RMT_RX_channel_3)))
            {
                // change owner state, disable rx
                RMT.conf_ch[RMT_RX_channel_3].conf1.rx_en = 0;
                RMT.conf_ch[RMT_RX_channel_3].conf1.mem_owner = RMT_MEM_OWNER_TX;
                intr_st_1 += 4;

                // RMT storage pointer
                item_3 = RMTMEM.chan[RMT_RX_channel_3].data32;
            }
#endif

            // parsing output buffer
            uint32_t raw;

            // whether this transmission is valid
            bool this_valid = false;

            // setbit(TEST_PIN_2);

            // parse RMT item into a uint32_t
            if ((intr_st_1 & 1) && Parse_RMT_item(item_1, &raw))
            {
                LIT_R;
                if (intr_st_1 & 2)
                    LIT_G;
                else
                    QUENCH_G;
                if (intr_st_1 & 4)
                    LIT_B;
                else
                    QUENCH_B;

                last_RX_time = rec_time;
                // add element to the pool if parsing is successful
                raw_msg_buffer.push(Trans_info{raw, intr_st_1, rec_time});
                this_valid = true;
            }
#if RMT_RX_CHANNEL_COUNT >= 2
            else if ((intr_st_1 & 2) && Parse_RMT_item(item_2, &raw))
            {
                QUENCH_R;
                LIT_G;
                if (intr_st_1 & 4)
                    LIT_B;
                else
                    QUENCH_B;

                last_RX_time = rec_time;
                // add element to the pool if parsing is successful
                raw_msg_buffer.push(Trans_info{raw, intr_st_1 & 0b110, rec_time});
                this_valid = true;
            }
#endif
#if RMT_RX_CHANNEL_COUNT == 3
            else if ((intr_st_1 & 4) && Parse_RMT_item(item_3, &raw))
            {
                QUENCH_R;
                QUENCH_G;
                LIT_B;
                last_RX_time = rec_time;
                // add element to the pool if parsing is successful
                raw_msg_buffer.push(Trans_info{raw, 0b100, rec_time});
                this_valid = true;
            }
#endif
            else
            {
                QUENCH_R;
                QUENCH_G;
                QUENCH_B;
            }

            // reset memory and owner state, enable rx
            if (intr_st_1 & 1)
            {
                RMT.conf_ch[RMT_RX_channel_1].conf1.mem_wr_rst = 1;
                RMT.conf_ch[RMT_RX_channel_1].conf1.mem_owner = RMT_MEM_OWNER_RX;
                RMT.conf_ch[RMT_RX_channel_1].conf1.rx_en = 1;
            }
#if RMT_RX_CHANNEL_COUNT >= 2
            if (intr_st_1 & 2)
            {
                RMT.conf_ch[RMT_RX_channel_2].conf1.mem_wr_rst = 1;
                RMT.conf_ch[RMT_RX_channel_2].conf1.mem_owner = RMT_MEM_OWNER_RX;
                RMT.conf_ch[RMT_RX_channel_2].conf1.rx_en = 1;
            }
#endif
#if RMT_RX_CHANNEL_COUNT == 3
            if (intr_st_1 & 4)
            {
                RMT.conf_ch[RMT_RX_channel_3].conf1.mem_wr_rst = 1;
                RMT.conf_ch[RMT_RX_channel_3].conf1.mem_owner = RMT_MEM_OWNER_RX;
                RMT.conf_ch[RMT_RX_channel_3].conf1.rx_en = 1;
            }
#endif

            // clear RMT interrupt status.
            RMT.int_clr.val = intr_st;

            // delay for a bit so that the interrupt status could be cleared
            delay50ns;
            delay100ns;

            // // test end pulse
            // delayhigh500ns(TEST_PIN);
            // clrbit(TEST_PIN);
            // clrbit(TEST_PIN_2);
        }

        /**
         * @brief get raw uint32_t data from buffer(data comes from RX_ISR) and organize and store all parsed data packet.
         *        this function shall be called at ~100Hz rate (just not very frequently).
         */
        void Preprocess()
        {
            // keep doing till the buffer is empty
            while (raw_msg_buffer.n_elem)
            {
                // fetch message from buffer
                Trans_info info = raw_msg_buffer.pop();

                // whether the message is from the top emitter
                uint32_t emitter = info.data & (1 << (32 - 1));

                // setup a temporary value and extract some basic properties of the message.
                Msg_t temp_msg;
                temp_msg.raw = info.data;
                uint32_t robot_ID = temp_msg.robot_ID, msg_type = temp_msg.msg_type, msg_ID = temp_msg.msg_ID;

                // check type, if single message
                if (msg_type <= Single_transmission_msg_type)
                {
                    // parse msg correctly
                    Msg_single_t real_msg;
                    real_msg.raw = info.data;

                    // check for CRC
                    if (crc4_itu(real_msg.content) != real_msg.CRC)
                        continue;
                    else
                    {
                    }
                }
                // if not, must be multiple message type, then check if it's message
                else if (msg_ID)
                {
                }
                // if multiple message type and not message, must be header
                else
                {
                }
            }
        }
    }
}