#include "IrRX.hpp"
#include "RMTCoding.hpp"
#include "RMTMessageDefs.hpp"
#include "Utilities/FastIO.hpp"
#include "Utilities/DebugDefs.hpp"
#include "Utilities/CRCCheck.hpp"
#include "Utilities/Circbuffer.hpp"

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

            class Robot_RX
            {
            public:
                uint32_t robot_ID;
                uint64_t last_reception_time;

                std::array<Circbuffer<Parsed_msg_single, msg_buffer_history_size>, Single_transmission_msg_type + 1> single_data;
                std::array<Circbuffer<Parsed_msg_multiple, msg_buffer_history_size>, Msg_type_max - Single_transmission_msg_type> multiple_dat;

                Circbuffer<Msg_timing_t, msg_buffer_history_size> timing_dat;
            };

            /**
             * @brief index of all robots
             *        msg_buffer_dict[Robot_ID] gives you the pointer of Robot_ID in msg_buffer
             *        if it's nullptr, then Robot_ID do not exist in msg_buffer
             *        should be initialized all to nullptr
             */
            std::array<Robot_RX *, 1 << Robot_ID_bits> msg_buffer_dict;

            /**
             * @brief message buffer that stores data in the form of msg_buffer[dict_id][msg_type][n] gives the nth oldest message of msg_type sent by robot_id.
             *        This buffer only provide information of content and last reception time, but not anything related to localization like emitter position or receiver position.
             */
            std::array<Robot_RX, Max_robots_simultaneous> msg_buffer;

            /**
             * @brief recent_msg_buffer stores data that recently completed transmission.
             */
            std::array<Circbuffer<Parsed_msg_completed, recent_msg_buffer_history_size>, Msg_type_max + 1> recent_msg_buffer;

            /**
             * @brief a buffer that stores timing information of
             *        'when the first message from each robot is received by each receiver in this revolution'.
             */
            Circbuffer<Msg_timing_t, timing_buffer_history_size> recent_timing_buffer;

            /**
             * @brief flag that will increase when write. used to resolve concurrency (check notes at the beginning of this file).
             */
            uint32_t io_flag = 0;
        }

        /**
         * @brief convert a regular message to msg_completed form
         *
         * @param msg Parsed_msg_single message
         * @return Parsed_msg_completed completed form
         */
        Parsed_msg_completed To_completed_form(uint32_t robot_ID, uint32_t msg_type, Parsed_msg_single msg)
        {
            Parsed_msg_completed res;

            res.robot_ID = robot_ID;
            res.msg_type = msg_type;
            res.finish_reception_time = msg.last_reception_time;
            res.content_length = 1;
            res.content[0] = msg.content;

            return res;
        }

        /**
         * @brief convert a regular message to msg_completed form
         *
         * @param msg Parsed_msg_multiple message
         * @return Parsed_msg_completed completed form
         *
         * @note if the message is not complete, the content length will be 0.
         */
        Parsed_msg_completed To_completed_form(uint32_t robot_ID, uint32_t msg_type, Parsed_msg_multiple msg)
        {
            if (msg.Content_valid_Q())
            {
                Parsed_msg_completed res;

                res.robot_ID = robot_ID;
                res.msg_type = msg_type;
                res.finish_reception_time = msg.last_reception_time;
                res.content_length = msg.Get_content_length();
                std::copy(msg.Get_content_pointer(), msg.Get_content_pointer() + msg.Get_content_length(), res.content);

                return res;
            }
            else
                return Parsed_msg_completed{};
        }

        /**
         * @brief find a place in msg_buffer to place a new robot.
         *          1. find the oldest robot
         *          2. delete the link of old robot in dict
         *          3. reset that place and make it the new robot's
         *          4. establish link for new robot in dict
         *
         * @param robot_ID new robot's ID
         * @return Robot_RX* pointer to the place corresponding to the robot_ID
         *
         * @note when robot_ID already exists in msg_buffer, then directly return the corresponding pointer.
         */
        Robot_RX *Add_new_robot(uint32_t robot_ID)
        {
            // return directly if the robot already exists in msg_buffer.
            if (msg_buffer_dict[robot_ID])
                return msg_buffer_dict[robot_ID];

            // minimum time
            uint64_t min_t = msg_buffer[0].last_reception_time, temp_t;
            // position corresponding to the minimum time
            uint32_t min_pos = 0;

            // if min_t == 0, that means the [0] place is free.
            // min_t only =0 if it's completely fresh, so very light setup
            if (min_t == 0)
            {
                // 3. make it the new robot's place
                msg_buffer[0].robot_ID = robot_ID;
                // 4. establish link for new robot in dict
                msg_buffer_dict[robot_ID] = &msg_buffer[0];
            }

            // 1. find the oldest robot
            for (uint32_t i = 1; i < Max_robots_simultaneous; i++)
            {
                temp_t = msg_buffer[i].last_reception_time;

                // if temp_t == 0, that means the [i] place is free.
                // temp_t only =0 if it's completely fresh, so very light setup
                if (temp_t == 0)
                {
                    // 3. make it the new robot's place
                    msg_buffer[i].robot_ID = robot_ID;
                    // 4. establish link for new robot in dict
                    msg_buffer_dict[robot_ID] = &msg_buffer[i];
                }
                // if this is older
                else if (temp_t < min_t)
                {
                    min_t = temp_t;
                    min_pos = i;
                }
            }

            // 2. delete the link of old robot in dict
            msg_buffer_dict[msg_buffer[min_pos].robot_ID] = nullptr;

            // 3. reset that place and make it the new robot's
            Robot_RX *res = &msg_buffer[min_pos];
            res->last_reception_time = 0;
            res->robot_ID = robot_ID;
            for (uint32_t i = 0; i <= Single_transmission_msg_type; i++)
                res->single_data[i].clear();
            for (uint32_t i = 0; i < Msg_type_max - Single_transmission_msg_type; i++)
                res->multiple_dat[i].clear();

            // 4. establish link for new robot in dict
            msg_buffer_dict[robot_ID] = res;

            return res;
        }

        uint32_t Parsed_msg_multiple::Add_content(Trans_info info)
        {
            // re-parse data
            Msg_t msg;
            msg.raw = info.data;

            // filling status of corresponding position
            bool this_filled = filling_status & (1 << (msg.msg_ID));

            // check if the new data is consistent
            // consistency is determind by three factors:
            //      1. time elapsed between last reception and this reception is less than Data_expire_time (it's not obsolete).
            //      2. msg_ID_init is consistent (probably the same message).
            //      3. msg_ID_max == 0 or msg_ID <= msg_ID_max (msg_ID out of bounds).
            //      4. either the data is missing or the data is the same as previous (data consistency).
            bool meta_consistency = (info.time - last_reception_time < Data_expire_time) && (msg.msg_ID_init == msg_ID_init) && (msg_ID_max == 0 || msg_ID_max >= msg.msg_ID);
            bool data_consistency = (this_filled == 0 || content[msg.msg_ID - 1] == msg.content);
            bool consistency = meta_consistency && data_consistency;

            // check if data is full and checked
            if (content_valid_flag)
            {
                // if consistent, no need for action except for update timing, if not, open new instance.
                if (consistency)
                {
                    last_reception_time = info.time;
                    return 0;
                }
                else
                    return 1;
            }
            // if not full...
            else
            {
                // if consistent
                if (consistency)
                {
                    // if the data is filled, then data is a duplicate, update time and quit.
                    if (this_filled)
                    {
                        last_reception_time = info.time;
                        return 0;
                    }
                    // if not, then the data is new, add it to the list
                    else
                    {
                        last_reception_time = info.time;
                        msg_count++;
                        filling_status |= 1 << (msg.msg_ID);
                        content[msg.msg_ID - 1] = msg.content;

                        // check if full now
                        // at least header should present to complete the message, so msg_count > 1.
                        // if full, check if valid
                        if (msg_count > 1 && msg_count == msg_ID_max + 1)
                        {
                            // if data is valid, set content_valid_flag
                            // if not, do nothing and we will reset the whole structure later.
                            if (crc8_maxim(content, msg_ID_max) == CRC)
                            {
                                content_valid_flag = 1;
                                return 2;
                            }
                        }
                        else
                        {
                            return 0;
                        }
                    }
                }
                // if not consistent, but meta is consistent, then data must be inconsistent, we just ignore the new data.
                // we do so instead of replacing the old with the new is that do nothing is faster, and these two data has equal chance of being wrong.
                // depending on the requirement, we can either update timing or not. Here we do not.
                else if (meta_consistency)
                {
                    return 0;
                }

                // if meta is not consistent or if CRC check failed, the old data will never be completed, so we discard it directly and re-start the pool.
                last_reception_time = info.time;
                msg_ID_init = msg.msg_ID_init;
                msg_ID_max = 0;
                msg_count = 1;
                filling_status = 1 << (msg.msg_ID);
                CRC = 0;
                content_valid_flag = 0;
                content[msg.msg_ID - 1] = msg.content;

                return 0;
            }
        }

        uint32_t Parsed_msg_multiple::Add_header(Trans_info info)
        {
            // re-parse data
            Msg_header_t msg;
            msg.raw = info.data;

            // filling status of header
            bool this_filled = filling_status & 1;

            // check if the new data is consistent
            // consistency is determind by three factors:
            //      1. time elapsed between last reception and this reception is less than Data_expire_time (it's not obsolete).
            //      2. msg_ID_init is consistent (probably the same message).
            //      3. no message with msg_ID > msg_ID_max has been received, which means (filling_status >> msg.msg_ID_len) at max is 1 (msg_ID out of bounds).
            //      4. either the header is missing or the header is the same as previous (data consistency).
            bool meta_consistency = (info.time - last_reception_time < Data_expire_time) && (msg.msg_ID_init == msg_ID_init) && ((filling_status >> msg.msg_ID_len) < 2);
            bool data_consistency = (this_filled == 0 || (CRC == msg.CRC && msg_ID_max == msg.msg_ID_len));
            bool consistency = meta_consistency && data_consistency;

            // check if data is full and checked
            if (content_valid_flag)
            {
                // if consistent, no need for action except for update timing, if not, open new instance.
                if (consistency)
                {
                    last_reception_time = info.time;
                    return 0;
                }
                else
                    return 1;
            }
            // if not full...
            else
            {
                // if meta data is consistent
                // note that here it's slightly different than the Add_content case
                // because when we meet new, inconsistent metadata, we always OVERRIDE.
                // consider a case where msg_ID_len is incorrect and is larger than the actual value.
                // if we never replace the old one, the content will never be full, thus it will never check CRC.
                // but at the same time, duplicated data will come in at all times, so refreshing last_reception_time to keep the pool fresh.
                // this means that the pool will always be incorrect yet never reset.
                // by replacing the metadata whenever there's inconsistency between the old and the new, we can ensure that even if the initial metadata is wrong
                // it will be overriden at some point, fixing the structure.
                if (meta_consistency)
                {
                    // if the header is filled and data is consistent, then header is a duplicate, update time and quit.
                    if (this_filled && data_consistency)
                    {
                        last_reception_time = info.time;
                        return 0;
                    }
                    // if not, then either
                    //      1. the header is new or
                    //      2. we have a inconsistency between old and new header
                    // in either case, we have to update the information
                    else
                    {
                        last_reception_time = info.time;
                        // only add msg_count when not filled
                        if (!this_filled)
                            msg_count++;
                        filling_status |= 1;
                        CRC = msg.CRC;
                        msg_ID_max = msg.msg_ID_len;

                        // check if full now
                        // at least header should present to complete the message, so msg_count > 1.
                        // if full, check if valid
                        if (msg_count == msg_ID_max + 1)
                        {
                            // if data is valid, set content_valid_flag
                            // if not, do nothing and we will reset the whole structure later.
                            if (crc8_maxim(content, msg_ID_max) == CRC)
                            {
                                content_valid_flag = 1;
                                return 2;
                            }
                        }
                        else
                        {
                            return 0;
                        }
                    }
                }

                // if meta is not consistent or if CRC check failed, the old data will never be completed, so we discard it directly and re-start the pool.
                last_reception_time = info.time;
                msg_ID_init = msg.msg_ID_init;
                msg_ID_max = msg.msg_ID_len;
                msg_count = 1;
                filling_status = 1;
                CRC = msg.CRC;
                content_valid_flag = 0;

                return 0;
            }
        }

        inline uint32_t Get_io_flag()
        {
            return io_flag;
        }

        Parsed_msg_completed Get_latest_msg_by_bot(uint32_t robot_ID, uint32_t msg_type, uint32_t age)
        {
            uint32_t curr_flag, empty = 0;
            Parsed_msg_completed res;

            do
            {
                // by default it's not empty
                empty = 0;

                // store flag before data read
                curr_flag = io_flag;

                // get index in msg_buffer from dictionary
                Robot_RX *ptr = msg_buffer_dict[robot_ID];
                // if no data is stored for this robot, just output nullptr
                if (ptr)
                {
                    // if single message type
                    // note that single message type is ALWAYS completed and valid!
                    if (msg_type <= Single_transmission_msg_type)
                    {
                        auto pool = ptr->single_data[msg_type];
                        // if there's element of this particular type, check if the first message is finished.
                        // if peeked message is valid, then just take the age th message.
                        if (pool.n_elem > age)
                            res = To_completed_form(robot_ID, msg_type, pool.peek_tail(age));
                        else
                            empty = 1;
                    }
                    else
                    {
                        auto pool = ptr->multiple_dat[msg_type - Single_transmission_msg_type - 1];

                        if (pool.n_elem)
                            empty = 1;
                        // note that we need the latest "completed" element
                        else if (pool.peek_tail().Content_valid_Q())
                        {
                            if (pool.n_elem > age)
                                res = To_completed_form(robot_ID, msg_type, pool.peek_tail(age));
                            else
                                empty = 1;
                        }
                        else
                        {
                            if (pool.n_elem > age + 1)
                                res = To_completed_form(robot_ID, msg_type, pool.peek_tail(age + 1));
                            else
                                empty = 1;
                        }
                    }
                }
                else
                    empty = 1;
            }
            // repeat if write task preempted this task
            while (io_flag != curr_flag);

            return empty ? Parsed_msg_completed{} : res;
        }

        Parsed_msg_completed Get_latest_msg_by_type(uint32_t msg_type, uint32_t age)
        {
            auto buf = recent_msg_buffer[msg_type];
            uint32_t curr_flag, empty = 0;
            Parsed_msg_completed res;

            do
            {
                curr_flag = buf.io_flag;
                if (buf.n_elem > age)
                {
                    empty = 0;
                    res = buf.peek_tail(age);
                }
                else
                    empty = 1;
            }
            // repeat if write edited this particular buffer
            while (buf.io_flag != curr_flag);

            return empty ? Parsed_msg_completed{} : res;
        }

        inline Circbuffer_copycat<Parsed_msg_completed, recent_msg_buffer_history_size> Get_msg_buffer_by_type(uint32_t msg_type)
        {
            return Circbuffer_copycat<Parsed_msg_completed, recent_msg_buffer_history_size>{&recent_msg_buffer[msg_type]};
        }

        // Msg_timing_t Get_timing_data(uint32_t age)
        // {
        //     uint32_t curr_flag;
        //     Msg_timing_t res;
        //
        //     do
        //     {
        //         curr_flag = io_flag;
        //         res = timing_buffer.peek_tail(age);
        //     }
        //     // repeat if write task preempted this task
        //     while (io_flag != curr_flag);
        //
        //     return res;
        // }

        uint32_t Copy_timing_data(Msg_timing_t *start)
        {
            uint32_t curr_flag;
            uint32_t len = 0;

            do
            {
                curr_flag = io_flag;

                len = recent_timing_buffer.n_elem;
                for (int i = 0; i < recent_timing_buffer.n_elem; i++)
                {
                    start[i] = recent_timing_buffer.peek_tail(i);
                }
            }
            // repeat if write task preempted this task
            while (io_flag != curr_flag);

            return len;
        }

        // actual tasks
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
         *
         * @note I know that this function is too long, but it's still pretty structured and clear so I am not gonna split it into multiple functions.
         */
        void Preprocess()
        {
            // increment io_flag as a coarse lock
            // all read task should have lower priority than preprocess task
            // and they should check io_flag to determine whether the read has been preempted by Preprocess
            io_flag++;

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

                // obtain pointer to buffer, create Robot_RX instance if needed.
                Robot_RX *robot_ptr = Add_new_robot(robot_ID);

                // deal with timing buffer
                // note that we always update timing buffer regardless of message's validity
                // I am fully aware that incorrect messages with incorrect robot_ID might disrupt this routine, but I don't have a choice right now.
                // because we do not have a way of verifying whether the message is error free for multiple-transmission messages
                // if we are gonna do this, we have to add CRC to all messages regardless of type, which is kinda expensive.
                // but anyway, now just a sanity check about pulse count and pulse width should be quite enough for most scenarios, so we will ignore this for now
                // if there's a problem later, we can always add an additional CRC about the raw message in the future.
                robot_ptr->timing_dat;

                // deal with data buffers
                // check type, if single message
                if (msg_type <= Single_transmission_msg_type)
                {
                    // parse msg correctly
                    Msg_single_t real_msg;
                    real_msg.raw = info.data;
                    uint32_t content = real_msg.content, msg_ID_init = real_msg.msg_ID_init;

                    // check for CRC, if cannot pass, discard message.
                    if (crc4_itu(content) != real_msg.CRC)
                        continue;

                    // update robot's timing info
                    robot_ptr->last_reception_time = info.time;

                    // fetch latest message if exist
                    if (robot_ptr->single_data[msg_type].n_elem)
                    {
                        Parsed_msg_single &tail = robot_ptr->single_data[msg_type].peek_tail();
                        // check if is the same and not obsolete, if so, simply change tail's reception time and continue.
                        if (tail.msg_ID_init == msg_ID_init && tail.content == content && (info.time - tail.last_reception_time) < Data_expire_time)
                        {
                            tail.last_reception_time = info.time;
                            continue;
                        }
                    }

                    // if one of the following is true
                    //  1. buffer is empty
                    //  2. the messages are not the same
                    //  3. the old message is just too old
                    // construct Parsed_msg_single and push it inside pool
                    Parsed_msg_single temp;
                    temp.content = content;
                    temp.msg_ID_init = msg_ID_init;
                    temp.last_reception_time = info.time;

                    robot_ptr->single_data[msg_type].push(temp);

                    // because it's single transmission message, always add to the recent_msg_buffer
                    recent_msg_buffer[msg_type].push(To_completed_form(robot_ID, msg_type, temp));
                }
                // if not, must be multiple message type
                else
                {
                    // update timing
                    // note that the timing for robot might be later than the timing for individual messages
                    // because when faulty multiple message type message is received, we will not update timing for individual pool
                    // but we will always update timing for robot time.
                    robot_ptr->last_reception_time = info.time;
                    // the corresponding typed circular buffer
                    auto circbuf = robot_ptr->multiple_dat[msg_type - Single_transmission_msg_type - 1];

                    // bit 0 -> whether the data should be in a new bin
                    // bit 1 -> whether the data is just full
                    uint32_t flags = 0b01;

                    // check if it's content message
                    if (msg_ID)
                    {
                        // if content message, try to figure out if the corresponding pool is empty
                        if (circbuf.n_elem)
                        {
                            // if not, try to add content to the existing, latest message
                            flags = circbuf.peek_tail().Add_content(info);
                        }
                        // if requires to generate a new content (flags's first bit is 1)
                        if (flags | 1)
                        {
                            Parsed_msg_multiple temp;
                            flags = temp.Add_content(info);
                            circbuf.push(temp);
                        }
                        // if data has just been finished update recent_msg_buffer
                        if (flags | 2)
                        {
                            recent_msg_buffer[msg_type].push(To_completed_form(robot_ID, msg_type, circbuf.peek_tail()));
                        }
                    }
                    // if multiple message type and not content, must be header
                    else
                    {
                        // if header message, try to figure out if the corresponding pool is empty
                        if (circbuf.n_elem)
                        {
                            // if not, try to add header to the existing, latest message
                            flags = circbuf.peek_tail().Add_header(info);
                        }
                        // if requires to generate a new content (flags's first bit is 1)
                        if (flags | 1)
                        {
                            Parsed_msg_multiple temp;
                            flags = temp.Add_header(info);
                            circbuf.push(temp);
                        }
                        // if data has just been finished update recent_msg_buffer
                        if (flags | 2)
                        {
                            recent_msg_buffer[msg_type].push(To_completed_form(robot_ID, msg_type, circbuf.peek_tail()));
                        }
                    }
                }
            }
        }

        void Init()
        {
            xxxx;
        }
    }
}