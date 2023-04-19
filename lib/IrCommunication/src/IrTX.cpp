#include "IrTX.hpp"
#include "RMTCoding.hpp"
#include <RobotDefs.hpp>
#include <FastIO.hpp>
#include <DebugDefs.hpp>
#include <CRCCheck.hpp>
#include "bootloader_random.h"
#include "hal/rmt_ll.h"
#include "driver/timer.h"

namespace IR
{
    using namespace detail;

    namespace TX
    {
        namespace
        {
            /**
             * @brief trigger timer group for TX_ISR
             */
            timer_group_t TX_timer_group = timer_group_t((IR_TX_trigger_timer_channel & 2U) >> 1);

            /**
             * @brief trigger timer num for TX_ISR
             */
            timer_idx_t TX_timer_num = timer_idx_t(IR_TX_trigger_timer_channel & 1U);

            /**
             * @brief generate the next triggering time
             *
             * @return uint32_t time in us.
             */
            uint32_t Generate_trigger_time()
            {
                return esp_random() % (RMT_TX_trigger_period_max - RMT_TX_trigger_period_min) + RMT_TX_trigger_period_min;
            }

            /**
             * @brief a helper class that convert the vector of input data to
             * rmt_item32_t and provide correct pointer for TX_ISR.
             */
            class TX_data
            {
            public:
                /**
                 * @brief default constructor
                 */
                TX_data() = default;

                /**
                 * @brief update tx data object with data and complete settings.
                 *
                 * @param type msg_type of data
                 * @param raw raw data, each uint16_t should contain Msg_content_bits of data.
                 * @note When constructing single transmission data, construct a vector out of it first.
                 */
                void TX_update(const std::vector<uint16_t> &raw)
                {
                    // number of all messages - 1 (excluding header message)
                    uint32_t n_msg = raw.size();

                    // check msg_type
                    // if single transmission
                    if (msg_type <= Single_transmission_msg_type)
                    {
                        // notify user if msg_type is not proper.
                        DEBUG_C(
                            if (n_msg > 1)
                                Serial.println("Feeding multiple transmission data into single transmission messages! Only the first data will be sent!"));
                        // resize, generate RMT_item, and setup pointer
                        data.resize(RMT_TX_length);
                        Generate_RMT_item(data.data(), Msg_single_t{{This_robot_ID, msg_type, msg_ID_init, crc4_itu(raw[0]), raw[0]}}.raw);

                        end_ptr = ptr = data.data();
                    }
                    // multiple transmissions
                    else
                    {
                        // setup container
                        data.resize((n_msg + 1) * RMT_TX_length);

                        // create header
                        Generate_RMT_item(data.data(), Msg_header_t{{This_robot_ID, msg_type, msg_ID_init, 0, n_msg, crc8_maxim(raw.data(), raw.size())}}.raw);

                        // cycle through data and generate data
                        for (size_t msgid = 1; msgid <= n_msg; msgid++)
                            Generate_RMT_item(data.data() + msgid * RMT_TX_length, Msg_t{{This_robot_ID, msg_type, msg_ID_init, msgid, raw[msgid - 1]}}.raw);

                        // setup pointers
                        ptr = data.data();
                        end_ptr = data.data() + n_msg * RMT_TX_length;
                    }

                    // change msg_ID_init for next data set
                    msg_ID_init = (msg_ID_init ? 0 : 1);
                }

                /**
                 * @brief Give correct pointer for transmission
                 * @return rmt_item32_t* starting pointer
                 */
                rmt_item32_t *Get_data()
                {
                    auto ptr_temp = ptr;
                    // increment pointer
                    if (ptr == end_ptr)
                        ptr = data.data();
                    else
                        ptr += RMT_TX_length;

                    return ptr_temp;
                }

                /**
                 * @brief type of msg. Though I didn't pose explicit restrictions here,
                 *        you are not allowed to change it except in initialization!
                 */
                uint32_t msg_type = 0;

            private:
                /**
                 * @brief complete rmt data
                 */
                std::vector<rmt_item32_t> data = {};

                /**
                 * @brief msg_id_init
                 */
                uint32_t msg_ID_init = 0;

                /**
                 * @brief a pointer to the next group of data to be sent
                 */
                rmt_item32_t *ptr = nullptr;

                /**
                 * @brief end pointer that points to the starting of the last transmission!
                 */
                rmt_item32_t *end_ptr = nullptr;
            };

            // here we construct a scheduler to help us determine which type of
            // message to send out. the implementation might be a bit
            // complicated, but the basic idea is just to simulate the RTOS's
            // behavoir and the implementation is similar too.
            // So we have a linked list which is sorted by
            //     1. priority
            //     2. last transmission time (this is slightly different from
            // FreeRTOS's mechanism, which determines the priority based on last
            // 'not-blocked' time)
            // the data structure is as follows:
            //
            // pointers:   start_ptr  pe_ptr[5]            pe_ptr[2]  pe_ptr[0]
            // priority:      5          5          2          2          0
            // nodes:       node_1 <-> node_2 <-> node_3 <-> node_4 <-> node_0
            //
            // where pe_ptr is prio_level_end_ptr, representing the pointer to
            // the least prioritized node. node_0 is the idle node and also the
            // end of the lined list. there should NOT be any other tasks with
            // priority 0!

            /**
             * @brief nodes used in scheduler, nodes can form a linked list
             */
            class Scheduler_node
            {
            public:
                Scheduler_node() = default;
                Scheduler_node(const TX_data &v1, Scheduler_node *v2, Scheduler_node *v3) : val(v1), prev(v2), next(v3) {}

                // class used to handle data
                TX_data val{};

                // priority of task
                uint32_t priority = 0;
                // how many msgs before expiration
                int32_t expiration_counter = -1;
                // how many idle before next transmission (frequency of transmission)
                uint32_t transmission_counter_max = 1;
                // how many idle left before next transmission
                uint32_t transmission_counter = 0;

                // previous node in linked list
                Scheduler_node *prev = nullptr;
                // next node in linked list
                Scheduler_node *next = nullptr;
            };

            /**
             * @brief a array containing all (TX_type_max + 1) Scheduler_node
             *
             * @note node_list[i] corresponds to Msg of type i undefined the
             * order of nodes in this array is not related to the order of nodes
             * in the linked list they form
             */
            Scheduler_node node_list[detail::Msg_type_max + 1];

            /**
             * @brief starting pointer of the linked list, not the starting
             * pointer of node_list!
             */
            Scheduler_node *start_ptr = nullptr;

            /**
             * @brief ending pointer for each priority level
             */
            Scheduler_node *prio_level_end_ptr[TX_priority_max + 1] = {nullptr};

            /**
             * @brief whether to enable reading/transmitting data in TX_ISR
             */
            bool TX_enable_flag = true;

            /**
             * @brief return the reference to the corresponding TX_data object, so
             * you can manipulate it.
             *
             * @param ntype type of message
             * @return TX_data& reference to TX_data object
             */
            TX_data &Get_TX_data(const uint32_t ntype)
            {
                return node_list[ntype].val;
            }

            /**
             * @brief add a task of certain type from scheduler. Note that this
             * task MUST have already been used and temporarily paused.
             *
             * @param ntype type of task
             */
            void Add_back_to_schedule(const uint32_t ntype)
            {
                // temp pointer to the msg
                Scheduler_node *temp_ptr = node_list + ntype;
                // ptr of the end of the new priority
                Scheduler_node *&prio_end_ptr = prio_level_end_ptr[temp_ptr->priority];
                // if there are previous messages of this priority
                if (prio_end_ptr)
                {
                    // add this element to the end of this priority
                    prio_end_ptr->next->prev = temp_ptr;
                    temp_ptr->next = prio_end_ptr->next;
                    prio_end_ptr->next = temp_ptr;
                    temp_ptr->prev = prio_end_ptr;

                    // set the end pointer to this msg
                    prio_end_ptr = temp_ptr;
                }
                // if this is brand new priority level
                else
                {
                    // this new priority level's end pointer is this msg's pointer
                    prio_end_ptr = temp_ptr;

                    Scheduler_node *high_ptr = nullptr;
                    // find the first priority level higher than this
                    for (size_t i = (temp_ptr->priority) + 1; i <= TX_priority_max; i++)
                    {
                        if (prio_level_end_ptr[i])
                        {
                            high_ptr = prio_level_end_ptr[i];
                            break;
                        }
                    }

                    // if found, then add after this
                    if (high_ptr)
                    {
                        high_ptr->next->prev = temp_ptr;
                        temp_ptr->next = high_ptr->next;
                        high_ptr->next = temp_ptr;
                        temp_ptr->prev = high_ptr;
                    }
                    // if not, this must be the highest priority msg, which
                    // makes it the starting point
                    else
                    {
                        start_ptr->prev = temp_ptr;
                        temp_ptr->next = start_ptr;
                        temp_ptr->prev = nullptr;
                        start_ptr = temp_ptr;
                    }
                }
            }

            rmt_item32_t *Schedule_next()
            {
                Scheduler_node *temp_ptr = start_ptr;

                // decrement all transmission counter, but stop at zero
                // only change back to counter_max at when fired!
                for (size_t i = 0; i <= Msg_type_max; i++)
                {
                    if (node_list[i].transmission_counter)
                    {
                        node_list[i].transmission_counter--;
                    }
                }

                // check which to transmit
                while (temp_ptr)
                {
                    uint32_t type = temp_ptr->val.msg_type;
                    // skip to next when this message shouldn't fire
                    if (temp_ptr->transmission_counter)
                    {
                        temp_ptr = temp_ptr->next;
                        // if it should fire, and it's not default msg (type 0)
                    }
                    else if (type)
                    {
                        // if the expiration counter is already 0, remove task by
                        // putting it at the back of priority 0. note that this
                        // message will fire!
                        if (temp_ptr->expiration_counter == 0)
                        {
                            Remove_from_schedule(type);
                        }
                        else
                        {
                            // decrease the expiration counter if it's not -1
                            if (temp_ptr->expiration_counter >= 0)
                            {
                                temp_ptr->expiration_counter--;
                            }

                            // reset the transmission counter
                            temp_ptr->transmission_counter =
                                temp_ptr->transmission_counter_max;

                            // ptr of the end of this priority
                            auto *prio_end_ptr = prio_level_end_ptr[temp_ptr->priority];
                            // move this msg to the end of this priority if it's not
                            if (temp_ptr != prio_end_ptr)
                            {
                                Remove_from_schedule(type);
                                Add_back_to_schedule(type);
                            }
                        }

                        // Serial.println(temp_ptr->val.msg_type);
                        // return the data
                        return temp_ptr->val.Get_data();
                    }
                    // if it's default msg (type 0) return without anything else
                    else
                    {
                        // Serial.println(0);
                        return node_list[0].val.Get_data();
                    }
                }

                DEBUG_C(Serial.println("While loop broken @ IR::TX::<unnamed>::Schedule_next: This shouldn't happen!"));
                return node_list[0].val.Get_data();
            }

            /**
             * @brief TX timer interrupt handler that transmit data though physical interface
             *
             * @note this ISR should be run on core 0, leaving core 1 for RX tasks
             * exclusively anyways. TX_ISR will reset itself and fire based on
             * timing.
             */
            bool IRAM_ATTR TX_ISR(void *)
            {
                // reset alarm value fast using native api
                timer_group_set_alarm_value_in_isr(TX_timer_group, TX_timer_num, Generate_trigger_time());

                DEBUG_C(setbit(DEBUG_PIN_1));

                // RMT TX only when data is not being modified.
                if (TX_enable_flag)
                {
                    // let the corresponding task write data to pool maybe we don't
                    // need a buffer anyways. just let blah blah blah return a
                    // starting pointer and we should make sure that no editing will
                    // happen on the content this pointer points to.
                    rmt_item32_t *v = Schedule_next();

                    // write both register
                    for (size_t i = 0; i < RMT_TX_length; i++)
                    {
                        RMTMEM.chan[RMT_TX_channel_2].data32[i].val = RMTMEM.chan[RMT_TX_channel_1].data32[i].val = (v + i)->val;
                    }

                    // edit channel 2 register
                    RMTMEM.chan[RMT_TX_channel_2].data32[0].duration0 -= detail::RMT_sync_ticks_num;
                    RMTMEM.chan[RMT_TX_channel_2].data32[RMT_data_pulse_count].duration1 = CH2_final_pulse_duration;

                    // reset pointers
                    rmt_ll_tx_reset_pointer(&RMT, RMT_TX_channel_1);
                    rmt_ll_tx_reset_pointer(&RMT, RMT_TX_channel_2);

                    // start RMT as fast as possible by directly manipulating the registers
                    RMT.conf_ch[RMT_TX_channel_1].conf1.tx_start = 1;
                    // (maybe) a tiny delay to sync the channels
                    // __asm__ __volatile__("nop;");
                    RMT.conf_ch[RMT_TX_channel_2].conf1.tx_start = 1;
                }

                DEBUG_C(clrbit(DEBUG_PIN_1));

                // don't do context switching!
                return false;
            }
        } // anonymous namespace

        void Init()
        {
            // enable random number generation
            bootloader_random_enable();

            // RMT peripheral start
#if RMT_TX_CHANNEL_COUNT >= 1
            pinMode(RMT_TX_PIN_1, OUTPUT);
            clrbit(RMT_TX_PIN_1);
            // config TX
            rmt_config_t rmt_tx;
            rmt_tx.channel = RMT_TX_channel_1;
            rmt_tx.gpio_num = (gpio_num_t)RMT_TX_PIN_1;
            rmt_tx.clk_div = RMT_clock_div;
            rmt_tx.mem_block_num = 1;
            rmt_tx.rmt_mode = RMT_MODE_TX;
            rmt_tx.tx_config.loop_en = false;
            // We modulate our own signal, instead of using carrier!
            rmt_tx.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
            rmt_tx.tx_config.carrier_en = 0;
            rmt_tx.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
            rmt_tx.tx_config.idle_output_en = 1;
            // initialization of RMT
            DEBUG_C(
                if (rmt_config(&rmt_tx) != ESP_OK)
                    Serial.println("RMT TX_1 init failed!"));

            // set source clk to APB clock
            rmt_set_source_clk(RMT_TX_channel_1, RMT_BASECLK_APB);

            // disable interrupt
            rmt_set_tx_intr_en(RMT_TX_channel_1, false);
            rmt_set_tx_thr_intr_en(RMT_TX_channel_1, false, 1);

            DEBUG_C(Serial.println("TX_1 init successful!"));
#endif
// second emitter
#if RMT_TX_CHANNEL_COUNT > 1
            pinMode(RMT_TX_PIN_2, OUTPUT);
            clrbit(RMT_TX_PIN_2);
            // config TX
            rmt_config_t rmt_tx_2;
            rmt_tx_2.channel = RMT_TX_channel_2;
            rmt_tx_2.gpio_num = (gpio_num_t)RMT_TX_PIN_2;
            rmt_tx_2.clk_div = RMT_clock_div;
            rmt_tx_2.mem_block_num = 1;
            rmt_tx_2.rmt_mode = RMT_MODE_TX;
            rmt_tx_2.tx_config.loop_en = false;
            // We modulate our own signal, instead of using carrier!
            rmt_tx_2.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
            rmt_tx_2.tx_config.carrier_en = 0;
            rmt_tx_2.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
            rmt_tx_2.tx_config.idle_output_en = 1;

            // initialization of RMT
            DEBUG_C(
                if (rmt_config(&rmt_tx_2) != ESP_OK)
                    Serial.println("RMT TX_2 init failed!"));

            // set source clk to APB clock
            rmt_set_source_clk(RMT_TX_channel_2, RMT_BASECLK_APB);

            // disable interrupt
            rmt_set_tx_intr_en(RMT_TX_channel_2, false);
            rmt_set_tx_thr_intr_en(RMT_TX_channel_2, false, 1);

            // uint8_t val=0;
            // rmt_get_clk_div(RMT_TX_channel_1, &val);
            // DEBUG_C(Serial.println(val));
            // rmt_get_clk_div(RMT_TX_channel_2, &val);
            // DEBUG_C(Serial.println(val));

            DEBUG_C(Serial.println("TX_2 init successful!"));
#endif

            // scheduler init
            // initialize type
            for (size_t i = 0; i <= detail::Msg_type_max; i++)
            {
                node_list[i].val.msg_type = i;
            }

            // initialize idle msg
            node_list[0].val.TX_update({0});

            // initialize staring and ending pointer
            start_ptr = prio_level_end_ptr[0] = node_list;

            DEBUG_C(Serial.println("Scheduler init successful!"));

            // auto t = micros();
            // // Add_to_schedule(1, {1}, 2, -1, 2);
            // // Add_to_schedule(2, {2}, 2, -1, 1);
            // // Add_to_schedule(3, {3}, 3, -1, 4);
            // // Add_to_schedule(5, {5}, 2, -1, 5);
            // for (size_t i = 0; i < 100000; i++)
            //     Schedule_next();
            // Serial.println(micros() - t);
            // Serial.println("Schedule_next test successful!");

            // config timer to transmit TX once in a while
            // we are using timer 3 to prevent confiction
            // timer ticks 1MHz
            timer_config_t config = {
                .alarm_en = TIMER_ALARM_EN,
                .counter_en = TIMER_PAUSE,
                .intr_type = TIMER_INTR_LEVEL,
                .counter_dir = TIMER_COUNT_UP,
                .auto_reload = TIMER_AUTORELOAD_EN,
                .divider = 80U};

            timer_init(TX_timer_group, TX_timer_num, &config);
            timer_set_counter_value(TX_timer_group, TX_timer_num, 0);
            timer_set_alarm_value(TX_timer_group, TX_timer_num, Generate_trigger_time());
            timer_enable_intr(TX_timer_group, TX_timer_num);
            timer_isr_callback_add(TX_timer_group, TX_timer_num, &TX_ISR, nullptr, 0);
            timer_start(TX_timer_group, TX_timer_num);

            DEBUG_C(Serial.println("Trigger_timer init successful!"));
        }

        /**
         * @brief remove an element from the list, ntype is the type number
         *
         * @param type type of message
         */
        void Remove_from_schedule(const uint32_t type)
        {
            // temp pointer to the msg
            Scheduler_node *const temp_ptr = node_list + type;
            // msg's priority
            Scheduler_node *&prio_end_ptr = prio_level_end_ptr[temp_ptr->priority];

            // remove this element from list if it's in list
            if (temp_ptr->prev)
            {
                temp_ptr->prev->next = temp_ptr->next;
            }
            if (temp_ptr->next)
            {
                temp_ptr->next->prev = temp_ptr->prev;
            }

            // if it's the starting pointer, move the starting pointer to the next
            if (temp_ptr == start_ptr)
            {
                start_ptr = temp_ptr->next;
                // if it's also the last message of this priority, it must be
                // the last, remote this priority level!
                if (temp_ptr == prio_end_ptr)
                {
                    prio_end_ptr = nullptr;
                }
            }
            // update the ending pointer of this category if this is the last
            // message of this priority
            else if (temp_ptr == prio_end_ptr)
            {
                // if this is not the only message left at this priority level
                // shift the level end pointer to the previous message
                if (temp_ptr->priority == (temp_ptr->prev->priority))
                {
                    prio_end_ptr = temp_ptr->prev;
                    // if this is, then remove this priority level
                }
                else
                {
                    prio_end_ptr = nullptr;
                }
            }

            // remove its own prev and next
            temp_ptr->prev = temp_ptr->next = nullptr;
        }

        void Add_to_schedule(const uint32_t type, const std::vector<uint16_t> &raw, const uint32_t priority1, const int32_t expiration_count, const uint32_t period)
        {
            // setup flag to skip the interrupt (interrupt still active, but
            // will not transmit any data)
            TX_enable_flag = false;
            // if current task is in operation, remove it first.
            Remove_from_schedule(type);
            // enable transmission again when preparing data
            TX_enable_flag = true;

            auto &nlt = node_list[type];
            // setup new schedule
            nlt.priority = priority1;
            nlt.expiration_counter = expiration_count;
            nlt.transmission_counter_max = period;
            nlt.transmission_counter = 0;
            nlt.val.TX_update(raw);

            // disable flag again when adding this node back
            TX_enable_flag = false;
            // add it back to the list
            Add_back_to_schedule(type);
            // reset flag to enable regular interrupt
            TX_enable_flag = true;
        }
    } // namespace TX
} // namespace IR