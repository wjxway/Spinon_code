#include "IrTX.hpp"
#include "RobotDefs.hpp"
#include "RMTMessageDefs.hpp"
#include "RMTCoding.hpp"
#include "Utilities\CRCCheck.hpp"
#include "Utilities\FastIO.hpp"
#include "Utilities\DebugDefs.hpp"
#include "bootloader_random.h"
#include "hal\rmt_ll.h"

namespace IR
{
    using namespace detail;

    namespace TX
    {
        // anonymous namespace
        namespace
        {
            /**
             * @brief trigger timer for Trigger_ISR
             */
            hw_timer_t *Trigger_timer;

            /**
             * @brief Trigger_semaphore is given by Trigger_ISR to Feed_data_tasks
             */
            SemaphoreHandle_t Trigger_semaphore;

            /**
             * @brief Trigger_semaphore is given by Feed_data_tasks to Transmit_task
             */
            SemaphoreHandle_t Data_semaphore;

            /**
             * @brief buffer of data
             */
            rmt_item32_t data_buffer[RMT_TX_length];

            /**
             * @brief delay of upper emitter in ticks
             */
            constexpr uint32_t CH2_delay = 1;

            /**
             * @brief final pulse duration of upper emitter
             */
            constexpr uint32_t CH2_final_pulse_duration = 2 * detail::RMT_ticks_num;

            /**
             * @brief generate the next triggering time
             *
             * @return uint32_t time in us.
             */
            inline uint32_t Generate_trigger_time()
            {
                return esp_random() % (RMT_TX_trigger_period_max - RMT_TX_trigger_period_min) + RMT_TX_trigger_period_min;
            }
        }

        void Init()
        {
            // enable random number generation
            bootloader_random_enable();

            // initialize semaphores
            Trigger_semaphore = xSemaphoreCreateBinary();
            Data_semaphore = xSemaphoreCreateBinary();

            // RMT peripheral start
#if RMT_TX_CHANNEL_COUNT
            pinMode(RMT_TX_PIN_1, OUTPUT);
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
                    Serial.println("TX_1 init failed!");)

            // disable interrupt
            rmt_set_tx_intr_en(RMT_TX_channel_1, false);
            rmt_set_tx_thr_intr_en(RMT_TX_channel_1, false, 1);

            DEBUG_C(Serial.println("TX_1 init successful!"));
#endif
// second emitter
#if RMT_TX_CHANNEL_COUNT > 1
            pinMode(RMT_TX_PIN_2, OUTPUT);
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
                    Serial.println("TX_2 init failed!");)

            // disable interrupt
            rmt_set_tx_intr_en(RMT_TX_channel_2, false);
            rmt_set_tx_thr_intr_en(RMT_TX_channel_2, false, 1);

            DEBUG_C(Serial.println("TX_2 init successful!"));
#endif
            // data transmit task
            // pinned to core 0, should be of really high priority, but get blocked almost all the time, only to released by data tasks.
            xTaskCreatePinnedToCore(
                Transmit_task,
                "Transmit_task",
                10000,
                NULL,
                20,
                NULL,
                0);

            // config timer to transmit TX once in a while
            // we are using timer 3 to prevent confiction
            // timer ticks 1MHz
            Trigger_timer = timerBegin(IR_TX_trigger_timer_channel, 80, true);
            // add timer interrupt
            timerAttachInterrupt(Trigger_timer, &Trigger_ISR, true);
            // trigger interrupt after some random time
            timerAlarmWrite(Trigger_timer, Generate_trigger_time(), false);
            timerAlarmEnable(Trigger_timer);
        }

        uint32_t temp = 0;

        void IRAM_ATTR Trigger_ISR()
        {
            // reset timer
            timerRestart(Trigger_timer);
            timerAlarmWrite(Trigger_timer, Generate_trigger_time(), false);
            timerAlarmEnable(Trigger_timer);
            // give semaphore
            // if no data task is present, this function will return false, but that's okay!
            xSemaphoreGiveFromISR(Trigger_semaphore, NULL);
        }

        void Transmit_task(void *pvParameters)
        {
            while (true)
            {
                // only activate when data and data semaphore is ready.
                xSemaphoreTake(Data_semaphore, portMAX_DELAY);

                // RMT TX
                // write channel 1 register
                rmt_ll_write_memory(&RMTMEM, RMT_TX_channel_1, data_buffer, RMT_TX_length, 0);
                rmt_ll_tx_reset_pointer(&RMT, RMT_TX_channel_1);

                // edit channel 2 data
                data_buffer[0].duration0 += CH2_delay;
                data_buffer[RMT_data_pulse_count].duration1 = CH2_final_pulse_duration;

                // write channel 2 register
                rmt_ll_write_memory(&RMTMEM, RMT_TX_channel_2, data_buffer, RMT_TX_length, 0);
                rmt_ll_tx_reset_pointer(&RMT, RMT_TX_channel_2);

                // start RMT as fast as possible by directly manipulating the registers
                RMT.conf_ch[RMT_TX_channel_1].conf1.tx_start = 1;
                // a tiny delay to sync the channels
                __asm__ __volatile__("nop;");
                RMT.conf_ch[RMT_TX_channel_2].conf1.tx_start = 1;
            }
        }

        TX_data::TX_data(const std::vector<uint32_t> &raw, const uint32_t msg_type1, const uint32_t priority, const int32_t expiration_count, const uint32_t cycle_per_transmission) : msg_type(msg_type1), Expiration_counter(expiration_count), transmission_counter(0), transmission_counter_max(cycle_per_transmission)
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
                        Serial.println("Feeding multiple transmission data into single transmission messages! Only the first data will be sent!");)
                // resize, generate RMT_item, and setup pointer
                data.resize(RMT_TX_length);
                Generate_RMT_item(data.data(), Msg_single_t{{Robot_ID, msg_type, msg_ID_init, crc4_itu(raw[0]), raw[0]}}.raw);

                ptr = data.data();
                end_ptr = ptr;
            }
            // multiple transmissions
            else
            {
                // setup container
                data.resize((n_msg + 1) * RMT_TX_length);

                // create header
                Generate_RMT_item(data.data(), Msg_header_t{{Robot_ID, msg_type, msg_ID_init, 0, n_msg, crc8_maxim(raw.data(), raw.size())}}.raw);

                // cycle through data and generate data
                for (uint32_t msgid = 1; msgid <= n_msg; msgid++)
                    Generate_RMT_item(data.data() + msgid * RMT_TX_length, Msg_t{{Robot_ID, msg_type, msg_ID_init, msgid, raw[msgid - 1]}}.raw);

                // setup pointers
                ptr = data.data();
                end_ptr = data.data() + n_msg * RMT_TX_length;
            }

            // change msg_ID_init for next data set
            msg_ID_init = (msg_ID_init ? 0 : 1);

            // data feed task
            // pinned to core 0, should be of really high priority, but get blocked almost all the time, only to released by trigger tasks.
            xTaskCreatePinnedToCore(
                static_Feed_data_task,
                "Feed_data_task",
                10000,
                this,
                priority,
                Feed_data_task_handler,
                0);
        }

        void TX_data::Enable_task()
        {
            vTaskResume(Feed_data_task_handler);
        }

        void TX_data::Disable_task()
        {
            vTaskSuspend(Feed_data_task_handler);
        }

        void TX_data::Change_data(const std::vector<uint32_t> &raw)
        {
            // disable task before swapping it out
            Disable_task();

            // number of all messages - 1 (excluding header message)
            uint32_t n_msg = raw.size();

            // check msg_type
            // if single transmission
            if (msg_type <= Single_transmission_msg_type)
            {
                // notify user if msg_type is not proper.
                DEBUG_C(
                    if (n_msg > 1)
                        Serial.println("Feeding multiple transmission data into single transmission messages! Only the first data will be sent!");)
                // resize, generate RMT_item, and setup pointer
                data.resize(RMT_TX_length);
                Generate_RMT_item(data.data(), Msg_single_t{{Robot_ID, msg_type, msg_ID_init, crc4_itu(raw[0]), raw[0]}}.raw);
                ptr = data.data();
                end_ptr = ptr;
            }
            // multiple transmissions
            else
            {
                // setup container
                data.resize((n_msg + 1) * RMT_TX_length);

                // create header
                Generate_RMT_item(data.data(), Msg_header_t{{Robot_ID, msg_type, msg_ID_init, 0, n_msg, crc8_maxim(raw.data(), raw.size())}}.raw);

                // cycle through data and generate data
                for (uint32_t msgid = 1; msgid <= n_msg; msgid++)
                    Generate_RMT_item(data.data() + msgid * RMT_TX_length, Msg_t{{Robot_ID, msg_type, msg_ID_init, msgid, raw[msgid - 1]}}.raw);

                // change msg_ID_init for next data set
                msg_ID_init = (msg_ID_init ? 0 : 1);

                // change pointer
                ptr = data.data();
                end_ptr = data.data() + n_msg * RMT_TX_length;
            }

            // resume
            Enable_task();
        }

        void TX_data::Change_expiration(int32_t expiration_count)
        {
            Expiration_counter = expiration_count;
        }

        void TX_data::Change_frequency(const int32_t cycle_per_transmission)
        {
            transmission_counter_max = cycle_per_transmission;
            transmission_counter = min(transmission_counter, transmission_counter_max);
        }

        void TX_data::Change_priority(const int32_t priority)
        {
            vTaskPrioritySet(Feed_data_task_handler, priority);
        }

        TX_data::~TX_data()
        {
            // just remove this task as its content won't exist anymore
            vTaskDelete(Feed_data_task_handler);
        }

        void TX_data::Feed_data_task(void *pvParameters)
        {
            while (true)
            {
                // only setup data when triggered
                xSemaphoreTake(Trigger_semaphore, portMAX_DELAY);

                // if transmission counter is not 0, then we don't want to transmit this time.
                // give the semaphore to someone else!
                if (transmission_counter)
                {
                    transmission_counter--;
                    // give this chance to others
                    xSemaphoreGive(Trigger_semaphore);
                }
                // if my turn
                else
                {
                    // reset the transmission counter
                    transmission_counter = transmission_counter_max;

                    // copy data
                    std::copy(ptr, ptr + RMT_TX_length, data_buffer);

                    // increment pointer
                    if (ptr == end_ptr)
                    {
                        // one full transmission has been made
                        if (Expiration_counter != -1)
                            Expiration_counter--;
                        ptr = data.data();
                    }
                    else
                        ptr += RMT_TX_length;

                    // give semaphore to Transmit task to enable transmission
                    xSemaphoreGive(Data_semaphore);

                    // disable task if it has expired
                    if (Expiration_counter == 0)
                        Disable_task();
                }
            }
        }

        void TX_data::static_Feed_data_task(void *pvParameter)
        {
            TX_data *ptr = reinterpret_cast<TX_data *>(pvParameter); // obtain the instance pointer
            ptr->Feed_data_task(NULL);                               // dispatch to the member function, now that we have an instance pointer
        }
    }
}