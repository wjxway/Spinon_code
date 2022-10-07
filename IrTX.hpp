// RMT_TX
#ifndef _IRTX_HPP_
#define _IRTX_HPP_
#pragma once

#include "Arduino.h"
#include "driver\rmt.h"
#include <vector>

namespace IR
{
    namespace TX
    {
        /**
         * @brief initialization of assets.
         */
        void Init();

        /**
         * @brief TX timer interrupt handler
         *
         * @note this ISR should be run on core 0, leaving core 1 for RX tasks exclusively anyways.
         *       this task will only give a binary semaphore, which signals that the hardware resource is available
         *       and then other data tasks will fight for the binary semaphore.
         *       trigger_isr will reset itself and fire based on timing.
         */
        void IRAM_ATTR Trigger_ISR();

        /**
         * @brief a task that interacts with the RMT hardware and transmits the data
         *
         * @note the selected data task will store the data to be sent at an anonymous variable
         *       and then give a semaphore to activate this task and actually send the message.
         *       please set the priority of this task to be as high as possible.
         */
        void Transmit_task(void *pvParameters);

        class TX_data
        {
        public:
            /**
             * @brief Construct a new tx data object with data and complete settings.
             *
             * @param raw raw data, each uint32_t should contain Msg_content_bits of data.
             * @param priority task's priority.
             * @param expiration_count how many complete transmissions before removal of task.
             * @param cycle_per_transmission how many time this task has to be called before an actual transmission.
             *
             * @note When constructing single transmission data, construct a vector out of it first.
             */
            TX_data(const std::vector<uint32_t> &raw, const uint32_t msg_type1, const uint32_t priority, const int32_t expiration_count = -1, const uint32_t cycle_per_transmission = 1);

            /**
             * @brief Enable the task associated with this data.
             */
            void Enable_task();

            /**
             * @brief Disable the task associated with this data.
             */
            void Disable_task();

            /**
             * @brief change the data!
             *
             * @param raw load raw data and convert them into rmt_item32_t objects.
             *            each uint32_t should contain Msg_content_bits of data.
             */
            void Change_data(const std::vector<uint32_t> &raw);

            /**
             * @brief change expiration counter
             *
             * @param expiration_count how many complete transmissions before removal of task.
             */
            void Change_expiration(const int32_t expiration_count);

            /**
             * @brief change transmission_counter_max to change frequency of transmission.
             *
             * @param cycle_per_transmission how many time this task has to be called before an actual transmission.
             */
            void Change_frequency(const int32_t cycle_per_transmission);

            /**
             * @brief change task priority.
             *
             * @param priority priority of task.
             */
            void Change_priority(const int32_t priority);

            /**
             * @brief Destroy the tx data object
             *
             * @note should stop and remove the task as well.
             */
            ~TX_data();

        private:
            /**
             * @brief rmt data preparing to be sent
             */
            std::vector<rmt_item32_t> data = {};

            /**
             * @brief type of msg. you are not allowed to change it except at initialization!
             */
            const uint32_t msg_type = 0;

            /**
             * @brief msg_id_init
             */
            uint32_t msg_ID_init = 0;

            /**
             * @brief a pointer to the next data to be sent
             */
            rmt_item32_t *ptr;

            /**
             * @brief end pointer that points to the starting of the last transmission!
             */
            rmt_item32_t *end_ptr;

            /**
             * @brief counter for expiration of data. when counter reached 0, this task shall be deleted.
             *
             * @note when it's -1, this task never expire!
             */
            int32_t Expiration_counter;
            /**
             * @brief counter for actual transmission. when counter reached 0, reset and transmission. when not 0, give the semaphore back.
             *
             */
            uint32_t transmission_counter = 0;
            /**
             * @brief transmission counter reset value
             */
            uint32_t transmission_counter_max;

            /**
             * @brief a task that takes semaphore from Trigger_ISR, and feed data to Transmit_task
             */
            void Feed_data_task(void *pvParameters);

            /**
             * @brief a static task as helper for task creation
             * @param pvParameter pass in "this" pointer
             */
            static void static_Feed_data_task(void *pvParameter);

            TaskHandle_t *Feed_data_task_handler = NULL;
        };
    }
}

#endif