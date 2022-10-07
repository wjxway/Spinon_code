#include "DataTransmission.hpp"
#include "Tasks.hpp"
#include "soc\timer_group_reg.h"
#include "soc\timer_group_struct.h"

#include <SPI.h>

#include <string>

#include "Utilities\FeedTheDog.hpp"
#include "Utilities\DebugDefs.hpp"
#include "Utilities\FastIO.hpp"
#include "hal\rmt_ll.h"

#include "IrTX.hpp"

uint64_t rec_finish_time = 0;

// Blink the LED
void blink_led(int n)
{
    for (int i = 0; i < n; i++)
    {
        LIT_R;
        LIT_G;
        LIT_B;

        delay(100);

        QUENCH_R;
        QUENCH_G;
        QUENCH_B;

        delay(100);
    }
}

// /**
//  * @brief a FreeRTOS queue for storing timing data
//  */
// xQueueHandle RMT_RX_time_queue;

// /**
//  * @brief a FreeRTOS queue for storing real data
//  */
// xQueueHandle RMT_RX_data_queue;

// // a mutex to allow only one task to use spi at a time
// xSemaphoreHandle HSPI_MUTEX;

// void Time_comm_task(void *pvParameters)
// {
//     // buffer
//     uint64_t buffer[RMT_RX_CHANNEL_COUNT + 2] = {};
//     while (1)
//     {
// #if RMT_RX_CHANNEL_COUNT
//         // check queues
//         xQueueReceive(RMT_RX_TX::time_queue, (void *)buffer, portMAX_DELAY);

//         // use MUTEX to ensure that only one task can use hspi at a time
//         xSemaphoreTake(HSPI_MUTEX, portMAX_DELAY);
//         delaylow100ns(HSS);
//         // transfer the data out via spi
//         hspi->transfer((uint8_t *)buffer, sizeof(buffer));
//         delaylow100ns(HSS);
//         setbit(HSS);
//         xSemaphoreGive(HSPI_MUTEX);

// #else
//         // if no receiver, just delete the task.
//         vTaskDelete(NULL);
// #endif
//     }
// }

// void Data_comm_task(void *pvParameters)
// {
//     // buffer, just 10 bytes for test
//     uint8_t buffer[10] = {0};
//     while (1)
//     {
// #if RMT_RX_CHANNEL_COUNT
//         // check queues
//         xQueueReceive(RMT_RX_TX::data_queue, (void *)buffer, portMAX_DELAY);

//         // use MUTEX to ensure that only one task can use hspi at a time
//         xSemaphoreTake(HSPI_MUTEX, portMAX_DELAY);
//         delaylow100ns(HSS);
//         // transfer the data out via spi
//         hspi->transfer((uint8_t *)buffer, sizeof(buffer));
//         delaylow100ns(HSS);
//         setbit(HSS);
//         xSemaphoreGive(HSPI_MUTEX);

// #else
//         // if no receiver, just delete the task.
//         vTaskDelete(NULL);
// #endif
//     }
// }

void LED_off_task(void *pvParameters)
{
    while (1)
    {
        // still need to feed the dog
        Feed_the_dog();

        // turn off led if they haven't been refreshed for a while
        if (micros() - RMT_RX_TX::last_RX_time > 200)
        {
            QUENCH_R;
            QUENCH_G;
            QUENCH_B;
        }
        delayMicroseconds(50);
    }
}

void real_setup(void *pvParameters)
{
    Serial.begin(115200);
    DEBUG_C(Serial.println("Init start..."));

    // // hspi
    // hspi = new SPIClass(HSPI);
    // hspi->begin();
    // hspi->beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));

    // pinMode(HSS_PIN, OUTPUT);
    // setbit(HSS_PIN);

    // test output
    pinMode(DEBUG_PIN_1, OUTPUT);
    pinMode(DEBUG_PIN_2, OUTPUT);

    pinMode(LED_PIN_R, OUTPUT);
    pinMode(LED_PIN_G, OUTPUT);
    pinMode(LED_PIN_B, OUTPUT);

    blink_led(3);

    // // LED off task
    // xTaskCreatePinnedToCore(
    //     LED_off_task,
    //     "LED_off_task",
    //     1000,
    //     NULL,
    //     2,
    //     NULL,
    //     0);

    // initialize all
    RMT_RX_TX::RMT_init();

    // load TX data and begin TX
#if RMT_TX_CHANNEL_COUNT
    RMT_RX_TX::TX_prep_1->TX_load(std::vector<uint8_t>{Robot_ID + 1, Robot_ID + 2, Robot_ID + 3, Robot_ID + 4}, 3, detail::RMT_ticks_num);

#if RMT_TX_CHANNEL_COUNT > 1
    RMT_RX_TX::TX_prep_2->TX_load(std::vector<uint8_t>{Robot_ID + 1, Robot_ID + 2, Robot_ID + 3, Robot_ID + 4}, 1, 2 * detail::RMT_ticks_num);
#endif

    RMT_RX_TX::RMT_TX_resume();
    DEBUG_C(Serial.println("RMT TX setup finished!"));
#endif

    // send RX data through SPI
#if RMT_RX_CHANNEL_COUNT
    // // setup hspi mutex
    // HSPI_MUTEX = xSemaphoreCreateMutex();

    // time comm task
    xTaskCreatePinnedToCore(
        Time_comm_task,
        "Time_comm_task",
        10000,
        NULL,
        3,
        NULL,
        0);

    // data comm task
    xTaskCreatePinnedToCore(
        Data_comm_task,
        "Data_comm_task",
        10000,
        NULL,
        3,
        NULL,
        0);

    DEBUG_C(Serial.println("RMT RX setup finished!"));
#endif

    // monitor the performance of cores
    xTaskCreatePinnedToCore(
        Idle_stats_task,
        "idle0",
        10000,
        NULL,
        1,
        NULL,
        0);

    xTaskCreatePinnedToCore(
        Idle_stats_task,
        "idle1",
        10000,
        NULL,
        1,
        NULL,
        1);

    DEBUG_C(Serial.println("Init end..."));
}

void setup()
{
    xTaskCreatePinnedToCore(
        real_setup,
        "setup_task",
        10000,
        NULL,
        20,
        NULL,
        0);
}

// delete loop() immediately
void loop()
{
    vTaskDelete(NULL);
}
