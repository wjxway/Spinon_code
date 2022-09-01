#include "FeedTheDog.hpp"
#include "FastIO.hpp"
#include "DataTransmission.hpp"
#include "Tasks.hpp"
#include "soc/timer_group_reg.h"
#include "soc/timer_group_struct.h"

#include <SPI.h>

#include <string>

#include "Prints.hpp"
#include "hal/rmt_ll.h"

uint64_t rec_finish_time = 0;

// Blink the LED
void blink_led(int n)
{
    for (int i = 0; i < n; i++)
    {
        digitalWrite(LED_PIN_1, LOW);
        digitalWrite(LED_PIN_2, LOW);
        digitalWrite(LED_PIN_3, LOW);

        delay(100);

        digitalWrite(LED_PIN_1, HIGH);
        digitalWrite(LED_PIN_2, HIGH);
        digitalWrite(LED_PIN_3, HIGH);

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
            digitalWrite(LED_PIN_1, HIGH);
            digitalWrite(LED_PIN_2, HIGH);
            digitalWrite(LED_PIN_3, HIGH);
        }
        delayMicroseconds(50);
    }
}

void setup()
{
    Serial.begin(115200);
    INIT_println("Init start...");

    // hspi
    hspi = new SPIClass(HSPI);
    hspi->begin();
    hspi->beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));

    // provide another ground
    pinMode(27, OUTPUT);
    digitalWrite(27, LOW);

    pinMode(HSS_PIN, OUTPUT);
    setbit(HSS_PIN);

    // test output
    pinMode(TEST_PIN, OUTPUT);
    pinMode(TEST_PIN_2, OUTPUT);

    pinMode(LED_PIN_1, OUTPUT);
    pinMode(LED_PIN_2, OUTPUT);
    pinMode(LED_PIN_3, OUTPUT);

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
    RMT_RX_TX::TX_prep_1->TX_load(std::vector<uint8_t>{THIS_ROBOT_ID + 1, THIS_ROBOT_ID + 2, THIS_ROBOT_ID + 3, THIS_ROBOT_ID + 4}, 3, detail::RMT_ticks_num);

#if RMT_TX_CHANNEL_COUNT > 1
    RMT_RX_TX::TX_prep_2->TX_load(std::vector<uint8_t>{THIS_ROBOT_ID + 1, THIS_ROBOT_ID + 2, THIS_ROBOT_ID + 3, THIS_ROBOT_ID + 4}, 1, 2 * detail::RMT_ticks_num);
#endif

    RMT_RX_TX::RMT_TX_resume();
    INIT_println("RMT TX setup finished!");
#endif

    // send RX data through SPI
#if RMT_RX_CHANNEL_COUNT
    // setup hspi mutex
    HSPI_MUTEX = xSemaphoreCreateMutex();

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

    INIT_println("RMT RX setup finished!");
#endif

    INIT_println("Init end...");
}

// do nothing in loop
void loop()
{
    vTaskDelay(portMAX_DELAY);
}
