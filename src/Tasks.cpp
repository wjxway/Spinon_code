#include "Tasks.hpp"
#include <RobotDefs.hpp>
#include <FastIO.hpp>
#include <FastMath.hpp>
#include <DebugDefs.hpp>
#include <FeedTheDog.hpp>
#include <Circbuffer.hpp>
#include <IrCommunication.hpp>
#include <iostream>
#include <string>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

using math::fast::square;

void LED_off_task(void *pvParameters)
{
    TickType_t prev_wake_time = xTaskGetTickCount();

    while (1)
    {
        // still need to feed the dog
        Feed_the_dog();

        // turn off led if they haven't been refreshed for a while
        if (esp_timer_get_time() - IR::RX::Get_last_RX_time() >= 300)
        {
            // QUENCH_R;
            // QUENCH_G;
            QUENCH_B;
        }

        vTaskDelayUntil(&prev_wake_time, pdMS_TO_TICKS(1));
    }
}

union timing_short
{
    struct
    {
        uint32_t time[3];
        uint16_t rid;
    } val;
    uint16_t raw[7];
};

void Buffer_raw_data_task(void *pvParameters)
{
    int64_t last_access_time = 0;

    while (true)
    {
        // wait till next message is received
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // get data
        uint32_t io_flag;

        IR::RX::Msg_timing_t temp[100];
        size_t temp_len;

        do
        {
            io_flag = IR::RX::Get_io_flag();

            // check if there's enough data to use
            temp_len = IR::RX::Get_timing_data(temp, esp_timer_get_time() - last_access_time + 1000);

        } while (io_flag != IR::RX::Get_io_flag());

        int64_t last_access_time_new = max(max(temp[0].time_arr[0], temp[0].time_arr[1]), temp[0].time_arr[2]);
        if (last_access_time_new > last_access_time)
        {
            timing_short ts;
            ts.val.rid = temp[0].robot_ID;
            ts.val.time[0] = temp[0].time_arr[0];
            ts.val.time[1] = temp[0].time_arr[1];
            ts.val.time[2] = temp[0].time_arr[2];

            std::vector<uint16_t> data(7, 0);
            for (size_t i = 0; i < 7; i++)
            {
                data[i] = ts.raw[i];
            }

            IR::TX::Add_to_schedule(5, data, 3);
            last_access_time = last_access_time_new;
        }
    }
}
