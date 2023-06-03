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
    uint16_t raw[8];
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
            temp_len = IR::RX::Get_timing_data(temp, min(esp_timer_get_time() - last_access_time + 1000, 50000LL));

        } while (io_flag != IR::RX::Get_io_flag());

        if (temp_len != 0)
        {
            int64_t last_access_time_new = max(max(temp[0].time_arr[0], temp[0].time_arr[1]), temp[0].time_arr[2]);
            if (last_access_time_new > last_access_time)
            {
                LIT_G;
                delay(1);
                QUENCH_G;

                timing_short ts;
                ts.val.rid = temp[0].robot_ID;
                ts.val.time[0] = temp[0].time_arr[0];
                ts.val.time[1] = temp[0].time_arr[1];
                ts.val.time[2] = temp[0].time_arr[2];

                std::vector<uint16_t> data(8, 0);
                for (size_t i = 0; i < 8; i++)
                {
                    data[i] = ts.raw[i];
                }

                IR::TX::Add_to_schedule(5, data, 2);
                last_access_time = last_access_time_new;
            }
        }
    }
}

void Message_relay_task(void *pvParameters)
{
    Circbuffer_copycat<IR::RX::Parsed_msg_completed, 20> msg_buf = IR::RX::Get_msg_buffer_by_type(5);

    while (true)
    {
        // wait till next data is obtained
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (!msg_buf.Empty_Q())
        {
            auto msg = msg_buf.pop();
            if (msg.content_length != 8)
            {
                continue;
            }

            timing_short ts;
            for (size_t i = 0; i < 8; i++)
            {
                ts.raw[i] = msg.content[i];
            }

            std::string str = "";
            str += "ID:" + std::to_string(ts.val.rid) + ",t_mid:" + std::to_string(ts.val.time[0]) + ",t_left:" + std::to_string(ts.val.time[1]) + ",t_right:" + std::to_string(ts.val.time[2]) + "\n";

            // std::string str = "";
            // for (size_t i = 0; i < msg.content_length; i++)
            // {
            //     str += "," + std::to_string(i) + ":" + std::to_string(msg.content[i]);
            // }
            // str += "\n";

            Serial.print(str.c_str());
        }
    }
}