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

namespace
{
    constexpr size_t Raw_time_buffer_max_size = 600U;
    Circbuffer<IR::RX::Msg_timing_t, Raw_time_buffer_max_size + 3> Raw_time_buffer;

    void Send_message_task(void *pvParameters)
    {
        while (true)
        {
            vTaskDelay(50);
            if (Serial.available())
            {
                delay(10);
                // deplete serial buffer
                while (Serial.available())
                {
                    Serial.read();
                }

                while (Raw_time_buffer.n_elem != 0)
                {
                    auto rtdat = Raw_time_buffer.pop();

                    std::string v = std::string("ID: ") + std::to_string(rtdat.robot_ID) + std::string(", t_mid: ") + std::to_string(rtdat.time_arr[0]) + std::string(", t_left: ") + std::to_string(rtdat.time_arr[1]) + std::string(", t_right: ") + std::to_string(rtdat.time_arr[2]) + "\n";

                    Serial.print(v.c_str());
                }
            }
        }
    }
}

void Buffer_raw_data_task(void *pvParameters)
{
    // send me messages through serial!
    xTaskCreatePinnedToCore(
        Send_message_task,
        "Send_message_task",
        20000,
        NULL,
        3,
        NULL,
        0);

    int64_t last_access_time = 0;

    while (true)
    {
        // wait till next localization is done
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

        for (size_t i = 0; i < temp_len; i++)
        {
            if (max(max(temp[i].time_arr[0], temp[i].time_arr[1]), temp[i].time_arr[2]) <= last_access_time)
            {
                break;
            }
            else
            {
                Raw_time_buffer.push(temp[i]);
            }
        }

        if (temp_len > 0)
        {
            last_access_time = max(max(max(temp[0].time_arr[0], temp[0].time_arr[1]), temp[0].time_arr[2]), last_access_time);
        }

        if (Raw_time_buffer.n_elem >= Raw_time_buffer_max_size)
        {
            LIT_G;
        }
    }
}
