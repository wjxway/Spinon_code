/**
 * @file TestRMT_1.ino
 * @brief main file
 */
#include <string>

#include <DebugDefs.hpp>
#include <FastIO.hpp>
#include <MotorCtrl.hpp>
#include <IrCommunication.hpp>
#include <Localization.hpp>
#include <BitCast.hpp>
#include "Tasks.hpp"

uint64_t rec_finish_time = 0;

// Blink the LED
void blink_led(int n)
{
    for (size_t i = 0; i < n; i++)
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

void real_setup(void *pvParameters)
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    DEBUG_C(Serial.println("Setup start"));
    DEBUG_C(Serial.print("Robot #"));
    DEBUG_C(Serial.println(This_robot_ID));

    // please keep it here! or the RMT might be buggy!
    delay(10);

    IR::TX::Init();

    DEBUG_C(Serial.println("TX inited"));

    // IR::TX::Add_to_schedule(1, {0U}, 2);
    // Serial.println("Thruster @ 0");

    // this is the data task that has type 4 and transmit the robot's position
    switch (This_robot_ID)
    {
    case 1:
        IR::TX::Add_to_schedule(4, {std::bit_cast<uint16_t>((int16_t)0), std::bit_cast<uint16_t>((int16_t)300), std::bit_cast<uint16_t>((int16_t)0)}, 2);
        break;
    case 2:
        IR::TX::Add_to_schedule(4, {std::bit_cast<uint16_t>((int16_t)260), std::bit_cast<uint16_t>((int16_t)-150), std::bit_cast<uint16_t>((int16_t)0)}, 2);
        break;
    case 3:
        IR::TX::Add_to_schedule(4, {std::bit_cast<uint16_t>((int16_t)-260), std::bit_cast<uint16_t>((int16_t)-150), std::bit_cast<uint16_t>((int16_t)0)}, 2);
        break;
    case 4:
        IR::TX::Add_to_schedule(4, {std::bit_cast<uint16_t>((int16_t)260), std::bit_cast<uint16_t>((int16_t)-450), std::bit_cast<uint16_t>((int16_t)0)}, 2);
        break;
    case 5:
        IR::TX::Add_to_schedule(4, {std::bit_cast<uint16_t>((int16_t)-260), std::bit_cast<uint16_t>((int16_t)-450), std::bit_cast<uint16_t>((int16_t)0)}, 2);
        break;
    default:
        Serial.println("This_robot_ID Error!");
        blink_led(20);
        break;
    }

    DEBUG_C(Serial.println("TX data set"));

    // // update thruster value through Serial
    // xTaskCreatePinnedToCore(
    //     Motor_TX_task,
    //     "Motor_TX_task",
    //     10000,
    //     NULL,
    //     5,
    //     NULL,
    //     0);

    DEBUG_C(Serial.println("Init finished"));

    // remove this task after use
    vTaskDelete(NULL);
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
