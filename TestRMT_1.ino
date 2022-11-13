#include <string>

#include "Utilities/FeedTheDog.hpp"
#include "Utilities/DebugDefs.hpp"
#include "Utilities/FastIO.hpp"
#include "MotorCtrl.hpp"
#include "IrTX.hpp"
#include "Tasks.hpp"

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

// void LED_off_task(void *pvParameters)
// {
//     while (1)
//     {
//         // still need to feed the dog
//         Feed_the_dog();

//         // turn off led if they haven't been refreshed for a while
//         if (micros() - RMT_RX_TX::last_RX_time > 200)
//         {
//             QUENCH_R;
//             QUENCH_G;
//             QUENCH_B;
//         }
//         delayMicroseconds(50);
//     }
// }

void real_setup(void *pvParameters)
{

    // put your setup code here, to run once:
    Serial.begin(115200);
    DEBUG_C(Serial.println("Setup start"));

    // test output
    pinMode(DEBUG_PIN_1, OUTPUT);
    pinMode(DEBUG_PIN_2, OUTPUT);

    // LED output
    pinMode(LED_PIN_R, OUTPUT);
    pinMode(LED_PIN_G, OUTPUT);
    pinMode(LED_PIN_B, OUTPUT);

    digitalWrite(LED_PIN_R, HIGH);
    digitalWrite(LED_PIN_G, HIGH);
    digitalWrite(LED_PIN_B, HIGH);

    // TX output
    pinMode(RMT_TX_PIN_1, OUTPUT);
    pinMode(RMT_TX_PIN_2, OUTPUT);
    digitalWrite(RMT_TX_PIN_1, LOW);
    digitalWrite(RMT_TX_PIN_2, LOW);

    // RX input
    pinMode(RMT_RX_PIN_1, INPUT);
    pinMode(RMT_RX_PIN_2, INPUT);
    pinMode(RMT_RX_PIN_3, INPUT);

    DEBUG_C(Serial.println("Pin setup finished"));

    Motor::Init();
    Motor::Set_speed(30);

    DEBUG_C(Serial.println("Motor started"));

    // IR::TX::Init();

    DEBUG_C(Serial.println("TX inited"));

    // this is the data task that has type 4 and transmit {0x1234, 0xFEDC}
    IR::TX::Add_to_schedule(4, {0x0123, 0xFEDC}, 2, -1, 2);

    DEBUG_C(Serial.println("TX data set"));

    DEBUG_C(Serial.println("Init finished"));

    // monitor the performance of cores
    xTaskCreatePinnedToCore(
        Idle_stats_task,
        "idle0",
        10000,
        NULL,
        1,
        NULL,
        0);

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
