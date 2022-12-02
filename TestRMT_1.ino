#include <string>

#include "src/Utilities/FeedTheDog.hpp"
#include "src/Utilities/DebugDefs.hpp"
#include "src/Utilities/FastIO.hpp"
#include "src/MotorCtrl/MotorCtrl.hpp"
#include "src/IrCommunication/IrCommunication.hpp"
#include "src/Tasks.hpp"

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

    blink_led(5);

    DEBUG_C(Serial.println("Pin setup finished"));

    // Motor::Init();
    // Motor::Set_speed(30);

    // DEBUG_C(Serial.println("Motor started"));

    // IR::TX::Init();

    // DEBUG_C(Serial.println("TX inited"));

    // // this is the data task that has type 4 and transmit {0x1234, 0xFEDC}
    // IR::TX::Add_to_schedule(4, {0x0123, 0xFEDC}, 2, -1, 2);

    // DEBUG_C(Serial.println("TX data set"));

    IR::RX::Init();

    DEBUG_C(Serial.println("RX inited"));

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

    // quench LED!
    xTaskCreatePinnedToCore(
        LED_off_task,
        "LED_off_task",
        20000,
        NULL,
        2,
        NULL,
        0);

    // send me messages through serial!
    xTaskCreatePinnedToCore(
        Send_message_task,
        "sendmsgtask",
        20000,
        NULL,
        3,
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
