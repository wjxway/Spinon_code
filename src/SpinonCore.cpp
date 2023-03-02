/**
 * @file TestRMT_1.ino
 * @brief main file
 */
#include <cstring>
#include <DebugDefs.hpp>
#include <FastIO.hpp>
#include <IrSense.hpp>
#include <FeedTheDog.hpp>

void setup()
{
    Serial.begin(115200);
    Serial.println("Init start!");

    IR::Sense::Init();

    Serial.println("Init finished!");
}

// delete loop() immediately
void loop()
{
    static int64_t tstart = esp_timer_get_time();

    while (esp_timer_get_time() - tstart < 2000)
        ;

    tstart += 2000;

    auto res = IR::Sense::Transmit_and_sense(7);

    std::string s = "CH1: ";
    s += std::to_string(res[0]) + " , CH2: " + std::to_string(res[1]) + " , CH3: " + std::to_string(res[2]);

    Serial.println(s.c_str());
    Serial.flush();

    Feed_the_dog();

    // while (esp_timer_get_time() - t_prev <= t_delay)
    // {
    //     t_prev += t_delay;

    //     auto res = IR::Sense::Transmit_and_sense(7);

    //     std::string s = std::string("CH1: ") + std::to_string(res[0]) + " , CH2: " + std::to_string(res[1]) + " , CH3: " + std::to_string(res[2]);

    //     Serial.println(s.c_str());
    //     Serial.flush();

    //     Feed_the_dog();
    // }

    // Serial.print("----DATA END----\n");
}
