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
    Serial.print("\n----DATA START----\n");

    for (int i = 0; i < 1000; i++)
    {
        auto res = IR::Sense::Transmit_and_sense(i);

        std::string s = "Delay_ticks: ";
        s += std::to_string(i) + " , CH1: " + std::to_string(res[0]) + " , CH2: " + std::to_string(res[1]) + " , CH3: " + std::to_string(res[2]);

        Serial.println(s.c_str());

        Feed_the_dog();
    }

    Serial.print("\n----DATA START----\n");
}
