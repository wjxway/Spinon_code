#include "FeedTheDog.h"
#include "FastIO.h"
#include "DataTransmission.h"
#include "soc/timer_group_reg.h"
#include "soc/timer_group_struct.h"

#include <string>

uint64_t rec_finish_time=0;

// Blink the LED
void blink_led(int n)
{
    for (int i = 0; i < n; i++)
    {
        clrbit(LED_PIN_1);
        clrbit(LED_PIN_2);
        clrbit(LED_PIN_3);

        delay(30);

        setbit(LED_PIN_1);
        setbit(LED_PIN_2);
        setbit(LED_PIN_3);

        delay(30);
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Init start...");
    
    // test output
    pinMode(TEST_PIN, OUTPUT);
    pinMode(LED_PIN_1, OUTPUT);
    pinMode(LED_PIN_2, OUTPUT);
    pinMode(LED_PIN_3, OUTPUT);

    blink_led(3);

    RMT_RX_TX::RMT_Init();
    
    Serial.println("Init end...");
}

uint64_t my_diff(uint64_t x, uint64_t y)
{
    if (x > y)
        return x - y;
    else
        return y - x;
}

// turn on light based on reception status.
void loop()
{
    FeedTheDog();

    static uint64_t t_delay = 70000, t_delay_1 = 60000, t_on = 5000, t_LED_off = 750;
    uint64_t t_now = micros();

    // timing blink

    // State of blinking
    // 0: already triggered
    // 1: awaiting emission
    // 2: emitting
    static byte time_blink_state = 0;

    // timing data
    static uint64_t avg_time = 0, t_distance = 0;

    // pool pointer
    RX_data_fragments_pool * poolptr=RMT_RX_TX::RX_prep->Get_pool(THIS_ROBOT_ID);

    // wait for new data when triggered
    if (!time_blink_state)
    {
        if (poolptr && poolptr->Time_valid_q())
        {
            // green and then red, red 27
            avg_time = 0.5 * (poolptr->first_message_time[0] + poolptr->first_message_time[1]);
            // uint64_t t_distance = abs(50000.0 / (0.01 + sin(2 * PI * 0.000001 * (pool.first_message_time[1] - pool.first_message_time[0]))));

            t_distance = 10 * my_diff(poolptr->first_message_time[1], poolptr->first_message_time[0]);

            time_blink_state = 1;
        }
    }
    else if (time_blink_state == 1)
    {
        if (t_now >= avg_time + t_delay)
        {
            clrbit(LED_PIN_3);

            time_blink_state = 2;
        }
    }
    else if (time_blink_state == 2)
    {
        if (t_now >= avg_time + t_delay + t_distance)
        {
            setbit(LED_PIN_3);

            // reset the pool
            *poolptr = RX_data_fragments_pool{};

            time_blink_state = 0;
        }
    }

    // data valid blink

    // State of blinking
    // 0: already triggered
    // 1: awaiting emission
    // 2: emitting
    static byte data_blink_state = 0;

    // timing data
    static uint64_t rec_finish_time_1 = 0;

    // wait for new data when triggered
    if (!data_blink_state)
    {
        if (rec_finish_time != 0)
        {
            // buffer the data
            rec_finish_time_1 = rec_finish_time;

            data_blink_state = 1;
        }
    }
    else if (data_blink_state == 1)
    {
        if (t_now >= rec_finish_time_1 + t_delay_1)
        {
            clrbit(LED_PIN_1);
            clrbit(LED_PIN_2);
            clrbit(LED_PIN_3);

            data_blink_state = 2;
        }
    }
    else if (data_blink_state == 2)
    {
        if (t_now >= rec_finish_time_1 + t_delay_1 + t_on)
        {
            setbit(LED_PIN_1);
            setbit(LED_PIN_2);
            setbit(LED_PIN_3);

            // reset the time
            rec_finish_time = 0;

            data_blink_state = 0;
        }
    }

    // turn off led
    if (t_now - RMT_RX_TX::last_message_time > t_LED_off && data_blink_state != 2)
    {
        setbit(LED_PIN_1);
        setbit(LED_PIN_2);
    }
}
