#include "hal/rmt_ll.h"
#include "FastIO.h"
#include "FeedTheDog.h"
#include "DataTransmission.h"
#include "soc/timer_group_reg.h"
#include "soc/timer_group_struct.h"

#include <string>

// RMT pins def
#define RMT_OUT 25
#define RMT_IN_1 26
#define RMT_IN_2 27

// test signal
#define TEST_PIN 18

#define LED_PIN_1 16
#define LED_PIN_2 17
#define LED_PIN_3 19

// test timing
uint64_t rec_finish_time = 0;

#define TO_GPIO(ch) PRIMITIVE_CAT(GPIO_NUM_, ch)

// RMT channels def
constexpr rmt_channel_t RMT_TX_CHANNEL = RMT_CHANNEL_0;
constexpr rmt_channel_t RMT_RX_CHANNEL_1 = RMT_CHANNEL_2;
constexpr rmt_channel_t RMT_RX_CHANNEL_2 = RMT_CHANNEL_4;

// Custom ISR for RX_1
// change the channel number in conf_ch to adapt to other RMT channels.
//   1. record time
//   2. record data
//   3. notify the user that a new message has come in.(In test, this is a set & unset pin command)
rmt_isr_handle_t xHandler = NULL;

RX_data_fragments_pool pool;

// interrupt trigger time
uint64_t trig_time = 0;

void IRAM_ATTR rmt_isr_handler(void *arg)
{
    // // test start pulse
    // delayhigh100ns(TEST_PIN);
    // clrbit(TEST_PIN);

    // delay for a bit to make sure all RMT channels finished receiving
    // I assume a identical signal's delay won't vary by 500ns which is half the RMT pulse width.
    // It is highly unlikely that two valid yet different signal received within such a short time is different.
    // If each cycle is 500us, that's a 1/1000 chance that two valid yet different signal will collide,
    // and this still haven't take into consideration of geometric configuration.
    delay100ns;
    delay100ns;
    delay100ns;
    // delay100ns;
    // delay100ns;

    // record time
    uint64_t rec_time;
    rec_time = micros();

    // read RMT interrupt status.
    uint32_t intr_st = RMT.int_st.val;

    // interrupt state, modified order
    uint32_t intr_st_1 = 0;
    // rmt item
    volatile rmt_item32_t *item_1, *item_2;
    // RMT_RX_1, corresponding to RMT channel 2
    if (intr_st & (1 << (1 + 3 * RMT_RX_CHANNEL_1)))
    {
        // change owner state, disable rx
        RMT.conf_ch[RMT_RX_CHANNEL_1].conf1.rx_en = 0;
        RMT.conf_ch[RMT_RX_CHANNEL_1].conf1.mem_owner = RMT_MEM_OWNER_TX;
        intr_st_1 += 1;

        // RMT storage pointer
        item_1 = RMTMEM.chan[RMT_RX_CHANNEL_1].data32;
    }
    // RMT_RX_1, corresponding to RMT channel 4
    // This channel has higher priority when two channels both received the signal.
    // So make sure that this is the channel that received the signal later.
    // Because the channel receiving the signal later will have higher received intensity.
    if (intr_st & (1 << (1 + 3 * RMT_RX_CHANNEL_2)))
    {
        // change owner state, disable rx
        RMT.conf_ch[RMT_RX_CHANNEL_2].conf1.rx_en = 0;
        RMT.conf_ch[RMT_RX_CHANNEL_2].conf1.mem_owner = RMT_MEM_OWNER_TX;
        intr_st_1 += 2;

        // RMT storage pointer
        item_2 = RMTMEM.chan[RMT_RX_CHANNEL_2].data32;
    }

    // parsing output buffer
    uint32_t raw;
    // parse RMT item into a uint32_t
    if ((intr_st_1 & 2) && Parse_RMT_item(item_2, &raw))
    {
        clrbit(LED_PIN_2);
        if(intr_st_1 & 1)
            clrbit(LED_PIN_1);

        trig_time = micros();

        // add element to the pool if parsing is successful
        pool.Add_element(raw, intr_st_1, rec_time);
    }
    else if ((intr_st_1 & 1) && Parse_RMT_item(item_1, &raw))
    {
        clrbit(LED_PIN_1);

        trig_time = micros();
        // add element to the pool if parsing is successful
        pool.Add_element(raw, intr_st_1, rec_time);
    }

    // print & reset when data complete
    if (pool.Data_valid_q() && !rec_finish_time)
    {
#if _DEBUG_PRINT_
        std::string str;
        // data
        for (uint8_t i = 0; i < (pool.Msg_max); i++)
            str += std::to_string(pool.pool[i]) + std::string(",");
        Serial.println(str.data());
        // timing
        str = "";
        str += std::to_string(pool.Time_valid_q()) + std::string(":");
        for (uint8_t i = 0; i < N_RECEIVERS; i++)
            str += std::to_string(pool.first_message_time[i]) + std::string(" ");
        Serial.println(str.data());
#endif

        // setup the time when data is fully captured.
        rec_finish_time = micros();

        // // reset the pool
        // pool = RX_data_fragments_pool{};
    }

    // reset memory and owner state, enable rx
    if (intr_st_1 & 1)
    {
        RMT.conf_ch[RMT_RX_CHANNEL_1].conf1.mem_wr_rst = 1;
        RMT.conf_ch[RMT_RX_CHANNEL_1].conf1.mem_owner = RMT_MEM_OWNER_RX;
        RMT.conf_ch[RMT_RX_CHANNEL_1].conf1.rx_en = 1;
    }
    if (intr_st_1 & 2)
    {
        RMT.conf_ch[RMT_RX_CHANNEL_2].conf1.mem_wr_rst = 1;
        RMT.conf_ch[RMT_RX_CHANNEL_2].conf1.mem_owner = RMT_MEM_OWNER_RX;
        RMT.conf_ch[RMT_RX_CHANNEL_2].conf1.rx_en = 1;
    }

    // clear RMT interrupt status.
    RMT.int_clr.val = intr_st;

    // delay for a bit so that the interrupt status could be cleared
    delay100ns;
    delay50ns;

    // // test end pulse
    // delayhigh500ns(TEST_PIN);
    // clrbit(TEST_PIN);
}

RMT_TX_prep *tx_prep;

volatile int interruptCounter;
hw_timer_t *RMT_TX_trigger_timer = NULL;

// trigger function for TX emission
void IRAM_ATTR RMT_TX_trigger()
{
    while (!(tx_prep->output_ready_flag))
    {
    }
    // start emission
    rmt_ll_write_memory(&RMTMEM, RMT_TX_CHANNEL, tx_prep->Get_TX_pointer(), RMT_TX_length, 0);
    rmt_ll_tx_reset_pointer(&RMT, RMT_TX_CHANNEL);
    rmt_ll_tx_start(&RMT, RMT_TX_CHANNEL);
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Init start...");
    // RMT output and input
    pinMode(RMT_OUT, OUTPUT);
    pinMode(RMT_IN_1, INPUT);
    pinMode(RMT_IN_2, INPUT);
    // test output
    pinMode(TEST_PIN, OUTPUT);
    pinMode(LED_PIN_1, OUTPUT);
    pinMode(LED_PIN_2, OUTPUT);
    pinMode(LED_PIN_3, OUTPUT);

    setbit(LED_PIN_1);
    setbit(LED_PIN_2);
    setbit(LED_PIN_3);

    Serial.println("Init end...");

    // config TX
    rmt_config_t rmt_tx;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = TO_GPIO(RMT_OUT);
    rmt_tx.clk_div = RMT_clock_div;
    rmt_tx.mem_block_num = 2;
    rmt_tx.rmt_mode = RMT_MODE_TX;
    rmt_tx.tx_config.loop_en = false;
    // We modulate our own signal, instead of using carrier!
    rmt_tx.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
    rmt_tx.tx_config.carrier_en = 0;
    rmt_tx.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    rmt_tx.tx_config.idle_output_en = 1;
    rmt_tx.tx_config.loop_en = 0;
    // initialization of RMT
    if (rmt_config(&rmt_tx) != ESP_OK)
        Serial.println("RMT TX init failed!");

    // disable interrupt
    rmt_set_tx_intr_en(RMT_TX_CHANNEL, false);
    rmt_set_tx_thr_intr_en(RMT_TX_CHANNEL, false, 1);

    Serial.println("RMT TX init successful!");

    // config RMT_TX_prep
    // here we set robot's id to 5
    tx_prep = new RMT_TX_prep{5};
    // set raw data to contain 12 bytes
    // and process raw data
    std::vector<uint8_t> raw_input = {6, 1, 0, 14, 255, 128, 0, 0, 0, 123, 155, 127};
    if (tx_prep->TX_load(raw_input))
    {
        Serial.println("RMT TX data prep successful!");
        Serial.print("TX data size = ");
        Serial.println(tx_prep->output.size());
    }
    else
        Serial.println("RMT TX data prep failed!");

    // config timer to start TX
    // timer ticks 1MHz
    RMT_TX_trigger_timer = timerBegin(3, 80, true);
    // add timer interrupt
    timerAttachInterrupt(RMT_TX_trigger_timer, &RMT_TX_trigger, true);
    // trigger interrupt every 500us
    timerAlarmWrite(RMT_TX_trigger_timer, 500, true);
    timerAlarmEnable(RMT_TX_trigger_timer);

    Serial.println("RMT TX timer interrupt successful!");

    // // start emission
    // rmt_ll_write_memory(&RMTMEM, RMT_TX_CHANNEL, emit_patt, emit_patt_length, 0);
    // rmt_ll_tx_reset_pointer(&RMT, RMT_TX_CHANNEL);
    // rmt_ll_tx_start(&RMT, RMT_TX_CHANNEL);

    Serial.println("RMT Emission successful!");

    // config RX_1
    rmt_config_t rmt_rx_1;
    rmt_rx_1.channel = RMT_RX_CHANNEL_1;
    rmt_rx_1.gpio_num = TO_GPIO(RMT_IN_1);
    rmt_rx_1.clk_div = RMT_clock_div;
    rmt_rx_1.mem_block_num = 2;
    rmt_rx_1.rmt_mode = RMT_MODE_RX;
    // use 2 as filter to filter out pulses with less than 250ns
    rmt_rx_1.rx_config.filter_en = 1;
    rmt_rx_1.rx_config.filter_ticks_thresh = 2;
    // stop when idle for 4us could be reduced to 3us when necessary, but should be longer than 2us.
    rmt_rx_1.rx_config.idle_threshold = 3 * RMT_ticks_num;

    if (rmt_config(&rmt_rx_1) != ESP_OK)
        Serial.println("RMT RX_1 init failed!");
    // set up filter again...
    else if (rmt_set_rx_filter(rmt_rx_1.channel, true, RMT_clock_div * RMT_ticks_num / 2) != ESP_OK)
        Serial.println("RMT RX_1 set filter failed!");
    // initialization of RMT RX interrupt
    else if (rmt_set_rx_intr_en(rmt_rx_1.channel, true) != ESP_OK)
        Serial.println("RMT RX_1 interrupt not started!");
    // else if (rmt_isr_register(rmt_isr_handler, NULL, ESP_INTR_FLAG_LEVEL3, &xHandler) != ESP_OK)
    //     Serial.println("RMT RX_1 interrupt register failed!");
    // enable RMT_RX_1

    // config RX_2
    rmt_config_t rmt_rx_2;
    rmt_rx_2.channel = RMT_RX_CHANNEL_2;
    rmt_rx_2.gpio_num = TO_GPIO(RMT_IN_2);
    rmt_rx_2.clk_div = RMT_clock_div;
    rmt_rx_2.mem_block_num = 2;
    rmt_rx_2.rmt_mode = RMT_MODE_RX;
    // use 2 as filter to filter out pulses with less than 250ns
    rmt_rx_2.rx_config.filter_en = 1;
    rmt_rx_2.rx_config.filter_ticks_thresh = 2;
    // stop when idle for 4us could be reduced to 3us when necessary, but should be longer than 2us.
    rmt_rx_2.rx_config.idle_threshold = 3 * RMT_ticks_num;

    if (rmt_config(&rmt_rx_2) != ESP_OK)
        Serial.println("RMT RX_2 init failed!");
    // set up filter again...
    else if (rmt_set_rx_filter(rmt_rx_2.channel, true, RMT_clock_div * RMT_ticks_num / 2) != ESP_OK)
        Serial.println("RMT RX_2 set filter failed!");
    // initialization of RMT RX interrupt
    else if (rmt_set_rx_intr_en(rmt_rx_2.channel, true) != ESP_OK)
        Serial.println("RMT RX_2 interrupt not started!");

    // setup ISR
    else if (rmt_isr_register(rmt_isr_handler, NULL, ESP_INTR_FLAG_LEVEL3, &xHandler) != ESP_OK)
        Serial.println("RMT RX interrupt register failed!");

    // enable RMT_RX
    else if (rmt_rx_start(rmt_rx_1.channel, 1) != ESP_OK)
        Serial.println("RMT RX_1 start failed!");
    else if (rmt_rx_start(rmt_rx_2.channel, 1) != ESP_OK)
        Serial.println("RMT RX_2 start failed!");
    else
        Serial.println("RMT RX setup finished!");
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

    static uint64_t t_delay = 100000, t_delay_1 = 50000, t_on = 20000, t_LED_off = 750;
    uint64_t t_now = micros();

    // turn off led
    if (t_now - trig_time > t_LED_off)
    {
        setbit(LED_PIN_1);
        setbit(LED_PIN_2);
    }

    // on or off
    static bool time_on = false;

    // starting to receive time
    if (pool.Time_valid_q())
    {
        //clrbit(LED_PIN_3);

        // green and then red, red 27
        uint64_t avg_time = 0.5 * (pool.first_message_time[0] + pool.first_message_time[1]);
        // uint64_t t_distance = abs(50000.0 / (0.01 + sin(2 * PI * 0.000001 * (pool.first_message_time[1] - pool.first_message_time[0]))));

        uint64_t t_distance = 15 * my_diff(pool.first_message_time[1],pool.first_message_time[0]);

        // turn on the light after a while of receiving the first signal
        if (t_now >= avg_time + t_delay && t_now < avg_time + t_delay + t_distance)
        {
            time_on = true;
            clrbit(LED_PIN_3);
        }
        // and turn it off after a while (timing proportional to the distance)
        else if (time_on)
        {
            time_on = false;
            setbit(LED_PIN_3);
        }
    }
    // else
    //     setbit(LED_PIN_3);

    // on or off
    static bool finish_on = false;
    // data finished time
    if (t_now >= rec_finish_time + t_delay_1 && t_now <= rec_finish_time + t_on + t_delay_1)
    {
        finish_on = true;
        clrbit(LED_PIN_1);
        clrbit(LED_PIN_2);
        clrbit(LED_PIN_3);
    }
    else if (finish_on)
    {
        rec_finish_time = 0;
        finish_on = false;
        setbit(LED_PIN_1);
        setbit(LED_PIN_2);
        setbit(LED_PIN_3);

        //pool = RX_data_fragments_pool{};
    }
}