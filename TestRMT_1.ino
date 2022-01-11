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

    // whether this transmission is valid
    bool this_valid = false;

    // parse RMT item into a uint32_t
    if ((intr_st_1 & 2) && Parse_RMT_item(item_2, &raw))
    {
        clrbit(LED_PIN_2);
        if (intr_st_1 & 1)
            clrbit(LED_PIN_1);

        trig_time = micros();
        // add element to the pool if parsing is successful
        pool.Add_element(Trans_info{raw, intr_st_1, rec_time});

        this_valid = true;
    }
    else if ((intr_st_1 & 1) && Parse_RMT_item(item_1, &raw))
    {
        clrbit(LED_PIN_1);

        trig_time = micros();
        // add element to the pool if parsing is successful
        pool.Add_element(Trans_info{raw, intr_st_1, rec_time});

        this_valid = true;
    }

    // print & reset when data complete
    if (this_valid && pool.Data_valid_q() && !rec_finish_time)
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
    FeedTheDog();
    if(Temp_data_ready)
    {
        RMT.conf_ch[RMT_RX_CHANNEL_1].conf1.rx_en = 0;
        RMT.conf_ch[RMT_RX_CHANNEL_2].conf1.rx_en = 0;

        Serial.println("--START--");

        Serial.print("BUG @ ");
        Serial.println(bug_pos);

        for(int i=0;i<64;i++)
            Serial.println(Temp_data[i]);
        
        Serial.println("--END--");

        Temp_data_ready=false;

        RMT.conf_ch[RMT_RX_CHANNEL_1].conf1.mem_wr_rst = 1;
        RMT.conf_ch[RMT_RX_CHANNEL_2].conf1.mem_wr_rst = 1;
        RMT.conf_ch[RMT_RX_CHANNEL_1].conf1.rx_en = 1;
        RMT.conf_ch[RMT_RX_CHANNEL_2].conf1.rx_en = 1;
    }
}
