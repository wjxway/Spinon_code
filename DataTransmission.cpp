#include "DataTransmission.h"
#include "FastIO.h"
#include "Prints.h"
#include "hal/rmt_ll.h"

using namespace detail;

const uint8_t detail::RMT_data_length = Robot_ID_bits + Msg_ID_bits + Msg_content_bits;

uint8_t crc8_maxim(uint8_t *data, uint16_t length)
{
    uint8_t i;
    uint8_t crc = 0; // Initial value
    while (length--)
    {
        crc ^= *data++; // crc ^= *data; data++;
        for (i = 0; i < 8; i++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0x8C; // 0x8C = reverse 0x31
            else
                crc >>= 1;
        }
    }
    return crc;
}

/**
 * @brief convert a small uint8_t array to uint32_t.
 *
 * @param pointer start pointer for data.
 * @return uint32_t content field.
 */
inline uint32_t Construct_content(uint8_t *pointer)
{
    uint32_t res = 0;
    for (uint32_t i = 0; i < Msg_content_bytes; i++)
        res += (uint32_t(pointer[i]) << (i << 3));
    return res;
}

/**
 * @brief convert a uint32_t to a small uint8_t array.
 *
 * @param data input uint32_t data.
 * @param pointer fill data starting from this pointer.
 */
inline void Deconstruct_content(uint32_t data, uint8_t *pointer)
{
    for (uint32_t i = 0; i < Msg_content_bytes; i++)
        *(pointer + i) = ((data & (0xFF << (i << 3))) >> (i << 3));
}

rmt_item32_t *RMT_TX_prep::Get_TX_pointer()
{
    rmt_item32_t *temp;

    // check the data update state
    switch (output_ready_flag)
    {
    // no one is accessing the data.
    case 0:
        temp = output.data() + output_pos;
        break;
    // TX updating program is updating the header, so we output the first data.
    case 1:
        temp = output.data() + RMT_TX_length;
        break;
    // TX updating program is updating the data, so we output the header.
    case 2:
        temp = output.data();
        break;
    }

    // cycle output_pos
    output_pos += RMT_TX_length;
    if (output_pos >= output.size())
        output_pos = 0;

    return temp;
}

bool RMT_TX_prep::TX_load(std::vector<uint8_t> raw)
{
    // make sure that raw_length_1 is a multiplication of Msg_content_bytes
    if (raw.size() % Msg_content_bytes)
        return false;

    // number of all messages - 1 (excluding header message)
    uint32_t n_msg = raw.size() / Msg_content_bytes;
    // setup container
    output.assign((n_msg + 1) * RMT_TX_length, {{0}});
    // reset output pointer
    output_pos = 0;

    // set flag to 1 to prevent acessing of header
    output_ready_flag = 1;
    // create header
    Generate_RMT_item(output.data(), Msg_header_t{{robot_ID, msg_id_initial, 0, n_msg, crc8_maxim(raw.data(), raw.size())}}.raw);
    // set flat to 2 to prevent acessing of data
    output_ready_flag = 2;
    // cycle through data
    for (uint32_t msgid = 1; msgid <= n_msg; msgid++)
        Generate_RMT_item(output.data() + msgid * RMT_TX_length, Msg_t{{robot_ID, msg_id_initial, msgid, Construct_content(raw.data() + (msgid - 1) * Msg_content_bytes)}}.raw);
    // reset flag to enable normal acessing of header & data
    output_ready_flag = 0;

    // change msg_id_initial for next data set
    msg_id_initial = (msg_id_initial ? 0 : 1);

    return true;
}

void RX_data_fragments_pool::reset_pool(Trans_info info)
{
    // convert the raw message to Msg_t type
    Msg_t msg;
    msg.raw = info.data;

    // reset filling status
    filling_status = 0;

    // reset first message time only when it's a time out reset
    if ((info.time - last_message_time) > Timing_expire_time)
    {
        for (uint8_t i = 0; i < RMT_RX_CHANNEL_COUNT; i++)
        {
            if ((info.receiver >> i) & 1)
                first_message_time[i] = info.time;
            else
                first_message_time[i] = 0;
        }
        time_ready_status = info.receiver;
    }

    last_message_time = info.time;
    Msg_ID_init = msg.Msg_ID_init;
    data_valid_flag = false;

    // header
    if (msg.Msg_ID)
    {
        Msg_count = 1;
        Msg_max = 0;
        CRC = 0;

        filling_status = (uint64_t(1) << (msg.Msg_ID - 1));
        Deconstruct_content(msg.content, pool + (msg.Msg_ID - 1) * Msg_content_bytes);
    }
    else
    {
        Msg_header_t msg_hd;
        msg_hd.raw = info.data;

        Msg_count = 0;
        Msg_max = msg_hd.Msg_ID_len;
        CRC = msg_hd.CRC;
    }
}

bool RX_data_fragments_pool::Add_element(Trans_info info)
{
    // convert the raw message to Msg_t type
    Msg_t msg;
    msg.raw = info.data;

    // some helper criteria
    bool prev_filled = false, header_invalid = false;

    // data or header?
    if (msg.Msg_ID)
        // if is data and previously filled
        prev_filled = (filling_status >> (msg.Msg_ID - 1)) & 1;
    else
    {
        Msg_header_t msg_hd;
        msg_hd.raw = info.data;

        // 1. Msg_max inconsistent or CRC inconsistent
        // 2. previous data's Msg_ID > Msg_ID_len
        header_invalid = (Msg_max && (Msg_max != msg_hd.Msg_ID_len || CRC != msg_hd.CRC)) || (filling_status >= (uint64_t(1) << msg_hd.Msg_ID_len));

#ifdef _DEBUG_PRINT_ENABLE_
        if (header_invalid)
        {
            if (Msg_max && (Msg_max != msg_hd.Msg_ID_len || CRC != msg_hd.CRC))
            {

                Serial.println("Header inconsistent");
            }
            else if (filling_status >= (uint64_t(1) << msg_hd.Msg_ID_len))
            {
                Serial.println("Previous Msg_ID out of bounds");
            }
        }
#endif

        // if new header and valid, set it up anyway.
        if (!Msg_max && !header_invalid)
        {
            Msg_max = msg_hd.Msg_ID_len;
            CRC = msg_hd.CRC;
        }
    }

    // If the data is invalid, then reset the pool and consider this data as the first.
    // 0. no data
    // 1. Msg_ID_init not the same
    // 2. data has expired
    // 3. Msg_ID too big
    if ((!Msg_count && !Msg_max)                                                                            // 0. no data
        || msg.Msg_ID_init != Msg_ID_init                                                                   // 1. Msg_ID_init not the same
        || (info.time - last_message_time) > Timing_expire_time                                             // 2. data has expired
        || (Msg_max && msg.Msg_ID > Msg_max)                                                                // 3. Msg_ID too big
        || (prev_filled && (Construct_content(pool + (msg.Msg_ID - 1) * Msg_content_bytes) != msg.content)) // 4. data inconsistent
        || header_invalid)                                                                                  // 5. header invalid
    {
#ifdef _DEBUG_PRINT_ENABLE_
        if (!Msg_count && !Msg_max)
            Serial.println("New pool reset");
        else if (msg.Msg_ID_init != Msg_ID_init)
            Serial.println("Inconsistent Msg_ID_init reset");
        else if ((time - last_message_time) > fragments_pool_decay_time)
            Serial.println("Pool decayed reset");
        else if (Msg_max && msg.Msg_ID > Msg_max)
            Serial.println("Msg_ID out of bounds reset");
        else if (prev_filled && (Construct_content(pool + (msg.Msg_ID - 1) * Msg_content_bytes) != msg.content))
        {
            Serial.println("Msg content inconsistent reset");
            Serial.print("Old content: ");
            Serial.println(Construct_content(pool + (msg.Msg_ID - 1) * Msg_content_bytes));
            Serial.print("New content: ");
            Serial.println(msg.content);
        }
#endif

        // // test set pin
        // delayhigh100ns(18);
        // clrbit(18);

        reset_pool(info);
    }
    // valid data
    else
    {
        // if this is the first message from a receiver, record the time.
        // we could just input RMT.int_st.val, but then it would be hard to change the RMT channels without changing the library.
        uint32_t temp = (info.receiver & (~time_ready_status));
        for (uint8_t i = 0; i < RMT_RX_CHANNEL_COUNT; i++)
            if ((temp >> i) & 1)
                first_message_time[i] = info.time;

        // update time_ready_status
        time_ready_status = time_ready_status | info.receiver;

        // update last_message_time
        last_message_time = info.time;

        // new message?
        if (msg.Msg_ID && !prev_filled)
        {
            Msg_count++;
            // fill data
            Deconstruct_content(msg.content, pool + (msg.Msg_ID - 1) * Msg_content_bytes);
            // set filling
            filling_status += (uint64_t(1) << (msg.Msg_ID - 1));
        }

        // if data is full, and haven't been check, check validity
        if (!data_valid_flag && Msg_count == Msg_max)
        {
            // if CRC valid, then data is complete
            if (CRC == crc8_maxim(pool, Msg_max * Msg_content_bytes))
            {
                data_valid_flag = true;
                return 1;
            }
            // if not, then reset the pool
            else
            {
                DEBUG_println("CRC mismatch reset");
                reset_pool(info);
            }
        }
    }

    return false;
}

bool RX_data_fragments_pool::Time_valid_q()
{
    return (time_ready_status == ((1 << RMT_RX_CHANNEL_COUNT) - 1));
}

bool RX_data_fragments_pool::Data_valid_q()
{
    return data_valid_flag;
}

uint32_t RX_data_fragments_pool::get_Msg_max()
{
    return Msg_max;
}

uint32_t RX_data_fragments_pool::size()
{
    return Msg_max * Msg_content_bytes;
}

void RX_data_fragments_pool::reset_pool()
{
    filling_status = 0;
    time_ready_status = 0;
    data_valid_flag = false;
    Msg_count = 0;
    Msg_max = 0;
    CRC = 0;
    last_message_time = 0;
    Msg_ID_init = 0;
}

bool RMT_RX_prep::Parse_RX(Trans_info info)
{
    // mask for robot ID field.
    static const uint32_t ID_mask = (Msg_t{{((1 << Robot_ID_bits) - 1), 0, 0, 0}}).raw;

    // robot's ID
    uint32_t robot_id = (ID_mask & info.data);

    // search for the corresponding pool
    uint32_t data_pos = 0, first_zero = Max_robots_simultaneous;
    for (data_pos = 0; data_pos < Max_robots_simultaneous; data_pos++)
    {
        // record first empty position
        if (!robot_id_list[data_pos] && first_zero == Max_robots_simultaneous)
            first_zero = data_pos;
        // check if robot's id exists in the list
        else if (robot_id_list[data_pos] == robot_id)
            break;
    }

    // if it's a new robot
    if (data_pos == Max_robots_simultaneous)
    {
        // setup robot id
        robot_id_list[first_zero] = robot_id;
        // setup pool
        // cleaning up should be Delete_obsolete's job, which shouldn't be done inside an interrupt.
        // please make sure that the pool is cleared as well when robot_id_list is cleared.
        return pools[first_zero].Add_element(info);
    }
    // if it's a already existing one, just update the corresponding pool.
    else
        return pools[data_pos].Add_element(info);
}

RX_data_fragments_pool *RMT_RX_prep::Get_pool(uint32_t robot_id)
{
    // search for the corresponding pool
    uint32_t data_pos = 0;
    for (data_pos = 0; data_pos < Max_robots_simultaneous; data_pos++)
        // check if robot's id exists in the list
        if (robot_id_list[data_pos] == robot_id)
            break;

    // if it's never there
    if (data_pos == Max_robots_simultaneous)
        return nullptr;
    else
        return pools + data_pos;
}

uint32_t RMT_RX_prep::Count_neighbors()
{
    // search for the corresponding pool
    uint32_t data_count = 0;
    for (uint32_t data_pos = 0; data_pos < Max_robots_simultaneous; data_pos++)
        // check if robot's id exists in the list
        if (robot_id_list[data_pos])
            data_count++;

    return data_count;
}

std::vector<uint32_t> RMT_RX_prep::Get_neighbors_ID()
{
    std::vector<uint32_t> temp = {};
    temp.reserve(Max_robots_simultaneous);

    for (uint32_t data_pos = 0; data_pos < Max_robots_simultaneous; data_pos++)
        // check if robot's id exists in the list
        if (robot_id_list[data_pos])
            temp.push_back(robot_id_list[data_pos]);

    return temp;
}

void RMT_RX_prep::Delete_obsolete()
{
    for (uint32_t pos = 0; pos < Max_robots_simultaneous; pos++)
    {
        // if data has expired, reset the id list as well as the pool itself.
        if (micros() - pools[pos].last_message_time >= Data_expire_time)
        {
            robot_id_list[pos] = 0;
            pools[pos].reset_pool();
        }
    }
}

bool RMT_RX_TX::RMT_Init()
{
#if EMITTER_ENABLED
    // config TX
    rmt_config_t rmt_tx;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = (gpio_num_t)RMT_OUT;
    rmt_tx.clk_div = RMT_clock_div;
    // I set all mem_block_num to 2 because I can!
    // Might help prevent overflow errors.
    // Theoretically, this could be reduced to 1.
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
    {
        INIT_println("RMT TX init failed!");
        return false;
    }

    // disable interrupt
    rmt_set_tx_intr_en(RMT_TX_CHANNEL, false);
    rmt_set_tx_thr_intr_en(RMT_TX_CHANNEL, false, 1);

    INIT_println("RMT TX init successful!");

    // config RMT_TX_prep
    TX_prep = new RMT_TX_prep{THIS_ROBOT_ID};
    // set raw data to contain 2 bytes of 0.
    std::vector<uint8_t> raw_input = {0, 0};
    if (!(TX_prep->TX_load(raw_input)))
    {
        INIT_println("RMT TX data prep failed!");
        return false;
    }
    else
    {
        INIT_println("RMT TX data prep successful!");
        INIT_print("TX data size = ");
        INIT_println(TX_prep->output.size());
    }

    // config timer to transmit TX once in a while
    // we are using timer 3 to prevent confiction
    // timer ticks 1MHz
    RMT_TX_trigger_timer = timerBegin(RMT_TX_TRIGGER_TIMER_CHANNEL, 80, true);
    // add timer interrupt
    timerAttachInterrupt(RMT_TX_trigger_timer, &RMT_TX_trigger, true);
    // trigger interrupt every RMT_TX_TRIGGER_PERIOD us.
    timerAlarmWrite(RMT_TX_trigger_timer, RMT_TX_TRIGGER_PERIOD, true);
    timerAlarmEnable(RMT_TX_trigger_timer);

    INIT_println("RMT TX timer interrupt successful!");

    // // start emission
    // rmt_ll_write_memory(&RMTMEM, RMT_TX_CHANNEL, emit_patt, emit_patt_length, 0);
    // rmt_ll_tx_reset_pointer(&RMT, RMT_TX_CHANNEL);
    // rmt_ll_tx_start(&RMT, RMT_TX_CHANNEL);
    // Serial.println("RMT Emission successful!");
#endif

#if RMT_RX_CHANNEL_COUNT >= 1
    // RX is enabled
    RX_prep = new RMT_RX_prep{};

    // config RX_1
    rmt_config_t rmt_rx_1;
    rmt_rx_1.channel = RMT_RX_CHANNEL_1;
    rmt_rx_1.gpio_num = (gpio_num_t)RMT_IN_1;
    rmt_rx_1.clk_div = RMT_clock_div;
    rmt_rx_1.mem_block_num = 2;
    rmt_rx_1.rmt_mode = RMT_MODE_RX;
    // use 2 as filter to filter out pulses with less than 250ns
    rmt_rx_1.rx_config.filter_en = 1;
    rmt_rx_1.rx_config.filter_ticks_thresh = 2;
    // stop when idle for 4us could be reduced to 3us when necessary, but should be longer than 2us.
    rmt_rx_1.rx_config.idle_threshold = 3 * RMT_ticks_num;

    if (rmt_config(&rmt_rx_1) != ESP_OK)
    {
        INIT_println("RMT RX_1 init failed!");
        return false;
    }
    // set up filter again...
    if (rmt_set_rx_filter(rmt_rx_1.channel, true, RMT_clock_div * RMT_ticks_num / 2) != ESP_OK)
    {
        INIT_println("RMT RX_1 set filter failed!");
        return false;
    }
    // initialization of RMT RX interrupt
    if (rmt_set_rx_intr_en(rmt_rx_1.channel, true) != ESP_OK)
    {
        INIT_println("RMT RX_1 interrupt not started!");
        return false;
    }

#if RMT_RX_CHANNEL_COUNT >= 2
    // config RX_2
    rmt_config_t rmt_rx_2;
    rmt_rx_2.channel = RMT_RX_CHANNEL_2;
    rmt_rx_2.gpio_num = (gpio_num_t)RMT_IN_2;
    rmt_rx_2.clk_div = RMT_clock_div;
    rmt_rx_2.mem_block_num = 2;
    rmt_rx_2.rmt_mode = RMT_MODE_RX;
    rmt_rx_2.rx_config.filter_en = 1;
    rmt_rx_2.rx_config.filter_ticks_thresh = 2;
    rmt_rx_2.rx_config.idle_threshold = 3 * RMT_ticks_num;

    if (rmt_config(&rmt_rx_2) != ESP_OK)
    {
        INIT_println("RMT RX_2 init failed!");
        return false;
    }
    // set up filter again...
    if (rmt_set_rx_filter(rmt_rx_2.channel, true, RMT_clock_div * RMT_ticks_num / 2) != ESP_OK)
    {
        INIT_println("RMT RX_2 set filter failed!");
        return false;
    }
    // initialization of RMT RX interrupt
    if (rmt_set_rx_intr_en(rmt_rx_2.channel, true) != ESP_OK)
    {
        INIT_println("RMT RX_2 interrupt not started!");
        return false;
    }
#endif

#if RMT_RX_CHANNEL_COUNT == 3
    // config RX_3
    rmt_config_t rmt_rx_3;
    rmt_rx_3.channel = RMT_RX_CHANNEL_3;
    rmt_rx_3.gpio_num = (gpio_num_t)RMT_IN_3;
    rmt_rx_3.clk_div = RMT_clock_div;
    rmt_rx_3.mem_block_num = 2;
    rmt_rx_3.rmt_mode = RMT_MODE_RX;
    rmt_rx_3.rx_config.filter_en = 1;
    rmt_rx_3.rx_config.filter_ticks_thresh = 2;
    rmt_rx_3.rx_config.idle_threshold = 3 * RMT_ticks_num;

    if (rmt_config(&rmt_rx_3) != ESP_OK)
    {
        INIT_println("RMT RX_3 init failed!");
        return false;
    }
    // set up filter again...
    if (rmt_set_rx_filter(rmt_rx_3.channel, true, RMT_clock_div * RMT_ticks_num / 2) != ESP_OK)
    {
        INIT_println("RMT RX_3 set filter failed!");
        return false;
    }
    // initialization of RMT RX interrupt
    if (rmt_set_rx_intr_en(rmt_rx_3.channel, true) != ESP_OK)
    {
        INIT_println("RMT RX_3 interrupt not started!");
        return false;
    }
#endif
    // //enable default isr
    // else if (rmt_isr_register(rmt_isr_handler, NULL, ESP_INTR_FLAG_LEVEL3, &xHandler) != ESP_OK)
    // {
    //     INIT_println("RMT RX_1 interrupt register failed!");
    //     return false;
    // }

    // setup ISR
    if (rmt_isr_register(RMT_RX_ISR_handler, NULL, ESP_INTR_FLAG_LEVEL3, &xHandler) != ESP_OK)
    {
        INIT_println("RMT RX interrupt register failed!");
        return false;
    }

    // enable RMT_RX
    if (rmt_rx_start(rmt_rx_1.channel, 1) != ESP_OK)
    {
        INIT_println("RMT RX_1 start failed!");
        return false;
    }
#if RMT_RX_CHANNEL_COUNT >= 2
    if (rmt_rx_start(rmt_rx_2.channel, 1) != ESP_OK)
    {
        INIT_println("RMT RX_2 start failed!");
        return false;
    }
#endif
#if RMT_RX_CHANNEL_COUNT == 3
    if (rmt_rx_start(rmt_rx_3.channel, 1) != ESP_OK)
    {
        INIT_println("RMT RX_2 start failed!");
        return false;
    }
#endif
    INIT_println("RMT RX setup finished!");
#endif

    return true;
}

void IRAM_ATTR RMT_RX_TX::RMT_TX_trigger()
{
    rmt_ll_write_memory(&RMTMEM, RMT_TX_CHANNEL, TX_prep->Get_TX_pointer(), RMT_TX_length, 0);
    rmt_ll_tx_reset_pointer(&RMT, RMT_TX_CHANNEL);
    rmt_ll_tx_start(&RMT, RMT_TX_CHANNEL);
}