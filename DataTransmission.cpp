#include "DataTransmission.hpp"
#include "FastIO.hpp"
#include "Prints.hpp"
#include "hal/rmt_ll.h"

using namespace detail;

const uint32_t detail::RMT_data_length = Robot_ID_bits + Msg_ID_bits + Msg_content_bits;

// declaration of queues
xQueueHandle RMT_RX_TX::time_queue;
xQueueHandle RMT_RX_TX::data_queue;

/**
 * @brief CRC function using CRC-8/Maxim standard
 *
 * @param data input data pointer, should be in uint8_t form.
 * @param length input data length.
 * @return crc8 result as a int.
 *
 * @note copied from to https://github.com/whik/crc-lib-c
 *       could change to other crc functions for less collision chance when the message is longer.
 */
uint32_t crc8_maxim(uint8_t *data, uint32_t length)
{
    uint32_t i;
    uint32_t crc = 0; // Initial value
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
    // setup the lock
    // you should reset this externally using Reset_reader_lock()
    reader_count_flag++;

    rmt_item32_t *temp;

    // check the data update state
    switch (edit_state_flag)
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

void RMT_TX_prep::Reset_reader_lock()
{
    reader_count_flag--;
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
    edit_state_flag = 1;
    // delay a bit while so no confict will occur
    delay25ns;
    // lock
    while (reader_count_flag)
    {
    }
    // create header
    Generate_RMT_item(output.data(), Msg_header_t{{robot_ID, msg_ID_init, 0, n_msg, crc8_maxim(raw.data(), raw.size())}}.raw);
    // set flat to 2 to prevent acessing of data
    edit_state_flag = 2;
    delay25ns;
    while (reader_count_flag)
    {
    }
    // cycle through data
    for (uint32_t msgid = 1; msgid <= n_msg; msgid++)
        Generate_RMT_item(output.data() + msgid * RMT_TX_length, Msg_t{{robot_ID, msg_ID_init, msgid, Construct_content(raw.data() + (msgid - 1) * Msg_content_bytes)}}.raw);
    // reset flag to enable normal acessing of header & data
    edit_state_flag = 0;

    // change msg_ID_init for next data set
    msg_ID_init = (msg_ID_init ? 0 : 1);

    return true;
}

void RX_data_fragments_pool::Reset_pool_data(Trans_info info)
{
    // convert the raw message to Msg_t type
    Msg_t msg;
    msg.raw = info.data;

    // reset filling status
    filling_status = 0;

    msg_ID_init = msg.msg_ID_init;
    data_valid_flag = false;

    // header
    if (msg.msg_ID)
    {
        msg_count = 1;
        msg_ID_max = 0;
        CRC = 0;

        filling_status = (uint64_t(1) << (msg.msg_ID - 1));
        Deconstruct_content(msg.content, pool + (msg.msg_ID - 1) * Msg_content_bytes);
    }
    else
    {
        Msg_header_t msg_hd;
        msg_hd.raw = info.data;

        msg_count = 0;
        msg_ID_max = msg_hd.msg_ID_len;
        CRC = msg_hd.CRC;
    }
}

bool RX_data_fragments_pool::Add_element(Trans_info info)
{
    // Check timing validity and update timing information
    // Timing information should be independent to data.
    // Timing information and data information should be processed differently,
    // because it is possible for data to be reset while timing info should be kept. (changed data halfway in a emission)
    // and it is also possible for timing info to be reset while data should be kept. (another round)

    // Buffer the last_RX_time before resetting,
    // so the preceding data processing could check if data has expired.
    uint64_t last_RX_time_temp = last_RX_time;

    if ((info.time - last_RX_time) > Timing_expire_time)
    {
        for (uint32_t i = 0; i < RMT_RX_CHANNEL_COUNT; i++)
        {
            if ((info.receiver >> i) & 1)
                first_message_time[i] = info.time;
            else
                first_message_time[i] = 0;
        }
        time_ready_status = info.receiver;

        last_RX_time = info.time;
    }
    else
    {
        // if this is the first message from a receiver, record the time.
        // we could just input RMT.int_st.val, but then it would be hard to change the RMT channels without changing the library.
        uint32_t temp = (info.receiver & (~time_ready_status));
        for (uint32_t i = 0; i < RMT_RX_CHANNEL_COUNT; i++)
            if ((temp >> i) & 1)
                first_message_time[i] = info.time;

        // time valid status before this update
        bool valid_temp=Time_valid_q();
        // update time_ready_status
        time_ready_status = time_ready_status | info.receiver;

        // when timing is ready at this step, add it to queue.
        if(Time_valid_q()&&!valid_temp)
        {
            //set timeout to 0, so if it's lost, it's lost.
            xQueueSend(RMT_RX_TX::time_queue,(void *)first_message_time,0);
        }

        // update last_RX_time
        last_RX_time = info.time;
    }

    // Process the data first, here we only use timing information to check whether the data has expired.
    // Timing information and data information should be processed differently.
    // convert the raw message to Msg_t type
    Msg_t msg;
    msg.raw = info.data;

    // some helper criteria
    bool prev_filled = false, header_invalid = false;

    // data or header?
    if (msg.msg_ID)
        // if is data and previously filled
        prev_filled = (filling_status >> (msg.msg_ID - 1)) & 1;
    else
    {
        Msg_header_t msg_hd;
        msg_hd.raw = info.data;

        // 1. Msg_max inconsistent or CRC inconsistent
        // 2. previous data's Msg_ID > Msg_ID_len
        header_invalid = (msg_ID_max && (msg_ID_max != msg_hd.msg_ID_len || CRC != msg_hd.CRC)) || (filling_status >= (uint64_t(1) << msg_hd.msg_ID_len));

#if _DEBUG_PRINT_ENABLE_
        if (header_invalid)
        {
            if (msg_ID_max && (msg_ID_max != msg_hd.msg_ID_len || CRC != msg_hd.CRC))
            {

                Serial.println("Header inconsistent");
            }
            else if (filling_status >= (uint64_t(1) << msg_hd.msg_ID_len))
            {
                Serial.println("Previous Msg_ID out of bounds");
            }
        }
#endif

        // if new header and valid, set it up anyway.
        if (!msg_ID_max && !header_invalid)
        {
            msg_ID_max = msg_hd.msg_ID_len;
            CRC = msg_hd.CRC;
        }
    }

    // If the data is invalid, then reset the pool and consider this data as the first.
    // 0. no data
    // 1. msg_ID_init not the same
    // 2. data has expired
    // 3. msg_ID too big
    if ((!msg_count && !msg_ID_max)                                                                         // 0. no data
        || msg.msg_ID_init != msg_ID_init                                                                   // 1. msg_ID_init not the same
        || (info.time - last_RX_time_temp) > Data_expire_time                                               // 2. data has expired
        || (msg_ID_max && msg.msg_ID > msg_ID_max)                                                          // 3. msg_ID too big
        || (prev_filled && (Construct_content(pool + (msg.msg_ID - 1) * Msg_content_bytes) != msg.content)) // 4. data inconsistent
        || header_invalid)                                                                                  // 5. header invalid
    {
#if _DEBUG_PRINT_ENABLE_
        if (!msg_count && !msg_ID_max)
            Serial.println("New pool reset");
        else if (msg.msg_ID_init != msg_ID_init)
            Serial.println("Inconsistent Msg_ID_init reset");
        else if ((info.time - last_RX_time_temp) > Data_expire_time)
            Serial.println("Pool data expire reset");
        else if (msg_ID_max && msg.msg_ID > msg_ID_max)
            Serial.println("Msg_ID out of bounds reset");
        else if (prev_filled && (Construct_content(pool + (msg.msg_ID - 1) * Msg_content_bytes) != msg.content))
        {
            Serial.println("Msg content inconsistent reset");
            Serial.print("Old content: ");
            Serial.println(Construct_content(pool + (msg.msg_ID - 1) * Msg_content_bytes));
            Serial.print("New content: ");
            Serial.println(msg.content);
        }
#endif

        Reset_pool_data(info);
    }
    // valid data
    else
    {
        // new message?
        if (msg.msg_ID && !prev_filled)
        {
            msg_count++;
            // fill data
            Deconstruct_content(msg.content, pool + (msg.msg_ID - 1) * Msg_content_bytes);
            // set filling
            filling_status += (uint64_t(1) << (msg.msg_ID - 1));
        }

        // if data is full, and haven't been check, check validity
        if (!data_valid_flag && msg_count == msg_ID_max)
        {
            // if CRC valid, then data is complete
            if (CRC == crc8_maxim(pool, msg_ID_max * Msg_content_bytes))
            {
                data_valid_flag = true;

                // send data via queue
                // set timeout to 0, so if it's lost, it's lost.
                xQueueSend(RMT_RX_TX::data_queue,(void *)pool,0);

                return true;
            }
            // if not, then reset the pool
            else
            {
                DEBUG_println("CRC mismatch reset");
                Reset_pool_data(info);
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
    return msg_ID_max;
}

uint32_t RX_data_fragments_pool::size()
{
    return msg_ID_max * Msg_content_bytes;
}

void RX_data_fragments_pool::Reset_pool_all()
{
    filling_status = 0;
    time_ready_status = 0;
    data_valid_flag = false;
    msg_count = 0;
    msg_ID_max = 0;
    CRC = 0;
    last_RX_time = 0;
    msg_ID_init = 0;
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
        if (micros() - pools[pos].last_RX_time >= Data_expire_time)
        {
            robot_id_list[pos] = 0;
            pools[pos].Reset_pool_all();
        }
    }
}

// initial definitions
rmt_isr_handle_t RMT_RX_TX::xHandler = nullptr;
hw_timer_t *RMT_RX_TX::RMT_TX_trigger_timer = nullptr;
RMT_RX_prep *RMT_RX_TX::RX_prep = nullptr;
RMT_TX_prep *RMT_RX_TX::TX_prep = nullptr;
volatile uint64_t RMT_RX_TX::last_RX_time = 0;

bool RMT_RX_TX::RMT_init()
{
// RMT output and input
#if EMITTER_ENABLED
    pinMode(RMT_OUT, OUTPUT);
#endif
#if RMT_RX_CHANNEL_COUNT >= 1
    pinMode(RMT_IN_1, INPUT);
#endif
#if RMT_RX_CHANNEL_COUNT >= 2
    pinMode(RMT_IN_1, INPUT);
#endif
#if RMT_RX_CHANNEL_COUNT == 3
    pinMode(RMT_IN_1, INPUT);
#endif

#if EMITTER_ENABLED
    // config TX
    rmt_config_t rmt_tx;
    rmt_tx.channel = RMT_TX_channel;
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
    rmt_set_tx_intr_en(RMT_TX_channel, false);
    rmt_set_tx_thr_intr_en(RMT_TX_channel, false, 1);

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
    }

    // config timer to transmit TX once in a while
    // we are using timer 3 to prevent confiction
    // timer ticks 1MHz
    RMT_TX_trigger_timer = timerBegin(RMT_TX_trigger_timer_channel, 80, true);
    // add timer interrupt
    timerAttachInterrupt(RMT_TX_trigger_timer, &RMT_TX_trigger, true);
    // trigger interrupt every RMT_TX_trigger_period us.
    timerAlarmWrite(RMT_TX_trigger_timer, RMT_TX_trigger_period, true);
    // timerAlarmEnable(RMT_TX_trigger_timer);

    INIT_println("RMT TX timer interrupt successful!");

    // // start emission
    // rmt_ll_write_memory(&RMTMEM, RMT_TX_channel, emit_patt, emit_patt_length, 0);
    // rmt_ll_tx_reset_pointer(&RMT, RMT_TX_channel);
    // rmt_ll_tx_start(&RMT, RMT_TX_channel);
    // Serial.println("RMT Emission successful!");
#endif

#if RMT_RX_CHANNEL_COUNT >= 1
    // RX is enabled
    RX_prep = new RMT_RX_prep{};

    // initialize Queues
    // time_queue, for simplicity, just store all time data
    time_queue=xQueueCreate(100,sizeof(uint64_t)*RMT_RX_CHANNEL_COUNT);
    // for simplicity, in the tests we will just transmit the data's first 10 bytes (because test data won't exceed this length).
    // in reality, we also need to send the length of the data
    data_queue=xQueueCreate(100,sizeof(uint8_t)*10);

    // config RX_1
    rmt_config_t rmt_rx_1;
    rmt_rx_1.channel = RMT_RX_channel_1;
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
    rmt_rx_2.channel = RMT_RX_channel_2;
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
    rmt_rx_3.channel = RMT_RX_channel_3;
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
    //     INIT_println("RMT RX interrupt register failed!");
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
        INIT_println("RMT RX_3 start failed!");
        return false;
    }
#endif

    xTaskCreatePinnedToCore(
        RMT_RX_TX::RX_process_task,
        "RX_process_task",
        50000,
        NULL,
        5,
        NULL,
        0);

    INIT_println("RMT RX setup finished!");
#endif

    return true;
}

void IRAM_ATTR RMT_RX_TX::RMT_TX_trigger()
{
    rmt_ll_write_memory(&RMTMEM, RMT_TX_channel, TX_prep->Get_TX_pointer(), RMT_TX_length, 0);
    // reset the lock so other threads can continue to update the TX data
    TX_prep->Reset_reader_lock();
    rmt_ll_tx_reset_pointer(&RMT, RMT_TX_channel);
    rmt_ll_tx_start(&RMT, RMT_TX_channel);
}

void IRAM_ATTR RMT_RX_TX::RMT_RX_ISR_handler(void *arg)
{
    // // test start pulse
    delayhigh100ns(TEST_PIN);
    // clrbit(TEST_PIN);

    // delay for a bit to make sure all RMT channels finished receiving
    // I assume a identical signal's delay won't vary by 100ns which is more than half the RMT pulse width.
    // It is highly unlikely that two valid yet different signal received within such a short time is different.
    // If each cycle is 250us, that's a 1/1000 chance that two valid yet different signal will collide,
    // and this still haven't take into consideration of geometric configuration.
    delay100ns;

    // record time
    uint64_t rec_time;
    rec_time = micros();

    // read RMT interrupt status.
    uint32_t intr_st = RMT.int_st.val;
    // interrupt state, with order modified so that its 0,1,2 bits represent interrupt state for ch0, ch1, ch2.
    uint32_t intr_st_1 = 0;

    // rmt item
    volatile rmt_item32_t *item_1, *item_2, *item_3;
    // RMT_RX_1, corresponding to RMT channel 2
    // This channel has the highest priority when >1 channels received the signal.
    // So make sure that this is the channel that received the signal later.
    // Because the channel receiving the signal later will have higher received intensity.
    if (intr_st & (1 << (1 + 3 * RMT_RX_channel_1)))
    {
        // change owner state, disable rx
        RMT.conf_ch[RMT_RX_channel_1].conf1.rx_en = 0;
        RMT.conf_ch[RMT_RX_channel_1].conf1.mem_owner = RMT_MEM_OWNER_TX;
        intr_st_1 += 1;

        // RMT storage pointer
        item_1 = RMTMEM.chan[RMT_RX_channel_1].data32;
    }
#if RMT_RX_CHANNEL_COUNT >= 2
    // RMT_RX_2, corresponding to RMT channel 4
    if (intr_st & (1 << (1 + 3 * RMT_RX_channel_2)))
    {
        // change owner state, disable rx
        RMT.conf_ch[RMT_RX_channel_2].conf1.rx_en = 0;
        RMT.conf_ch[RMT_RX_channel_2].conf1.mem_owner = RMT_MEM_OWNER_TX;
        intr_st_1 += 2;

        // RMT storage pointer
        item_2 = RMTMEM.chan[RMT_RX_channel_2].data32;
    }
#endif
#if RMT_RX_CHANNEL_COUNT == 3
    // RMT_RX_3, corresponding to RMT channel 6
    if (intr_st & (1 << (1 + 3 * RMT_RX_channel_3)))
    {
        // change owner state, disable rx
        RMT.conf_ch[RMT_RX_channel_3].conf1.rx_en = 0;
        RMT.conf_ch[RMT_RX_channel_3].conf1.mem_owner = RMT_MEM_OWNER_TX;
        intr_st_1 += 4;

        // RMT storage pointer
        item_3 = RMTMEM.chan[RMT_RX_channel_3].data32;
    }
#endif

    // parsing output buffer
    uint32_t raw;

    // whether this transmission is valid
    bool this_valid = false;

    // parse RMT item into a uint32_t
    if ((intr_st_1 & 1) && Parse_RMT_item(item_1, &raw))
    {
#if DEBUG_LED_ENABLED
        digitalWrite(LED_PIN_1, LOW);
        if (intr_st_1 & 2)
            digitalWrite(LED_PIN_2, LOW);
        else
            digitalWrite(LED_PIN_2,HIGH);
        if (intr_st_1 & 4)
            digitalWrite(LED_PIN_3, LOW);
        else
            digitalWrite(LED_PIN_3,HIGH);
#endif
        last_RX_time = micros();
        // add element to the pool if parsing is successful
        RX_prep->msg_buffer.push(Trans_info{raw, intr_st_1, rec_time});
        this_valid = true;
    }
#if RMT_RX_CHANNEL_COUNT >= 2
    else if ((intr_st_1 & 2) && Parse_RMT_item(item_2, &raw))
    {
#if DEBUG_LED_ENABLED
        digitalWrite(LED_PIN_1, HIGH);
        digitalWrite(LED_PIN_2, LOW);
        if (intr_st_1 & 4)
            digitalWrite(LED_PIN_3, LOW);
        else
            digitalWrite(LED_PIN_3,HIGH);
#endif
        last_RX_time = micros();
        // add element to the pool if parsing is successful
        RX_prep->msg_buffer.push(Trans_info{raw, intr_st_1 & 0b110, rec_time});
        this_valid = true;
    }
#endif
#if RMT_RX_CHANNEL_COUNT == 3
    else if ((intr_st_1 & 4) && Parse_RMT_item(item_3, &raw))
    {
#if DEBUG_LED_ENABLED
        digitalWrite(LED_PIN_1, HIGH);
        digitalWrite(LED_PIN_2, HIGH);
        digitalWrite(LED_PIN_3, LOW);
#endif
        last_RX_time = micros();
        // add element to the pool if parsing is successful
        RX_prep->msg_buffer.push(Trans_info{raw, 0b100, rec_time});
        this_valid = true;
    }
#endif

    //     // test timing
    //     // the following code should be deleted when released
    //     extern uint64_t rec_finish_time;

    //     // print & reset when data complete
    //     RX_data_fragments_pool *poolptr = RX_prep->Get_pool(THIS_ROBOT_ID);
    //     if (this_valid && poolptr && poolptr->Data_valid_q() && !rec_finish_time)
    //     {
    // #if _DEBUG_PRINT_
    //         std::string str;
    //         // data
    //         for (uint32_t i = 0; i < (pool.Msg_max); i++)
    //             str += std::to_string(pool.pool[i]) + std::string(",");
    //         Serial.println(str.data());
    //         // timing
    //         str = "";
    //         str += std::to_string(pool.Time_valid_q()) + std::string(":");
    //         for (uint32_t i = 0; i < N_RECEIVERS; i++)
    //             str += std::to_string(pool.first_message_time[i]) + std::string(" ");
    //         Serial.println(str.data());
    // #endif

    //         // setup the time when data is fully captured.
    //         rec_finish_time = micros();

    //         // // reset the pool
    //         // pool = RX_data_fragments_pool{};
    //     }

    // reset memory and owner state, enable rx
    if (intr_st_1 & 1)
    {
        RMT.conf_ch[RMT_RX_channel_1].conf1.mem_wr_rst = 1;
        RMT.conf_ch[RMT_RX_channel_1].conf1.mem_owner = RMT_MEM_OWNER_RX;
        RMT.conf_ch[RMT_RX_channel_1].conf1.rx_en = 1;
    }
#if RMT_RX_CHANNEL_COUNT >= 2
    if (intr_st_1 & 2)
    {
        RMT.conf_ch[RMT_RX_channel_2].conf1.mem_wr_rst = 1;
        RMT.conf_ch[RMT_RX_channel_2].conf1.mem_owner = RMT_MEM_OWNER_RX;
        RMT.conf_ch[RMT_RX_channel_2].conf1.rx_en = 1;
    }
#endif
#if RMT_RX_CHANNEL_COUNT == 3
    if (intr_st_1 & 4)
    {
        RMT.conf_ch[RMT_RX_channel_3].conf1.mem_wr_rst = 1;
        RMT.conf_ch[RMT_RX_channel_3].conf1.mem_owner = RMT_MEM_OWNER_RX;
        RMT.conf_ch[RMT_RX_channel_3].conf1.rx_en = 1;
    }
#endif

    // clear RMT interrupt status.
    RMT.int_clr.val = intr_st;

    // delay for a bit so that the interrupt status could be cleared
    delay100ns;
    delay50ns;

    // // test end pulse
    // delayhigh500ns(TEST_PIN);
    clrbit(TEST_PIN);
}

void RMT_RX_TX::RX_process_task(void *parameters)
{
    // initialization
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        // only deal with data when there are data
        while (RX_prep->msg_buffer.n_elem)
        {
            RX_prep->Parse_RX(RX_prep->msg_buffer.pop());
        }

        // delay and let other tasks work
        vTaskDelayUntil(&xLastWakeTime, Msg_process_period / portTICK_PERIOD_MS);
    }
}

bool RMT_RX_TX::RMT_TX_add_time(uint64_t dt)
{
    timerWrite(RMT_TX_trigger_timer, (timerRead(RMT_TX_trigger_timer) + dt * RMT_TX_trigger_period * 80) % (RMT_TX_trigger_period * 80));
    return true;
}

bool RMT_RX_TX::RMT_TX_resume()
{
    timerAlarmEnable(RMT_TX_trigger_timer);
    return true;
}

bool RMT_RX_TX::RMT_TX_pause()
{
    timerAlarmDisable(RMT_TX_trigger_timer);
    return true;
}

bool RMT_RX_TX::RMT_RX_resume()
{
#if RMT_RX_CHANNEL_COUNT >= 1
    // enable RMT_RX
    if (rmt_rx_start(RMT_RX_channel_1, 1) != ESP_OK)
    {
        INIT_println("RMT RX_1 start failed!");
        return false;
    }
#endif
#if RMT_RX_CHANNEL_COUNT >= 2
    if (rmt_rx_start(RMT_RX_channel_2, 1) != ESP_OK)
    {
        INIT_println("RMT RX_2 start failed!");
        return false;
    }
#endif
#if RMT_RX_CHANNEL_COUNT == 3
    if (rmt_rx_start(RMT_RX_channel_3, 1) != ESP_OK)
    {
        INIT_println("RMT RX_3 start failed!");
        return false;
    }
#endif
    return true;
}

bool RMT_RX_TX::RMT_RX_pause()
{
#if RMT_RX_CHANNEL_COUNT >= 1
    // enable RMT_RX
    if (rmt_rx_stop(RMT_RX_channel_1) != ESP_OK)
    {
        INIT_println("RMT RX_1 start failed!");
        return false;
    }
#endif
#if RMT_RX_CHANNEL_COUNT >= 2
    if (rmt_rx_stop(RMT_RX_channel_2) != ESP_OK)
    {
        INIT_println("RMT RX_2 start failed!");
        return false;
    }
#endif
#if RMT_RX_CHANNEL_COUNT == 3
    if (rmt_rx_stop(RMT_RX_channel_3) != ESP_OK)
    {
        INIT_println("RMT RX_3 start failed!");
        return false;
    }
#endif
    return true;
}