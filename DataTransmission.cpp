#include "DataTransmission.h"

const uint8_t RMT_data_length = Robot_ID_bits + Msg_ID_bits + Msg_content_bits;

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

inline uint32_t Construct_content(uint8_t *pointer)
{
    uint32_t res = 0;
    for (uint32_t i = 0; i < Msg_content_bytes; i++)
        res += (uint32_t(pointer[i]) << (i << 3));
    return res;
}

inline void Deconstruct_content(uint32_t data, uint8_t *pointer)
{
    for (uint32_t i = 0; i < Msg_content_bytes; i++)
        *(pointer + i) = ((data & (0xFF << (i << 3))) >> (i << 3));
}

rmt_item32_t *RMT_TX_prep::Get_TX_pointer()
{
    rmt_item32_t *temp = output.data() + output_pos;

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

    // wait for spin lock
    while (!output_ready_flag)
    {
    }
    // set flag to false to stop all access of TX data
    output_ready_flag = false;

    // number of all messages - 1 (excluding header message)
    uint32_t n_msg = raw.size() / Msg_content_bytes;
    // setup container
    output.assign((n_msg + 1) * RMT_TX_length, {{0}});
    // reset output pointer
    output_pos = 0;

    // create header
    Generate_RMT_item(output.data(), Msg_header_t{{robot_ID, msg_id_initial, 0, n_msg, crc8_maxim(raw.data(), raw.size())}}.raw);
    // cycle through data
    for (uint32_t msgid = 1; msgid <= n_msg; msgid++)
        Generate_RMT_item(output.data() + msgid * RMT_TX_length, Msg_t{{robot_ID, msg_id_initial, msgid, Construct_content(raw.data() + (msgid - 1) * Msg_content_bytes)}}.raw);

    // change msg_id_initial for next data set
    msg_id_initial = (msg_id_initial ? 0 : 1);

    // reset spin lock
    output_ready_flag = true;

    return true;
}

void RX_data_fragments_pool::reset_pool(uint32_t data, uint32_t receiver, uint64_t time)
{
    // convert the raw message to Msg_t type
    Msg_t msg;
    msg.raw = data;

    // reset filling status
    filling_status = 0;

    // reset first message time only when it's a time out reset
    if ((time - last_message_time) > Timing_expire_time)
    {
        for (uint8_t i = 0; i < N_RECEIVERS; i++)
        {
            if ((receiver >> i) & 1)
                first_message_time[i] = time;
            else
                first_message_time[i] = 0;
        }
        time_ready_status = receiver;
    }

    last_message_time = time;
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
        msg_hd.raw = data;

        Msg_count = 0;
        Msg_max = msg_hd.Msg_ID_len;
        CRC = msg_hd.CRC;
    }
}

bool RX_data_fragments_pool::Add_element(uint32_t data, uint32_t receiver, uint64_t time)
{
    // convert the raw message to Msg_t type
    Msg_t msg;
    msg.raw = data;

    // some helper criteria
    bool prev_filled = false, header_invalid = false;

    // data or header?
    if (msg.Msg_ID)
        // if is data and previously filled
        prev_filled = (filling_status >> (msg.Msg_ID - 1)) & 1;
    else
    {
        Msg_header_t msg_hd;
        msg_hd.raw = data;

        // 1. Msg_max inconsistent or CRC inconsistent
        // 2. previous data's Msg_ID > Msg_ID_len
        header_invalid = (Msg_max && (Msg_max != msg_hd.Msg_ID_len || CRC != msg_hd.CRC)) || (filling_status >= (uint64_t(1) << msg_hd.Msg_ID_len));

#ifdef _DEBUG_PRINT_
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
        || (time - last_message_time) > Timing_expire_time                                                  // 2. data has expired
        || (Msg_max && msg.Msg_ID > Msg_max)                                                                // 3. Msg_ID too big
        || (prev_filled && (Construct_content(pool + (msg.Msg_ID - 1) * Msg_content_bytes) != msg.content)) // 4. data inconsistent
        || header_invalid)                                                                                  // 5. header invalid
    {
#ifdef _DEBUG_PRINT_
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

        reset_pool(data, receiver, time);
    }
    // valid data
    else
    {
        // if this is the first message from a receiver, record the time.
        // we could just input RMT.int_st.val, but then it would be hard to change the RMT channels without changing the library.
        uint32_t temp = (receiver & (~time_ready_status));
        for (uint8_t i = 0; i < N_RECEIVERS; i++)
            if ((temp >> i) & 1)
                first_message_time[i] = time;

        // update time_ready_status
        time_ready_status = time_ready_status | receiver;

        // update last_message_time
        last_message_time = time;

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
#if _DEBUG_PRINT_
                Serial.println("CRC mismatch reset");
#endif
                reset_pool(data, receiver, time);
            }
        }
    }

    return 0;
}

bool RX_data_fragments_pool::Time_valid_q()
{
    return (time_ready_status == ((1 << N_RECEIVERS) - 1));
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

void RX_data_fragments_pool::Reset_all()
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

bool RMT_RX_prep::Parse_RX(volatile rmt_item32_t *pointer)
{
    return true;
}

void RMT_RX_prep::Delete_obsolete()
{

}