/**
 * @file RMTMessageDefs.hpp
 * @brief Definitions for RMT messages
 *
 * @note Because I implemented a HPF in the filter to deal with change in
 * environmental light and irrelavent blinks, The data we are sending must have
 * a fixed portion of low level and a fixed portion of high level, evenly
 * distributed. A natural idea would be to use Manchester encoding. (This is
 * also a commonly used IR protocol) But the problem with such method is that
 * pulses can be merged into a single larger pulse, and each pulse contains only
 * one bit of info. This will bring difficulty to the parsing process.
 * So what I do right now is use a modified 4ppm method (let's call it 1+4ppm)
 * where a unit of empty space is added between two 4ppm cycle. Now pulses won't
 * merge and each pulse represent two bits. an additional benifit is that the
 * power consumption is lower too due to smaller duty.
 *   0 -> 00001
 *   1 -> 00010
 *   2 -> 00100
 *   3 -> 01000
 * To identify the top and bottom emitter while allowing them to be transmitted
 * at the same time (I don't want to use up the multiplexing ability!) I make
 * the last pulse to be variable in length, and the emitter that will always be
 * seen first (by convention, the top one) will have longer pulse. So that if
 * the drone can see the top emitter or both, it will localize based on the top
 * emitter's position. For example, a signal of 01100011 will be translated
 * into:
 *   01100011 -> 1|00010|00100|00001|01(1)
 * where the first 1 is the starting signal, and 01(1) is the last two bits,
 * which can be long or short depending on the emitter.
 *
 * @note Each raw message is in the following form:
 * | ---robot_ID--- | ---msg_ID--- | ---Data or header info--- | (---ECC--- |)
 * where ID is the robot's id, MSG_ID is the ID of message, when all msg_id sent
 * from robot_id is received, then a transmission is complete. I think ECC bit
 * is not necessary, because it is highly unlikely that the received signal is
 * distorted in a way that result in a wrong signal instead of a bad signal. I
 * will test that in the reality before deciding whether to add in the ECC bits.
 *
 * Messages with Msg_ID = x000.. is header message, which contains information
 * about how many raw message would complete the data and probably some ECC.
 *
 * @warning this header file is for messages <=32 bits, if you want to use
 * larger messages, change uint32_t to uint64_t.
 */
#ifndef RMTMESSAGEDEFS_HPP__
#define RMTMESSAGEDEFS_HPP__

#include <PinDefs.hpp>

/**
 * @brief IR communication and localization
 */
namespace IR
{
	namespace detail
	{
		// message content specifications

		/**
		 * @brief number of bits for robot's id.
		 */
		constexpr uint32_t Robot_ID_bits = 6;

		/**
		 * @brief type of message
		 * @note the reason we have this is because we want to transmit multiple
		 * types of messages at the same time. and possibly with different
		 * frequency and different priority. For example, control signal are
		 * short and should be sent more frequently,
		 *
		 *      timing sync messages only need to be sent once in a while.
		 *      highest priority alert messages should be sent immediately and
		 * occupy the whole channel.
		 *      localization information messages are of lowest priority and
		 * should be sent all the time.
		 *
		 * Msg_type_bits also allow us to have certain pre-defined types where
		 * one transmission is be sufficient. e.g. control/alert signal. this is
		 * set by Single_transmission_msg_type. Any messages with Msg_type <=
		 * Single_transmission_msg_type is single transmission messages.
		 */
		constexpr uint32_t Msg_type_bits = 3;

		/**
		 * @brief max msg type
		 */
		constexpr uint32_t Msg_type_max = (1 << Msg_type_bits) - 1;

		/**
		 * @brief Messages with Msg_type <= Single_transmission_msg_type are
		 * single transmission messages.
		 */
		constexpr uint32_t Single_transmission_msg_type = (1 << (Msg_type_bits - 1)) - 1;

		/**
		 * @brief number of bits for message's id.
		 * @note message ID on messages for the same data should have same
		 * initial bit, so that we can distinguish from different data groups
		 * sent by the same robot at different time. If the robot received a
		 * message with the other intial bit, it should discard all previous
		 * incomplete messages With the previous initial bit, because the data
		 * they contains are already obsolete. Should set to <= 7, for two
		 * reasons.
		 *  1. With Msg_ID_bits <= 8, the ID has <= 7 bits, thus we can use a
		 * uint64_t to indicate whether we received all the data. Save space &
		 * time.
		 *  2. The full data should be transmitted in <30 deg time, and with 0.5
		 * deg / message, this means 2^6 messages. Adding up with the header
		 * bit, it's 7 bits.
		 */
		constexpr uint32_t Msg_ID_bits = 5;

		/**
		 * @brief number of BYTES for message's content.
		 */
		constexpr uint32_t Msg_content_bytes = 2;

		/**
		 * @brief number of bits for message's content.
		 *
		 * @note please make sure that this is a multiplier of 8 for easy
		 * conversion. If not, you will have to write your own function to
		 * convert an arbitrary data into a array of that many bits. And you
		 * will have to write your own ECC as well.
		 */
		constexpr uint32_t Msg_content_bits = Msg_content_bytes * 8;

		/**
		 * @brief The amount of data, in bits, that's transmitted in each RMT
		 * transmission.
		 *
		 * @note RMT length should be kept below 63 for convenience of
		 * processing (leaving some space for init code and probably ECC.) When
		 * RMT length is larger than 63, it will occupy at least two RMT memory
		 * register block. That's inconvenient and requires much more
		 * processing. I would suggest using rmt length from 16 to 32. Using
		 * rmt length >32 then you will need to modify the code here and there,
		 * changing uint32_t to uint64_t.
		 *
		 * @warning Keep this an even number if using 4ppm encoding!
		 *
		 * @note for simplicity, make sure that this is <= 31, we are currently
		 * using the 32th bit to pass the receiver's position (top or bottom).
		 */
		constexpr uint32_t RMT_data_length = Robot_ID_bits + Msg_type_bits + Msg_ID_bits + Msg_content_bits;

		// data structure specification

		/**
		 * @brief general structure of messages with <=32 bits.
		 */
		union Msg_t
		{
			struct
			{
				uint32_t robot_ID : Robot_ID_bits;
				uint32_t msg_type : Msg_type_bits;
				uint32_t msg_ID_init : 1;
				uint32_t msg_ID : Msg_ID_bits - 1;
				uint32_t content : Msg_content_bits;
			};
			uint32_t raw;
		};

		/**
		 * @brief general structure of single transmission messages.
		 */
		union Msg_single_t
		{
			struct
			{
				uint32_t robot_ID : Robot_ID_bits;
				uint32_t msg_type : Msg_type_bits;
				uint32_t msg_ID_init : 1;
				uint32_t CRC : Msg_ID_bits - 1;
				uint32_t content : Msg_content_bits;
			};
			uint32_t raw;
		};

		/**
		 * @brief general structure of message headers with <=32 bits.
		 */
		union Msg_header_t
		{
			struct
			{
				uint32_t robot_ID : Robot_ID_bits;
				uint32_t msg_type : Msg_type_bits;
				uint32_t msg_ID_init : 1;
				uint32_t msg_ID : Msg_ID_bits - 1;
				uint32_t msg_ID_len
					: Msg_ID_bits -
					  1; // Msg_ID will be from 0 (the header) to Msg_ID_len.
				uint32_t CRC : Msg_content_bits - Msg_ID_bits + 1;
			};
			uint32_t raw;
		};

		/**
		 * @brief a struct packed with data / receiver state / time of
		 * reception, which is all information contained in a transmission.
		 */
		struct Trans_info
		{
			uint32_t data;
			uint32_t receiver;
			int64_t time;
		};

		// RMT timing specifications

		/**
		 * @brief RMT clock division, rmt clock speed = 80MHz / RMT_clock_div
		 *
		 * @note RMT_ticks_num * RMT_clock_div * 1/80MHz is the pulse width.
		 */
		constexpr uint32_t RMT_clock_div = 2;

		/**
		 * @brief RMT_ticks_num = 1 << RMT_ticks_shift
		 */
		constexpr uint32_t RMT_ticks_shift = 3;

		/**
		 * @brief Ticks count of each pulse
		 *
		 * @note RMT_ticks_num * RMT_clock_div * 1/80MHz is the pulse width.
		 */
		constexpr uint32_t RMT_ticks_num = 1 << RMT_ticks_shift;

		/**
		 * @brief padding ticks added before TX channel 2 signal for
		 * synchronization
		 */
		constexpr uint32_t RMT_sync_ticks_num = 5;

		/**
		 * @brief final pulse duration of TX channel 2
		 */
		constexpr uint32_t CH2_final_pulse_duration = 2 * RMT_ticks_num;

		/**
		 * @brief Ticks number that deviate from RMT_ticks_num by RMT_ticks_tol
		 * will still be accepted
		 *
		 * @note There will be error in ticks number in reality, so we could
		 * give it some tolerance
		 */
		constexpr uint32_t RMT_ticks_tol = 1;

		/**
		 * @brief Number of bits per cycle/pulse
		 *
		 * @note Please keep it at 2 for simplicity, making it larger won't
		 * improve the performance much further.
		 */
		constexpr uint32_t Bit_per_cycle = 2;

		/**
		 * @brief Padding before each cycle in ticks
		 */
		constexpr uint32_t Pad_per_cycle = RMT_ticks_num;

		/**
		 * @brief RMT data pulse count.
		 *
		 * @note RMT_data_pulse_count = RMT_data_length / Bit_per_cycle
		 */
		constexpr uint32_t RMT_data_pulse_count = RMT_data_length / Bit_per_cycle;

		/**
		 * @brief RMT TX data structure length (array of rmt_item32_t).
		 *
		 * @note RMT_TX_length = RMT_data_pulse_count + 2 because there will be
		 * a '0' header and a {0} as ending block
		 */
		constexpr uint32_t RMT_TX_length = RMT_data_pulse_count + 3;

		// TX period info

		/**
		 * @brief RMT TX minimum trigger period in us
		 */
		constexpr uint32_t RMT_TX_trigger_period_min = 50;
		// constexpr uint32_t RMT_TX_trigger_period_min = 1000;

		/**
		 * @brief RMT TX maximum trigger period in us
		 */
		constexpr uint32_t RMT_TX_trigger_period_max = 100;
		// constexpr uint32_t RMT_TX_trigger_period_max = 2000;

		/**
		 * @brief elevation distance between up and down emitters
		 */
		constexpr float LED_elevation_diff = 35.74F;
	} // namespace detail
} // namespace IR

#endif