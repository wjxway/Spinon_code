#include "IrSense.hpp"
#include <array>
#include <SPI.h>
#include <PinDefs.hpp>
#include <FastIO.hpp>
#include <DebugDefs.hpp>
#include <RMTMessageDefs.hpp>
#include <RMTCoding.hpp>
#include <CRCCheck.hpp>
#include "hal/rmt_ll.h"
#include "driver/rmt.h"
#include "driver/timer.h"

// HSPI SS pin
#define SENSE_SS 25

namespace IR
{
    using namespace detail;

    namespace Sense
    {
        /**
         * @brief ADC SPI SCK frequency
         */
        constexpr uint32_t ADC_SCK_frequency = 10000000U;

        namespace
        {
            char HexLookUp[] = "0123456789ABCDEF";
            void bytes2hex(unsigned char *src, char *out, int len)
            {
                while (len--)
                {
                    *out++ = HexLookUp[*src >> 4];
                    *out++ = HexLookUp[*src & 0x0F];
                    src++;
                }
                *out = 0;
            }

            // SPI interface
            SPIClass *hspi;

            // LTC1408/LTC2351 driver
            /**
             * @brief initialize ADC
             */
            void ADC_init()
            {
                pinMode(SENSE_SS, OUTPUT);
                setbit(SENSE_SS);

                // spi init
                hspi = new SPIClass(HSPI);
                hspi->begin();
            }

            /**
             * @brief read all channels of ADC (simultaneous sampling)
             *
             * @return std::array<int16_t, RMT_RX_CHANNEL_COUNT> data
             */
            std::array<int16_t, RMT_RX_CHANNEL_COUNT> ADC_read()
            {
                delay500ns;

                setbit(SENSE_SS);
                delay50ns;
                clrbit(SENSE_SS);

                std::array<int16_t, RMT_RX_CHANNEL_COUNT> data;
                uint8_t in[6] = {0, 0, 0, 0, 0, 0};
                uint8_t out[6];

                // read out
                hspi->beginTransaction(SPISettings(ADC_SCK_frequency, MSBFIRST, SPI_MODE3));
                // we only read first 3 channels, 2 bytes each.
                hspi->transferBytes(in, out, 6);
                hspi->endTransaction();

                // two complement
                for (size_t i = 0; i < RMT_RX_CHANNEL_COUNT; i++)
                {
                    data[i] = (((int16_t(out[2 * i] & 0x3F) << 8) + out[2 * i + 1]) ^ 0x2000) - 0x2000;
                }

                // char str[15];
                // bytes2hex(out,str,6);
                // Serial.println(str);

                return data;
            }

            // // AD7381-4 driver
            // /**
            //  * @brief initialize ADC
            //  *
            //  * @note 1 wire MISO SPI interface, other options to default.
            //  */
            // void ADC_init()
            // {
            //     pinMode(SENSE_SS, OUTPUT);
            //     setbit(SENSE_SS);

            //     // spi init
            //     hspi = new SPIClass(HSPI);
            //     hspi->begin();

            //     // reset
            //     hspi->beginTransaction(SPISettings(ADC_SCK_frequency, MSBFIRST, SPI_MODE2));
            //     clrbit(SENSE_SS);
            //     hspi->transfer16(0xA3FF);
            //     setbit(SENSE_SS);
            //     hspi->endTransaction();

            //     delayMicroseconds(1000);

            //     // set to enable RX CRC
            //     hspi->beginTransaction(SPISettings(ADC_SCK_frequency, MSBFIRST, SPI_MODE2));
            //     clrbit(SENSE_SS);
            //     hspi->transfer16(0x9010);
            //     setbit(SENSE_SS);
            //     hspi->endTransaction();

            //     delay1us;

            //     // set to 1-wire mode
            //     hspi->beginTransaction(SPISettings(ADC_SCK_frequency, MSBFIRST, SPI_MODE2));
            //     clrbit(SENSE_SS);
            //     hspi->transfer16(0xA300);
            //     setbit(SENSE_SS);
            //     hspi->endTransaction();
            // }

            // /**
            //  * @brief read all channels of ADC (simultaneous sampling)
            //  *
            //  * @return std::array<int16_t, RMT_RX_CHANNEL_COUNT> data
            //  */
            // std::array<int16_t, RMT_RX_CHANNEL_COUNT> ADC_read()
            // {
            //     std::array<uint16_t, RMT_RX_CHANNEL_COUNT> tmp;
            //     std::array<int16_t, RMT_RX_CHANNEL_COUNT> data;
            //     uint8_t in[8] = {0, 0, 0, 0, 0, 0, 0, 0};
            //     uint8_t out[8];

            //     // convert
            //     clrbit(SENSE_SS);
            //     delay1us;
            //     setbit(SENSE_SS);
            //     delay1us;

            //     // read out
            //     hspi->beginTransaction(SPISettings(ADC_SCK_frequency, MSBFIRST, SPI_MODE2));
            //     clrbit(SENSE_SS);
            //     // we only read first 3 channels, 3*14 = 42 bits < 6 bytes
            //     // if this becomes a problem, just switch to 7 bits and read all channels
            //     hspi->transferBytes(in, out, 8);
            //     setbit(SENSE_SS);
            //     hspi->endTransaction();

            //     tmp[0] = (uint16_t(out[0] & 0x7F) << 7) | (out[1] >> 1);
            //     tmp[1] = (uint16_t(out[1] & 0x01) << 13) | (uint16_t(out[2]) << 5) | (out[3] >> 3);
            //     tmp[2] = (uint16_t(out[3] & 0x07) << 11) | (uint16_t(out[4]) << 3) | (out[5] >> 5);

            //     for (size_t i = 0; i < 3; i++)
            //     {
            //         data[i] = int32_t(tmp[i] ^ 0x2000) - 0x2000;
            //     }

            //     return data;
            // }

            /**
             * @brief data transmitted for sensing
             */
            rmt_item32_t Sensing_TX_data[RMT_TX_length];
        } // anonymous namespace

        void Init()
        {
            // init ADC
            ADC_init();

            // init RMT TX channel 1 only
            pinMode(RMT_TX_PIN_1, OUTPUT);
            clrbit(RMT_TX_PIN_1);
            // config TX
            rmt_config_t rmt_tx;
            rmt_tx.channel = RMT_TX_channel_1;
            rmt_tx.gpio_num = (gpio_num_t)RMT_TX_PIN_1;
            rmt_tx.clk_div = RMT_clock_div;
            rmt_tx.mem_block_num = 1;
            rmt_tx.rmt_mode = RMT_MODE_TX;
            rmt_tx.tx_config.loop_en = false;
            // We modulate our own signal, instead of using carrier!
            rmt_tx.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
            rmt_tx.tx_config.carrier_en = 0;
            rmt_tx.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
            rmt_tx.tx_config.idle_output_en = 1;
            // initialization of RMT
            DEBUG_C(
                if (rmt_config(&rmt_tx) != ESP_OK)
                    Serial.println("RMT TX_1 init failed!"));

            // set source clk to APB clock
            rmt_set_source_clk(RMT_TX_channel_1, RMT_BASECLK_APB);

            // disable interrupt
            rmt_set_tx_intr_en(RMT_TX_channel_1, false);
            rmt_set_tx_thr_intr_en(RMT_TX_channel_1, false, 1);

            DEBUG_C(Serial.println("TX_Sense init successful!"));

            // generate a constant message for convenience
            // message of type 0, ID_init 0, content 0x1234
            Generate_RMT_item(Sensing_TX_data, Msg_single_t{{This_robot_ID, 0, 0, crc4_itu(0x1234), 0x1234}}.raw);

            DEBUG_C(Serial.println("TX_Sense message generation successful!"));
        }

        std::array<int16_t, RMT_RX_CHANNEL_COUNT> Transmit_and_sense(int32_t ticks_delay)
        {
            portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
            taskENTER_CRITICAL(&myMutex);

            // write register for RMT TX channel 1
            for (size_t i = 0; i < RMT_TX_length; i++)
            {
                RMTMEM.chan[RMT_TX_channel_1].data32[i].val = (Sensing_TX_data + i)->val;
            }

            // add delay to transmission.
            // 1 tick is RMT_clock_div * 1/80MHz.
            // now with RMT_clock_div = 2, 1 tick is 25ns.
            RMTMEM.chan[RMT_TX_channel_1].data32[0].duration0 += ticks_delay;

            // reset pointers
            rmt_ll_tx_reset_pointer(&RMT, RMT_TX_channel_1);
            // start RMT as fast as possible by directly manipulating the registers
            RMT.conf_ch[RMT_TX_channel_1].conf1.tx_start = 1;

            // return read value
            auto temp = ADC_read();
            
            taskEXIT_CRITICAL(&myMutex);

            return temp;
        }
    } // namespace Sense
} // namespace IR