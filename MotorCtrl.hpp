// Motor control functions
#ifndef _MOTORCTRL_HPP_
#define _MOTORCTRL_HPP_
#include "Arduino.h"

class Motor
{
public:
    /**
     * @brief resolution of the input PWM
     *
     * @note there's no point of making it higher than 9 bits or even lower at higher output PWM frequency.
     *       cause ultimately it will be restricted by the output PWM resolution of the motor driver IC.
     *       for output PWM frequency vs resolution:
     *         <23.4kHz --- 9 bits
     *          46.8kHz --- 8 bits
     *          93.6kHz --- 7 bits
     *         187.2kHz --- 6 bits
     */
    static constexpr uint32_t PWM_resolution = 7;

    /**
     * @brief set so that it fits the PWM resolution.
     *
     * @note according to the datasheet:
     *         <20kHz --- 9 bits
     *          40kHz --- 8 bits
     *         100kHz --- 7 bits
     *       cannot be higher than 100kHz
     */
    static constexpr uint32_t PWM_frequency = 40000;

    /**
     * @brief the last set speed or duty.
     * 
     * @note it is the value you set through Set_speed last time, and will be different from the actual motor speed. 
     *       to get the actual motor speed, use Measure_speed().
     */
    uint32_t Last_set_speed = 0;

    /**
     * @brief initialize motor
     * 
     * @return 1 for success, 0 for failure
     */
    uint8_t Init();

    /**
     * @brief config the registers
     * 
     * @note you don't have to set default config. that's already initialized.
     *       the default config should be used if there's no very special need.
     * 
     * @param address the register's address
     * @param value the new value
     * 
     * @return 1 for success, 0 for failure
     */
    uint8_t Config_register(uint8_t address, uint8_t value);

    /**
     * @brief set motor speed
     *
     * @param duty duty cycle (when in open loop mode) or motor speed (when in closed loop mode)
     */
    void Set_speed(uint32_t duty);

    /**
     * @brief get motor speed
     *
     * @return uint32_t speed in some form, maybe pulse width maybe something else. still haven't decided yet.
     * 
     * @note DO NOT call this function frequently because it will halt this core and wait for pulses, potentially up to 10ms depending on the time_out settings. 
     *       if we ever need to, we can switch to an interrupt based, always on function, but I will keep it like this for now.
     */
    uint32_t Measure_speed();

    /**
     * @brief forcefully brake (not break) the motor
     *
     * @note this function is not the same as Motor_set_speed(0), where it's passively braked (coast).
     *       this function actively brakes the motor faster.
     *       But you will need to release brake manually using Active_brake_release().
     */
    void Active_brake();

    /**
     * @brief release the brake
     */
    void Active_brake_release();

    /**
     * @brief an ISR that get triggered when alert is fired.
     *
     * @note actively brake the motor for now.
     */
    void IRAM_ATTR Alert_ISR(void *arg);

private:
    static constexpr uint8_t Default_config[] = {
        // 0x00, 0x00,
              0x01, 0x03, 0x00, 0xFE, // 2~5
        0x00, 0x00, 0x10, 0x20, 0x00, // 6~10
        0x01, 0x00, 0x00, 0x97, 0xD5, // 11~15
        0x00, 0xE6, 0x03, 0x16, 0x0A, // 16~20
        0x3F, /*0x2E*/ 0x32+4, 0x2B, 0x40,       // 21~24
    };
};

#endif