/**
 * @file MotorCtrl.hpp
 * @brief Motor control functions
 */
#ifndef MOTORCTRL_HPP__
#define MOTORCTRL_HPP__

#include <cstdint>

/**
 * @brief whether we allow overdriving motor to gain higher thrust
 */
#define MOTOR_OVERDRIVE_ENABLED 1

namespace Motor
{
    /**
     * @brief resolution of the input PWM
     *
     * @note there's no point of making it higher than 9 bits or even lower at
     * higher output PWM frequency. cause ultimately it will be restricted by
     * the output PWM resolution of the motor driver IC. for output PWM
     * frequency vs resolution:
     *         <23.4kHz --- 9 bits
     *          46.8kHz --- 8 bits
     *          93.6kHz --- 7 bits
     *         187.2kHz --- 6 bits
     */
    constexpr uint32_t PWM_resolution = 8U;

    /**
     * @brief set so that it fits the PWM resolution.
     *
     * @note according to the datasheet:
     *         <20kHz --- 9 bits
     *          40kHz --- 8 bits
     *         100kHz --- 7 bits
     *       cannot be higher than 100kHz
     */
    constexpr uint32_t PWM_frequency = 40000U;

    // NOLINTBEGIN
    /**
     * @brief default register configuration
     *
     * @note optimized for happy model SE0802 19000KV
     */
    constexpr uint8_t Default_config[] = {
        // 0x00, 0x00,
        0x01, 0x03, 0x00, 0xFE,       // 2~5
        0x00, 0x00, 0x10, 0x20, 0x00, // 6~10
        0x01, 0x00, 0x00, 0x97, 0xD5, // 11~15
        0x00, 0xE6, 0x03, 0x16, 0x0A, // 16~20
        0x8F, 0x56, 0x3F, 0xC0,       // 21~24
    };

    /**
     * @brief when below this thrust and not zero, round up to this thrust.
     */
    constexpr float Min_thrust = 4.0F;

    /**
     * @brief max thrust achievable in grams
     */
    constexpr float Max_thrust = 22.5F;

#if MOTOR_OVERDRIVE_ENABLED
    /**
     * @brief at high speed, we might want to switch to 192kHz to reduce peak
     * current and obtain higher thrust. This value is for register 22.
     */
    constexpr uint8_t Overdrive_config = 0x5E;

    /**
     * @brief if lower than this thrust, automatically exit overdrive mode to
     * prevent failure of reboot and grant higher resolution.
     */
    constexpr float Min_thrust_overdrive = 8.0F;

    /**
     * @brief max thrust in overdrive mode in grams
     */
    constexpr float Max_thrust_overdrive = 28.0F;
    // constexpr float Max_thrust_overdrive = 25.0F;
#endif

    // /**
    //  * @brief default register configuration
    //  *
    //  * @note optimized for beta fpv 0802SE 22000KV
    //  */
    // constexpr uint8_t Default_config[] = {
    //     // 0x00, 0x00,
    //     0x01, 0x03, 0x00, 0xFE,       // 2~5
    //     0x00, 0x00, 0x10, 0x20, 0x00, // 6~10
    //     0x01, 0x00, 0x00, 0x97, 0xD5, // 11~15
    //     0x00, 0xE6, 0x03, 0x16, 0x0A, // 16~20
    //     0x6F, 0x56, 0x3F, 0xC0,       // 21~24
    // };
    // NOLINTEND

    /**
     * @brief initialize motor
     *
     * @return 1 for success, 0 for failure
     */
    uint32_t Init();

    /**
     * @brief config the registers
     *
     * @note you don't have to set default config. that's already initialized.
     * the default config should be used if there's no very special need.
     *
     * @param address the register's address
     * @param value the new value
     *
     * @return 1 for success, 0 for failure
     *
     * @warning you shouldn't use this unless you know what registers really
     * are!
     */
    uint32_t Config_register(const uint8_t address, const uint8_t value);

    /**
     * @brief set motor speed. In open loop, the PWM duty cycle is
     * duty/2^PWM_resolution
     *
     * @param duty duty cycle (when in open loop mode) or motor speed (when in
     * closed loop mode)
     */
    void Set_speed(const uint32_t duty);

    /**
     * @brief get last set speed
     *
     * @return uint32_t which is the speed you just set.
     *
     * @note this returns the speed you set last time, not the current actual
     * speed! For that please call Measure_speed().
     */
    uint32_t Get_last_set_speed();

    /**
     * @brief get motor speed
     *
     * @return uint32_t speed in some form, maybe pulse width maybe something
     * else. still haven't decided yet.
     *
     * @note DO NOT call this function frequently because it will halt this core
     * and wait for pulses, potentially up to 10ms depending on the time_out
     * settings. if we ever need to, we can switch to an interrupt based, always
     * on function, but I will keep it like this for now.
     */
    uint32_t Measure_speed();

    /**
     * @brief forcefully brake (not break) the motor
     *
     * @note this function is not the same as Motor_set_speed(0), where it's
     * passively braked (coast). this function actively brakes the motor faster.
     * But you will need to release brake manually using Active_brake_release().
     */
    void Active_brake();

    /**
     * @brief release the brake
     */
    void Active_brake_release();

    /**
     * @brief Get brake status
     * 
     * @return true brake enabled
     */
    bool Get_brake_status();

    /**
     * @brief set thrust to approximately a certain value
     *
     * @param thrust desired thrust value in grams.
     */
    void Set_thrust(const float thrust);

#if MOTOR_OVERDRIVE_ENABLED
    /**
     * @brief enter or exit overdrive mode which grant higher thrust!
     *
     * @param state true for enter, false for exit.
     */
    void Set_overdrive(bool state);

    /**
     * @brief whether we are in overdrive mode or not
     * 
     * @return true overdrive mode enabled
     */
    bool Get_overdrive_mode();
#endif
} // namespace Motor

#endif