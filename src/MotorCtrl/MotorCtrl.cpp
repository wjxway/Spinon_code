#include "MotorCtrl.hpp"
#include "../Utilities/FastIO.hpp"
#include "../Utilities/PinDefs.hpp"
#include <Wire.h>

namespace Motor
{
    namespace
    {
        /**
         * @brief the last set speed or duty.
         *
         * @note it is the value you set through Set_speed last time, and will be
         * different from the actual motor speed. to get the actual motor speed, use
         * Measure_speed().
         */
        uint32_t Last_set_speed = 0u;
    }

    /**
     * @brief an ISR that get triggered when alert is fired.
     *
     * @note actively brake the motor for now.
     */
    void IRAM_ATTR Alert_ISR();

    uint32_t Init()
    {
        // pin modes
        pinMode(MOTOR_ALERT_PIN, INPUT_PULLUP);
        pinMode(MOTOR_SPD_FB_PIN, INPUT_PULLUP);
        pinMode(MOTOR_SPD_CTRL_PIN, OUTPUT);
        pinMode(MOTOR_BRAKE_PIN, OUTPUT);

        // default state
        clrbit(MOTOR_BRAKE_PIN);
        clrbit(MOTOR_SPD_CTRL_PIN);

        // i2c start
        Wire.begin(MOTOR_SDA_PIN, MOTOR_SCL_PIN, 200000u);

        // check if successful
        uint32_t temp = 1u;
        // initial config
        for (uint8_t addr = 0u; addr < 23u; addr++)
        {
            temp &= Config_register(addr + 2u, Default_config[addr]);
        }

        // ledc start
        ledcSetup(Motor_LEDC_PWM_channel, PWM_frequency, PWM_resolution);
        ledcAttachPin(MOTOR_SPD_CTRL_PIN, Motor_LEDC_PWM_channel);
        ledcWrite(Motor_LEDC_PWM_channel, 0u);

        // attach alert interrupt
        attachInterrupt(MOTOR_ALERT_PIN, Alert_ISR, FALLING);

        return temp;
    }

    uint32_t Config_register(const uint8_t address, const uint8_t value)
    {
        uint32_t state = 0u;

        // write i2c device address
        Wire.beginTransmission(Motor_address);
        // write register address
        Wire.write(address);
        // write data
        Wire.write(value);
        // end transmission
        state = Wire.endTransmission();

        // not sure whether necessary, but let's add it anyways
        delayMicroseconds(100u);

        return state;
    }

    void Set_speed(const uint32_t duty)
    {
        Last_set_speed = duty;
        ledcWrite(Motor_LEDC_PWM_channel, duty);
    }

    uint32_t Get_last_set_speed()
    {
        return Last_set_speed;
    }

    uint32_t Measure_speed()
    {
        // set a time out (in us) or else it will freeze the core when at 0 speed.
        constexpr unsigned long time_out = 5000u;

        // starting time
        unsigned long t_start = micros();
        // wait for the pin to go low
        while (fastread(MOTOR_SPD_FB_PIN) && (micros() - t_start < time_out))
        {
        }
        // wait for the pin to go high
        while ((!fastread(MOTOR_SPD_FB_PIN)) && (micros() - t_start < time_out))
        {
        }
        // directly return if timed out
        if (micros() - t_start > time_out)
            return 0u;

        // start recording time
        t_start = micros();
        // wait for the pin to go high
        while (fastread(MOTOR_SPD_FB_PIN) && (micros() - t_start < time_out))
        {
        }

        // save time frame
        t_start = micros() - t_start;

        return (t_start > time_out) ? 0u : (4000000u / t_start);
    }

    void Active_brake()
    {
        Last_set_speed = 0u;
        setbit(MOTOR_BRAKE_PIN);
    }

    void Active_brake_release()
    {
        clrbit(MOTOR_BRAKE_PIN);
    }

    void IRAM_ATTR Alert_ISR()
    {
        Active_brake();
    }
}