#include "MotorCtrl.hpp"
#include <FastIO.hpp>
#include <FastMath.hpp>
#include <PinDefs.hpp>
#include <Wire.h>

#include <hal/cpu_hal.h>

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
		uint32_t Last_set_speed = 0U;

		/**
		 * @brief if in brake mode, if so, no thrust command is valid!
		 */
		bool Brake_mode = false;

		/**
		 * @brief whether we are in overdrive mode which grant higher thrust!
		 */
		bool Overdrive_state = false;

		/**
		 * @brief an ISR that get triggered when alert is fired.
		 *
		 * @note actively brake the motor for now.
		 */
		void IRAM_ATTR Alert_ISR()
		{
			Active_brake();
		}

		/**
		 * @brief compute throttle value based on thrust force
		 *
		 * @param thrust thrust value in gram
		 * @return uint32_t corresponding throttle value
		 */
		constexpr uint32_t Compute_throttle(float thrust)
		{
			return uint32_t(4.25F * thrust + 40.0F);
		}
	} // anonymous namespace

	uint32_t Init()
	{
		// always disable overdrive in the beginning
		Overdrive_state = false;

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
		uint32_t temp = 1U;
		// initial config
		for (uint8_t addr = 0U; addr < 23U; addr++) // NOLINT
		{
			temp &= Config_register(addr + 2U, Default_config[addr]);
		}

		// LEDC start
		// PWM LEDC will take up general purpose timer 0.
		ledcSetup(Motor_LEDC_PWM_channel, PWM_frequency, PWM_resolution);
		ledcAttachPin(MOTOR_SPD_CTRL_PIN, Motor_LEDC_PWM_channel);
		ledcWrite(Motor_LEDC_PWM_channel, 0U);

		// attach alert interrupt
		attachInterrupt(MOTOR_ALERT_PIN, Alert_ISR, FALLING);

		return temp;
	}

	uint32_t Config_register(const uint8_t address, const uint8_t value)
	{
		uint32_t state = 0U;

		// write i2c device address
		Wire.beginTransmission(Motor_address);
		// write register address
		Wire.write(address);
		// write data
		Wire.write(value);
		// end transmission
		state = Wire.endTransmission();

		// not sure whether necessary, but let's add it anyways
		delayMicroseconds(100U); // NOLINT

		return state;
	}

	void Set_speed(const uint32_t duty)
	{
		if (!Brake_mode)
		{
			Last_set_speed = duty;
			ledcWrite(Motor_LEDC_PWM_channel, duty ? (8U * duty - 4U) : 0U);
		}
	}

	uint32_t Get_last_set_speed()
	{
		return Last_set_speed;
	}

	uint32_t Measure_speed()
	{
		// set a time out (in CPU ticks) or else it will freeze the core when at 0 speed.
		constexpr uint32_t time_out_ticks = 100000U;

		// starting time
		uint32_t t_enter = cpu_hal_get_cycle_count();

		// what's the initial state of the SPD_FB pin
		bool init = fastread(MOTOR_SPD_FB_PIN);

		// wait for the pin to go the other direction
		while ((fastread(MOTOR_SPD_FB_PIN) == init) && (cpu_hal_get_cycle_count() - t_enter < time_out_ticks))
		{
		}
		// start recording time
		int64_t t_start = cpu_hal_get_cycle_count();
		// wait for the pin to go high
		while ((fastread(MOTOR_SPD_FB_PIN) == !init) && (cpu_hal_get_cycle_count() - t_enter < time_out_ticks))
		{
		}
		int64_t t_end = cpu_hal_get_cycle_count();
		// directly return if timed out
		return (t_end - t_enter >= time_out_ticks) ? 0U : t_end - t_start;
	}

	void Active_brake()
	{
		Brake_mode = true;
		Last_set_speed = 0U;
		setbit(MOTOR_BRAKE_PIN);
	}

	void Active_brake_release()
	{
		Brake_mode = false;
		clrbit(MOTOR_BRAKE_PIN);
	}

	bool Get_brake_status()
	{
		return Brake_mode;
	}

#if MOTOR_OVERDRIVE_ENABLED
	void Set_overdrive(bool state)
	{
		if (state != Overdrive_state)
		{
			Overdrive_state = state;
			Config_register(22, state ? Overdrive_config : Default_config[20]);

			if (state)
			{
				Set_speed(math::fast::clip(Last_set_speed, Compute_throttle(Min_thrust_overdrive), Compute_throttle(Max_thrust_overdrive)));
			}
			else
			{
				Set_speed(math::fast::clip(Last_set_speed, Compute_throttle(Min_thrust), Compute_throttle(Max_thrust)));
			}
		}
	}
#endif

	void Set_thrust(const float thrust)
	{
#if MOTOR_OVERDRIVE_ENABLED
		if (Overdrive_state)
		{
			// exit overdrive mode when thrust value is too small.
			// Or we might fail to start up again.
			if (thrust >= Min_thrust_overdrive)
			{
				Set_speed(Compute_throttle(math::fast::clip(thrust, Min_thrust_overdrive, Max_thrust_overdrive)));
				return;
			}

			Set_overdrive(false);
		}
#endif

		Set_speed(Compute_throttle(math::fast::clip(thrust, Min_thrust, Max_thrust)));
	}

	bool Get_overdrive_mode()
	{
		return Overdrive_state;
	}
} // namespace Motor