/**
 * @file EKFCore.hpp
 * @brief core for EKF filtering
 */
#ifndef EKFCORE_HPP__
#define EKFCORE_HPP__

#include <Arduino.h>
#include <cstdint>

namespace EKF
{
	// dimension of state vector
	constexpr size_t dim_state = 11;

	/**
	 * @brief enum that stores the order of parameters
	 *
	 * @note you can use it in the way like state[state_para::v_x]
	 */
	enum state_para : int
	{
		gamma_x = 0,
		gamma_y = 1,
		v_x = 2,
		v_y = 3,
		x = 4,
		y = 5,
		v_z = 6,
		z = 7,
		omega = 8,
		theta = 9,
		offset = 10
	};

	/**
	 * @brief struct to store the drone's state
	 *
	 * @details	ordered in the form {\gamma_x, \gamma_y, v_x, v_y, x, y, vz, z, \omega, \theta, Fa\omega offset}
	 *  \gamma_x,\gamma_y are the projection of angular momentum unit vector on the horizontal plane. Unit in degree.
	 *  v_x, v_y, v_z are the velocity. Unit in cm/s.
	 *  x, y, z are position. Unit in cm.
	 *  \omega is the angular velocity. Unit in deg/s.
	 *  \theta is the heading angle of the sensor. Unit in deg.
	 *  Fa\omega offset is the constant angular velocity acceleration (compensation). Unit in deg/s^2.
	 */
	struct State_vector
	{
		float state[dim_state];
	};

	struct Cov_matrix
	{
		float cov[dim_state][dim_state];
	};

	/**
	 * @brief actuation vector
	 */
	struct Act_vector
	{
		float F_a;	  // average thrust. Unit in gf.
		float F_x;	  // differential thrust in X direction. Unit in gf.
		float F_y;	  // differential thrust in Y direction. Unit in gf.
		float F_beta; // driving torque due to acceleration.
	};

	// standard deviation of angle measurement
	constexpr float sigma_theta = 1.5f;
	// standard deviation of elevation angle measurement
	constexpr float sigma_phi = 2.0f;
	// standard deviation function of distance measurements
	float sigma_d(const float d);

	// measurement vector
	struct Meas_vector
	{
		int64_t time; // when this measurement is obtained

		float x_0;	   // position of drone in X
		float y_0;	   // in Y
		float z_0;	   // in Z
		float sigma_0; // standard deviation of position

		float d = 0.0f; // distance, if = 0 then means only horizontal angle information is useful.

		float phi = 0.0f; // elevation angle
	};

	/**
	 * @brief initialize the state and covariance
	 */
	void Init_state();

	/**
	 * @brief manually set state and covariance matrix
	 *
	 * @param tt is the time of state
	 * @param st is the state vector
	 * @param co is the covariance of state
	 *
	 * @warning this is not write thread safe!
	 * @warning this function will not do validity check for covariance!
	 */
	void Set_state(const int64_t tt, const float *st, const float (*co)[dim_state]);

	/**
	 * @brief get the state of the drone at a certain time
	 *
	 * @param t_target time to predect to.
	 * @param Fa input thrust
	 * @param std_ptr pointer where we store a float type standard deviation of position in. can be nullptr if this info is not needed.
	 *
	 * @return State_vector the predicted state at that time.
	 */
	State_vector Get_state(const int64_t t_target, const float Fa, float *std_ptr = nullptr);

	/**
	 * @brief get the angular velocity at last update, unit in degree/s
	 *
	 * @note this is only for EKFTask, you should not call it for most of the time!
	 */
	float Get_last_angular_velocity();

	/**
	 * @brief execute a full cycle of extrapolate - correct
	 *
	 * @param act actuation vector
	 * @param meas measurement vector
	 *
	 * @note this function will directly change the state and cov!
	 */
	void Iterate_state(const Act_vector act, const Meas_vector meas);

} // EKF namespace

#endif
