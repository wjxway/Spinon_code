#include "EKFCore.hpp"
#include <FastMath.hpp>
#include <dspm_mult.h>
#include <mat.h>

using namespace math::fast;

namespace EKF
{
	namespace
	{
		// drone mass in g
		constexpr float drone_mass = 20.16f;
		// gravity constant in cm/s^2
		constexpr float gravity = 980.0f;
		// distance between thrust point and center of geometry in cm
		constexpr float thrust_offset = 1.3f;
		// moment of inertia around rotation axis in g.cm^2
		constexpr float I_0 = 150.0f;
		// drone's vertical viewing angle's offset in degrees
		constexpr float psi_s = -0.6f;
		// angle difference between sensor and motor in degrees
		constexpr float motor_angle_offset = 10.0f;

		// coefficient that connects thrust with angular acceleration
		constexpr float k_Fa = 50.0f;
		// coefficient that connects angular velocity with angular acceleration
		constexpr float k_drag = 50.0f / 360;
		// coefficient that connects motor acceleration with angular acceleration
		constexpr float k_inertia = 0.04f * 360;

		// degree to rad
		constexpr float degree = float(M_PI) / 180.0F;

		// default angular velocity
		constexpr float default_angular_velocity = 20.0f * 360.0f;

		// switch between data group 0 and group 1
		// solve concurrency issue, always make sure there's one ready for access
		size_t avail_index = 0;

		// corresponding time of the drone's state
		int64_t drone_state_time[2] = {0};

		// state of the drone
		float drone_state[2][dim_state] = {0};

		// covariance matrix of the drone
		float drone_cov[2][dim_state][dim_state] = {0};

		// covariance increase rate
		// note that here we assume it's a diagonal matrix, so we only record the diagonal
		constexpr float Cov_increase_rate[dim_state] = {1.0f, 1.0f, 4.0f, 4.0f, 4.0f, 4.0f, 100.0f, 4.0f, 10000.0f, 1.0f, 100000.0f};

		/**
		 * @brief extrapolate the state and covariance
		 *
		 * @param act actuation vector
		 * @param t_target the time we are trying to extrapolate to
		 *
		 * @note this function will directly change the state and cov!
		 */
		void Extrapolate_state(const Act_vector act, const int64_t t_target)
		{
			size_t free_index = 1 - avail_index;
			auto dsa = drone_state[avail_index], dsf = drone_state[free_index];
			auto cova = drone_cov[avail_index], covf = drone_cov[free_index];

			float dt = 1.0e-6f * (t_target - drone_state_time[avail_index]);
			// cap dt to 1s... just to make things not outrageously high
			if(dt>1.0f)
			{
				dt=1.0f;
			}

			// state extrapolation
			float C1 = 4.0f * thrust_offset * gravity * dt / (I_0 * dsa[omega] * (float(M_PI) * float(M_PI) * degree * degree));
			float C2 = act.F_a * degree * gravity * dt / drone_mass;

			dsf[gamma_x] = dsa[gamma_x] + C1 * act.F_y;
			dsf[gamma_y] = dsa[gamma_y] - C1 * act.F_x;
			dsf[v_x] = dsa[v_x] + dsa[gamma_x] * C2;
			dsf[v_y] = dsa[v_y] + dsa[gamma_y] * C2;
			dsf[v_z] = dsa[v_z] + (act.F_a / drone_mass - 1) * gravity * dt;

			dsf[x] = dsa[x] + dsa[v_x] * dt;
			dsf[y] = dsa[y] + dsa[v_y] * dt;
			dsf[z] = dsa[z] + dsa[v_z] * dt;

			dsf[omega] = dsa[omega] + (k_Fa * act.F_a - k_drag * dsa[omega] + k_inertia * act.F_beta - dsa[offset]) * dt;
			dsf[theta] = dsa[theta] + dsa[omega] * dt;

			dsf[offset] = dsa[offset];

			// covariance extrapolation
			// only do so when dt>0, when dt<0, we do not decrase covariance
			// note that dt<0 only occur at rare occasion when two localization timing msg1 and msg2 are received at different time,
			// such that msg1's last reception event is earlier than msg2's, but the average timing of left and right sensor is the other way around.
			if (dt > 0.0f)
			{
				float C3 = -C1 * act.F_y / dsa[omega];
				float C4 = C1 * act.F_x / dsa[omega];
				float C5 = 1.0f - k_drag * dt;

				// now we compute F.cov.F^T+cov_add
				// exploit the sparse nature of F to speed up the computation

				// covtemp can be static as it's re-initialized every time.
				static float covtemp[dim_state][dim_state] = {0};

				// F.cov
				for (int i = 0; i < dim_state; i++)
				{
					covtemp[gamma_x][i] = cova[gamma_x][i] + C3 * cova[omega][i];
					covtemp[gamma_y][i] = cova[gamma_y][i] + C4 * cova[omega][i];
					covtemp[v_x][i] = cova[v_x][i] + C2 * cova[gamma_x][i];
					covtemp[v_y][i] = cova[v_y][i] + C2 * cova[gamma_y][i];
					covtemp[x][i] = cova[x][i] + dt * cova[v_x][i];
					covtemp[y][i] = cova[y][i] + dt * cova[v_y][i];
					covtemp[v_z][i] = cova[v_z][i];
					covtemp[z][i] = cova[z][i] + dt * cova[v_z][i];
					covtemp[omega][i] = C5 * cova[omega][i] - dt * cova[offset][i];
					covtemp[theta][i] = cova[theta][i] + dt * cova[omega][i];
					covtemp[offset][i] = cova[offset][i];
				}
				// (F.cov).F^T
				for (int i = 0; i < dim_state; i++)
				{
					covf[i][gamma_x] = covtemp[i][gamma_x] + C3 * covtemp[i][omega];
					covf[i][gamma_y] = covtemp[i][gamma_y] + C4 * covtemp[i][omega];
					covf[i][v_x] = covtemp[i][v_x] + C2 * covtemp[i][gamma_x];
					covf[i][v_y] = covtemp[i][v_y] + C2 * covtemp[i][gamma_y];
					covf[i][x] = covtemp[i][x] + dt * covtemp[i][v_x];
					covf[i][y] = covtemp[i][y] + dt * covtemp[i][v_y];
					covf[i][v_z] = covtemp[i][v_z];
					covf[i][z] = covtemp[i][z] + dt * covtemp[i][v_z];
					covf[i][omega] = C5 * covtemp[i][omega] - dt * covtemp[i][offset];
					covf[i][theta] = covtemp[i][theta] + dt * covtemp[i][omega];
					covf[i][offset] = covtemp[i][offset];
				}
				// F.cov.F^T+cov_add
				for (int i = 0; i < dim_state; i++)
				{
					covf[i][i] += Cov_increase_rate[i] * dt;
				}
			}

			drone_state_time[free_index] = t_target;
		}

		/**
		 * @brief correct the state and covariance based on measurements
		 *
		 * @note this apply only to the case when only angle measurement are available. i.e. d=0
		 * @note be aware that this function should operate on the free state and covariance
		 */
		void Correct_state_angle_only(const Meas_vector meas)
		{
			size_t free_index = 1 - avail_index;
			auto dsf = drone_state[free_index];
			auto covf = drone_cov[free_index];

			float dr_isq = 1.0f / (square(meas.x_0 - dsf[x]) + square(meas.y_0 - dsf[y]));

			// err is error vector, obs_cov is observation covariance matrix
			float err = (dsf[theta] + 10.0f) / 360.0f - atan2f(meas.y_0 - dsf[y], meas.x_0 - dsf[x]) / (2.0F * float(M_PI));
			err = (round(err) - err) * 2.0F * float(M_PI);
			float obs_cov = square(meas.sigma_0) * dr_isq / 3.0f + square(sigma_theta * degree);

			// observation matrix's 5th and 6th elements
			float H4 = (dsf[y] - meas.y_0) * dr_isq, H5 = (meas.x_0 - dsf[x]) * dr_isq, H9 = degree;

			// compute kalman gain K
			float K[dim_state] = {0};

			// compute cov.H^T
			for (int i = 0; i < dim_state; i++)
			{
				K[i] = H4 * covf[i][x] + H5 * covf[i][y] + H9 * covf[i][theta];
			}

			// compute (H.cov.H^T + R)^-1
			float temp1 = 1.0f / (H4 * K[x] + H5 * K[y] + H9 * K[theta] + obs_cov);

			// K = cov.H^T.(H.cov.H^T + R)^-1
			for (int i = 0; i < dim_state; i++)
			{
				K[i] *= temp1;
			}

			// state correct
			// state += K.y
			for (int i = 0; i < dim_state; i++)
			{
				dsf[i] += K[i] * err;
			}

			// I-K.H
			float temp2[dim_state][dim_state] = {0}, temp3[dim_state][dim_state];

			for (int i = 0; i < dim_state; i++)
			{
				temp2[i][i] = 1.0f;
				temp2[i][x] -= H4 * K[i];
				temp2[i][y] -= H5 * K[i];
				temp2[i][theta] -= H9 * K[i];
			}

			dspm_mult_f32_ae32(temp2[0], covf[0], temp3[0], dim_state, dim_state, dim_state);

			// transpose temp2
			for (int i = 0; i < dim_state; i++)
			{
				for (int j = i + 1; j < dim_state; j++)
				{
					std::swap(temp2[i][j], temp2[j][i]);
				}
			}

			dspm_mult_f32_ae32(temp3[0], temp2[0], covf[0], dim_state, dim_state, dim_state);

			// (I-K.H).cov.(I-K.H)^T + K.R.K^T
			for (int i = 0; i < dim_state; i++)
			{
				for (int j = 0; j < dim_state; j++)
				{
					covf[i][j] += obs_cov * K[i] * K[j];
				}
			}
		}

		/**
		 * @brief correct the state and covariance based on measurements
		 *
		 * @note this apply only to the case when all measurement are available. i.e. d!=0
		 */
		void Correct_state_all_meas(const Meas_vector meas)
		{
			size_t free_index = 1 - avail_index;
			auto dsf = drone_state[free_index];
			auto covf = drone_cov[free_index];

			float dr = norm(meas.x_0 - dsf[x], meas.y_0 - dsf[y]);
			float dr_isq = 1.0f / square(dr);

			// err is error vector, obs_cov is observation covariance matrix
			float err[3] = {(dsf[theta] + 10.0f) / 360.0f - atan2f(meas.y_0 - dsf[y], meas.x_0 - dsf[x]) / (2.0F * float(M_PI)),
							dr - meas.d,
							(meas.z_0 - dsf[z]) / dr - tan((meas.phi - psi_s) * degree)};
			err[0] = (round(err[0]) - err[0]) * 2.0F * float(M_PI);
			float obs_cov[3][3] = {{square(meas.sigma_0) * dr_isq / 3.0f + square(sigma_theta * degree), 0, 0},
								   {0, square(meas.sigma_0) / 3.0f + square(sigma_d(meas.d)), (dsf[z] - meas.z_0) * square(meas.sigma_0) * dr_isq / 3.0f},
								   {0, (dsf[z] - meas.z_0) * square(meas.sigma_0) * dr_isq / 3.0f,
									square(meas.sigma_0 * dr_isq) * (square(dr) + square(meas.z_0 - dsf[z])) / 3.0f + square(sigma_phi * degree / square(cosf((meas.phi - psi_s) * degree)))}};

			// observation matrix's elements
			float H04 = (dsf[y] - meas.y_0) * dr_isq,
				  H05 = (meas.x_0 - dsf[x]) * dr_isq,
				  H09 = degree;
			float H14 = (meas.x_0 - dsf[x]) / dr,
				  H15 = (meas.y_0 - dsf[y]) / dr;
			float H24 = H14 * (dsf[z] - meas.z_0) * dr_isq,
				  H25 = H15 * (dsf[z] - meas.z_0) * dr_isq,
				  H27 = 1.0f / dr;

			// compute kalman gain K
			float temp0[dim_state][3] = {0};

			// compute cov.H^T
			for (int i = 0; i < dim_state; i++)
			{
				temp0[i][0] = H04 * covf[i][x] + H05 * covf[i][y] + H09 * covf[i][theta];
				temp0[i][1] = H14 * covf[i][x] + H15 * covf[i][y];
				temp0[i][2] = H24 * covf[i][x] + H25 * covf[i][y] + H27 * covf[i][z];
			}

			// compute (H.cov.H^T + R)
			float temp1[3][3] = {0}, temp2[3][3] = {0};

			for (int i = 0; i < 3; i++)
			{
				temp1[0][i] = H04 * temp0[x][i] + H05 * temp0[y][i] + H09 * temp0[theta][i];
				temp1[1][i] = H14 * temp0[x][i] + H15 * temp0[y][i];
				temp1[2][i] = H24 * temp0[x][i] + H25 * temp0[y][i] + H27 * temp0[z][i];

				for (int j = 0; j < 3; j++)
				{
					temp1[j][i] += obs_cov[j][i];
				}
			}

			// inverse it!
			float det = temp1[0][0] * (temp1[1][1] * temp1[2][2] - temp1[1][2] * temp1[2][1]) - temp1[0][1] * (temp1[1][0] * temp1[2][2] - temp1[1][2] * temp1[2][0]) + temp1[0][2] * (temp1[1][0] * temp1[2][1] - temp1[1][1] * temp1[2][0]);

			if (det == 0.0f)
			{
				for (int i = 0; i < 3; i++)
				{
					temp2[i][i] = 1000000.0f;
				}
			}
			else
			{
				float invDet = 1.0f / det;

				temp2[0][0] = (temp1[1][1] * temp1[2][2] - temp1[1][2] * temp1[2][1]) * invDet;
				temp2[0][1] = (temp1[0][2] * temp1[2][1] - temp1[0][1] * temp1[2][2]) * invDet;
				temp2[0][2] = (temp1[0][1] * temp1[1][2] - temp1[0][2] * temp1[1][1]) * invDet;
				temp2[1][0] = (temp1[1][2] * temp1[2][0] - temp1[1][0] * temp1[2][2]) * invDet;
				temp2[1][1] = (temp1[0][0] * temp1[2][2] - temp1[0][2] * temp1[2][0]) * invDet;
				temp2[1][2] = (temp1[0][2] * temp1[1][0] - temp1[0][0] * temp1[1][2]) * invDet;
				temp2[2][0] = (temp1[1][0] * temp1[2][1] - temp1[1][1] * temp1[2][0]) * invDet;
				temp2[2][1] = (temp1[0][1] * temp1[2][0] - temp1[0][0] * temp1[2][1]) * invDet;
				temp2[2][2] = (temp1[0][0] * temp1[1][1] - temp1[0][1] * temp1[1][0]) * invDet;
			}

			// K = cov.H^T.(H.cov.H^T + R)^-1
			float K[dim_state][3];

			dspm_mult_f32_ae32(temp0[0], temp2[0], K[0], dim_state, 3, 3);

			// state correct
			// state += K.y
			for (int i = 0; i < dim_state; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					dsf[i] += K[i][j] * err[j];
				}
			}

			// I-K.H
			float temp3[dim_state][dim_state] = {0}, temp4[dim_state][dim_state];

			for (int i = 0; i < dim_state; i++)
			{
				temp3[i][i] = 1.0f;
				temp3[i][x] -= K[i][0] * H04 + K[i][1] * H14 + K[i][2] * H24;
				temp3[i][y] -= K[i][0] * H05 + K[i][1] * H15 + K[i][2] * H25;
				temp3[i][z] -= K[i][2] * H27;
				temp3[i][theta] -= K[i][0] * H09;
			}

			// (I-K.H).cov
			dspm_mult_f32_ae32(temp3[0], covf[0], temp4[0], dim_state, dim_state, dim_state);

			// (I-K.H).cov.(I-K.H)^T
			for (int i = 0; i < dim_state; i++)
			{
				for (int j = i + 1; j < dim_state; j++)
				{
					std::swap(temp3[i][j], temp3[j][i]);
				}
			}
			dspm_mult_f32_ae32(covf[0], temp4[0], temp3[0], dim_state, dim_state, dim_state);

			// K.R
			float temp5[dim_state][3], temp6[dim_state][dim_state], KT[3][dim_state];
			dspm_mult_f32_ae32(K[0], obs_cov[0], temp5[0], dim_state, 3, 3);

			for (int i = 0; i < dim_state; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					KT[j][i] = K[i][j];
				}
			}

			dspm_mult_f32_ae32(temp5[0], KT[0], temp6[0], dim_state, 3, dim_state);

			// (I-K.H).cov.(I-K.H)^T + K.R.K^T
			for (int i = 0; i < dim_state; i++)
			{
				for (int j = 0; j < dim_state; j++)
				{
					covf[i][j] += temp6[i][j];
				}
			}
		}

		/**
		 * @brief correct the state and covariance based on measurements
		 *
		 * @param meas measurement vector
		 *
		 * @note this function will directly change the state and cov!
		 */
		void Correct_state(const Meas_vector meas)
		{
			if (meas.d != 0.0f)
			{
				Correct_state_all_meas(meas);
			}
			else
			{
				Correct_state_angle_only(meas);
			}
		}

	} // anonymous namespace

	float sigma_d(const float d)
	{
		return 0.0033f * d * d;
	}

	void Init_state()
	{
		avail_index = 0;

		constexpr float init_state[dim_state] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, default_angular_velocity, 0.0f};
		for (int i = 0; i < dim_state; i++)
		{
			drone_state[0][i] = init_state[i];
		}

		for (int i = 0; i < dim_state; i++)
		{
			for (int j = 0; j < dim_state; j++)
			{
				drone_cov[0][i][j] = 0.0f;
			}
			drone_cov[0][i][i] = 1000000.0f;
		}

		drone_state_time[avail_index] = 0;
	}

	void Set_state(const int64_t tt, const float *st, const float (*co)[dim_state])
	{
		size_t free_index = 1 - avail_index;

		drone_state_time[free_index] = tt;

		for (int i = 0; i < dim_state; i++)
		{
			drone_state[free_index][i] = st[i];
		}

		for (int i = 0; i < dim_state; i++)
		{
			for (int j = 0; j < dim_state; j++)
			{
				drone_cov[free_index][i][j] = co[i][j];
			}
		}

		avail_index = free_index;
	}

	State_vector Get_state(const int64_t t_target, const float Fa, float* std_ptr)
	{
		auto avail_index_temp = avail_index;
		State_vector temp;
		float pos_std;

		do
		{
			auto dsa = drone_state[avail_index_temp];
			auto cova = drone_cov[avail_index_temp];

			for (int i = 0; i < dim_state; i++)
			{
				temp.state[i] = dsa[i];
			}

			float dt = 1.0e-6f * (t_target - drone_state_time[avail_index_temp]);

			float C2 = Fa * degree / drone_mass * gravity * dt;

			temp.state[v_x] += dsa[gamma_x] * C2;
			temp.state[v_y] += dsa[gamma_y] * C2;
			temp.state[v_z] += (Fa / drone_mass - 1) * gravity * dt;

			temp.state[x] += dsa[v_x] * dt;
			temp.state[y] += dsa[v_y] * dt;
			temp.state[z] += dsa[v_z] * dt;

			temp.state[omega] += (k_Fa * Fa - k_drag * dsa[omega] - dsa[offset]) * dt;
			temp.state[theta] += dsa[omega] * dt;

			pos_std=sqrtf(cova[x][x]+cova[y][y]+cova[z][z]);
		} while (avail_index != avail_index_temp);

		if(std_ptr)
		{
			*std_ptr = pos_std;
		}

		return temp;
	}

    float Get_last_angular_velocity()
    {
        return drone_state[avail_index][omega];
    }

    void Iterate_state(const Act_vector act, const Meas_vector meas)
	{
		Extrapolate_state(act, meas.time);
		Correct_state(meas);
		avail_index = 1 - avail_index;
	}
} // EKF namespace