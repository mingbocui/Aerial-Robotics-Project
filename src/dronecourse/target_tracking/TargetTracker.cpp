/**
 * @file TargetTracker.hpp
 * Class to track moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "TargetTracker.hpp"
#include <uORB/topics/target_position_ned.h>
#include <uORB/topics/target_position_ned_filtered.h>
#include <drivers/drv_hrt.h>
#include <float.h>

TargetTracker::TargetTracker(float dt)
{
	// ----------------------------------------------
	// TODO set up the arrays of parameter handles
	//      _p_kal_sys_noise
	// ----------------------------------------------
	_p_kal_sys_noise[0] = param_find("SYSTEM_NOISE_X");
	_p_kal_sys_noise[1] = param_find("SYSTEM_NOISE_Y");
	_p_kal_sys_noise[2] = param_find("SYSTEM_NOISE_Z");
	_p_kal_sys_noise[3] = param_find("SYSTEM_NOISE_VX");
	_p_kal_sys_noise[4] = param_find("SYSTEM_NOISE_VY");
	_p_kal_sys_noise[5] = param_find("SYSTEM_NOISE_VZ");

	// ----------------------------------------------
	// TODO Check validity of parameter handles
	// ----------------------------------------------
	update_parameters();

	// ----------------------------------
	// TODO initialize kalman filter
	// ----------------------------------
	// Dynamic matrix F
	float f_init[_nb_states * _nb_states] = {
		0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 
		0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
	};
	matrix::SquareMatrix<float, _nb_states> f(f_init);

	// Design (measurement) matrix H
	float h_init[_nb_meas_vars * _nb_states] = {
		1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
		0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
		0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f 
	};
	matrix::Matrix<float, _nb_meas_vars, _nb_states> h(h_init);

	// Initial state x(k=0)
	matrix::Vector<float, _nb_states> x0;

	// Standard deviation of initial state
	float p0_init[_nb_states] = {
		70.0f, 70.0f, 5.0f, 10.0f, 10.0f, 1.0f
	};
	matrix::Vector<float, _nb_states> p0(p0_init);
	
	_kalman_filter.init(f, _w, h, x0, p0, dt);

	// ----------------------------------
	// TODO subscribe to uORB topics:
	//  target_position_ned
	// ----------------------------------
	_sub_target_position_ned = orb_subscribe(ORB_ID(target_position_ned));

	// ----------------------------------------------------
	// TODO initialize the handle of your uORB publication
	//      of target_position_ned_filtered to 'nullptr'
	// ----------------------------------------------------
	_pub_target_position_ned_filtered = nullptr;
}

void TargetTracker::update()
{
	// ------------------------------------------------------------
	// TODO call update_parameters() and execute the prediction of
	//      the Kalman filter
	// ------------------------------------------------------------
	update_parameters();

	_kalman_filter.predict();

	bool new_measure = false;

	// ------------------------------------------------------------
	// TODO check if we have a new target_position_ned message
	// and set new_measure accordingly
	// ------------------------------------------------------------
	orb_check(_sub_target_position_ned, &new_measure);

	if (new_measure) {
		// --------------------------------------------------------------------
		// TODO copy message content to a local variable
		// --------------------------------------------------------------------
		target_position_ned_s target_position_ned;
		orb_copy(ORB_ID(target_position_ned), _sub_target_position_ned, &target_position_ned);

		float measurement_array[3] = {target_position_ned.x, target_position_ned.y, target_position_ned.z};

		_kalman_filter.correct(matrix::Vector<float, _nb_meas_vars>(measurement_array), 
							   matrix::SquareMatrix<float, _nb_meas_vars>(target_position_ned.var));
	}

	// -------------------------------------------------------------------------
	// TODO call publish_filtered_target_position(...) to publish
	//      filtered the filtered target position
	// -------------------------------------------------------------------------
	matrix::Vector<float, 6> state_estimate = _kalman_filter.get_state_estimate();
	matrix::Vector<float, 6> state_variance = _kalman_filter.get_state_variances();
	publish_filtered_target_position(state_estimate, state_variance);
}

void TargetTracker::update_parameters()
{
	// ---------------------------------------------------
	// TODO get system noise parameter values and find
	// out if they have changed
	// ---------------------------------------------------
	bool changed = false;
	float kal_sys_noise[_nb_states];

	for (int i = 0; i < _nb_states; ++i)
	{
		param_get(_p_kal_sys_noise[i], &kal_sys_noise[i]);
		if(fabsf(kal_sys_noise[i] - _w(i)) > FLT_EPSILON) {
			changed = true;
			_w(i) = kal_sys_noise[i];
		}
	}
	
	// ---------------------------------------------------
	// TODO if system noise parameter values have changed,
	// updated system noise of Kalman
	// ---------------------------------------------------
	if(changed) {
		_kalman_filter.set_system_noise(_w);
	}
}

void TargetTracker::publish_filtered_target_position(const matrix::Vector<float, 6> &pos_vel,
		const matrix::Vector<float, 6> &variance)
{
	// -------------------------------------------------------------------------------------------
	// TODO create local variable of type struct target_position_ned_filtered_s
	//      set all fields to 0 and then fill fields
	// -------------------------------------------------------------------------------------------
	target_position_ned_filtered_s target_filtered;
	target_filtered.x = pos_vel(0);
	target_filtered.y = pos_vel(1);
	target_filtered.z = pos_vel(2);

	target_filtered.vx = pos_vel(3);
	target_filtered.vy = pos_vel(4);
	target_filtered.vz = pos_vel(5);

	target_filtered.var_x = variance(0);
	target_filtered.var_y = variance(1);
	target_filtered.var_z = variance(2);

	target_filtered.var_vx = variance(3);
	target_filtered.var_vy = variance(4);
	target_filtered.var_vz = variance(5);
	target_filtered.timestamp = hrt_absolute_time();

	target_filtered.target_id = 0;

	// -------------------------------------------------------------------------------------------
	// TODO publish your target_position_ned_s message
	// -------------------------------------------------------------------------------------------
	if (_pub_target_position_ned_filtered == nullptr) {
		_pub_target_position_ned_filtered = orb_advertise(ORB_ID(target_position_ned_filtered), &target_filtered);
	} else {
		orb_publish(ORB_ID(target_position_ned_filtered), _pub_target_position_ned_filtered, &target_filtered);
	}
}