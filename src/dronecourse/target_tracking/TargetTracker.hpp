/**
 * @file TargetTracker.hpp
 * Class to track moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#pragma once
// #include <poll.h>
#include <px4_posix.h>
#include <uORB/Subscription.hpp>

#include "Kalman.hpp"

class TargetTracker
{
	// ------------------------------------------------
	// TODO Create static const int variables
	//      for number of states variables and
	//      measurement variables and set their values
	// ------------------------------------------------
	static const int _nb_states = 6;
	static const int _nb_meas_vars = 3;

public:
	TargetTracker(float dt);

	void update();

private:
	void update_parameters();
	void publish_filtered_target_position(const matrix::Vector<float, 6> &pos_vel,
					      const matrix::Vector<float, 6> &variance);

	// --------------------------------------------
	// TODO define Kalman filter member variable
	//      using M and N as template parameters
	// ---------------------------------------------
	KalmanFilter<_nb_states,_nb_meas_vars> _kalman_filter;

	// --------------------------------------------
	// TODO define a vector containing
	//      the system noise standard deviation _w
	// ---------------------------------------------
	matrix::Vector<float, _nb_states> _w;

	// --------------------------------------------
	// TODO create onboard parameter handle arrays
	// --------------------------------------------
	param_t _p_kal_sys_noise[_nb_states];

	// ------------------------------------------
	// TODO add uORB subscriptions for
	// target_position_ned
	// ------------------------------------------
	int _sub_target_position_ned;

	// -------------------------------------------
	// TODO create handle for uORB publication of
	//      of topic target_position_ned_filtered
	// -------------------------------------------
	orb_advert_t _pub_target_position_ned_filtered;

};


