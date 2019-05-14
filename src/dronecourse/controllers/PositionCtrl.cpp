/**
 * @file PositionCtrl.cpp
 * Class to convert position command to velocity command
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 * @author Arthur Gay <arthur.gay@epfl.ch>
 * @author Fabian Schilling <fabian.schilling@epfl.ch>
 */

#include "PositionCtrl.hpp"
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/Subscription.hpp>

		//PX4_INFO("Info message with integer %i", i)
		//PX4_WARN("Warning message with double %f", d)

PositionCtrl::PositionCtrl(GimbalCtrl &gimbal) :
	BaseCtrl(gimbal),
	_goal_pos(0.0f, 0.0f, 0.0f),
	_current_pos(0.0f, 0.0f, 0.0f)
{
	// --------------------------------------------------------------------------
	// TODO: Add uORB subscription to drone's estimated position
	// --------------------------------------------------------------------------
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	// --------------------------------------------------------------------------
	// TODO: Read gain and acceptance radius parameters
	// --------------------------------------------------------------------------
	_p_pos_gain = param_find("POS_GAIN");
	_p_pos_accept_rad = param_find("POS_ACCEPT_RAD");
}

void PositionCtrl::update()
{
	update_subscriptions();
	update_parameters();

	//print_current_position();
	_position_error = compute_position_error(_goal_pos, _current_pos);

	matrix::Vector3f velocity_command = compute_velocity_command(_position_error, get_position_gain());

	send_velocity_command(velocity_command);
}

void PositionCtrl::print_current_position()
{
	// --------------------------------------------------------------------------
	// TODO: Print current position to console to verify working subscription
	// --------------------------------------------------------------------------
	PX4_INFO("Current position : %8.4f\t%8.4f\t%8.4f\t", (double)_current_pos(0), (double)_current_pos(1), (double)_current_pos(2));
}

matrix::Vector3f PositionCtrl::compute_position_error(matrix::Vector3f goal_pos, matrix::Vector3f current_pos)
{
	// --------------------------------------------------------------------------
	// TODO: Calculate the target vector (vector from drone to goal position)
	// --------------------------------------------------------------------------
	matrix::Vector3f position_error = goal_pos - current_pos;

	return position_error;
}

matrix::Vector3f PositionCtrl::compute_velocity_command(matrix::Vector3f position_error, float position_gain)
{
	// --------------------------------------------------------------------------
	// TODO: Calculate velocity command using proportional gain
	// --------------------------------------------------------------------------

	matrix::Vector3f _target_vector = position_error * position_gain;// to complete

	return _target_vector;
}

bool PositionCtrl::is_goal_reached()
{
	// --------------------------------------------------------------------------
	// TODO: Implement the decision when goal is reached
	// --------------------------------------------------------------------------
	float dist = sqrtf(_position_error(0)*_position_error(0) +
					   _position_error(1)*_position_error(1) +
					   _position_error(2)*_position_error(2));

	if(dist < _pos_accept_rad)
		return true;
	else
		return false;
}

void PositionCtrl::update_subscriptions()
{
	bool updated;

	// --------------------------------------------------------------------------
	// TODO: Check if drone's local position topic has been updated
	// --------------------------------------------------------------------------
	orb_check(_local_pos_sub, &updated);

	if (updated) {

		// ------------------------------------------------------------------------
		// TODO: Update current position member variable with new position data
		// ------------------------------------------------------------------------
		vehicle_local_position_s local_pos;
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &local_pos);

		_current_pos(0) = local_pos.x;
		_current_pos(1) = local_pos.y;
		_current_pos(2) = local_pos.z;
	}
}

void PositionCtrl::update_parameters()
{
	// --------------------------------------------------------------------------
	// TODO: Get new parameter values and update corresponding member variables
	// --------------------------------------------------------------------------
	param_get(_p_pos_gain, &_pos_gain);
	param_get(_p_pos_accept_rad, &_pos_accept_rad);
}
