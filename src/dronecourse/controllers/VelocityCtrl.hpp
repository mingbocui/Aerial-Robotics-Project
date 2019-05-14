/**
 * @file PositionCtrl.hpp
 * Class to convert position command to velocity command
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Matteo Macchini <matteo.macchini@epfl.ch>
 * @author Fabian Schilling <fabian.schilling@epfl.ch>
 */

#pragma once
#include <px4_posix.h>
#include "BaseCtrl.hpp"

class VelocityCtrl : public BaseCtrl
{

public:

	/**
	 * Constructor
	 *
	 * @param gimbal     Gimbal controller to control pitch and yaw of gimbal
	 */
	VelocityCtrl(GimbalCtrl &gimbal);

	~VelocityCtrl() {};

	/**
	 * Updates the controller
	 * i.e. sends velocity command
	 */
	void update();

	/**
	 * Check if velocity setpoint is reached.
	 *
	 * @return true if velocity setpoint is close to estimated velocity
	 */
	bool is_goal_reached();

	/**
	 * Sets goal velocity of this controller
	 *
	 * @param pos   Goal position in local frame
	 */
	void set_velocity_command(matrix::Vector3f &vel) {_goal_vel = vel;};

private:

	void update_subscriptions();

	/** Goal velocity in local frame */
	matrix::Vector3f _goal_vel;
};
