/**
 * @file PositionCtrl.cpp
 * Class to convert position command to velocity command
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Matteo Macchini <matteo.macchini@epfl.ch>
 * @author Fabian Schilling <fabian.schilling@epfl.ch>
 */

#include "VelocityCtrl.hpp"
#include <matrix/math.hpp>

VelocityCtrl::VelocityCtrl(GimbalCtrl &gimbal) :
	BaseCtrl(gimbal)
{
}

void VelocityCtrl::update()
{
	send_velocity_command(_goal_vel);
}

bool VelocityCtrl::is_goal_reached()
{
	// TODO: check whether velocity setpoint matches estimated velocity
	return false;
}