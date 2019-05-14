/**
 * @file TargetFollower.hpp
 * Class to follow moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#pragma once
#include "PositionCtrl.hpp"
#include "GimbalCtrl.hpp"
#include <uORB/topics/target_position_ned_filtered.h>
#include <uORB/topics/vehicle_land_detected.h>

class TargetFollower : public PositionCtrl
{


public:
	TargetFollower(GimbalCtrl &gimbal);

	~TargetFollower() {;};

	void update();

	/**
	* Check if goal is reached
	* true if target_pos and current_pos and _target_vel and current_vel are close enough
	*
	* @return  True if distance to goal smaller than POS_ACCEPT_RAD
	*/
	virtual bool is_goal_reached() const;

private:
	void update_subscriptions();
	void update_parameters();

	int _target_position_ned_filtered_sub;
	int _local_pos_sub;
	int _vehicle_land_detected_sub;

	target_position_ned_filtered_s _target_position_ned_filtered;
	vehicle_land_detected_s _vehicle_land_detected;

	bool _target_detected;
	bool _force_landing;

	uint64_t _last_iteration_time;
	float _last_gimbal_yaw;

	GimbalCtrl _gimbal;

	matrix::Vector3f _current_pos;
};


