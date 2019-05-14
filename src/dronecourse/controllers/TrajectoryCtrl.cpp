/**
 * @file TrajectoryController.cpp
 * Class to handle predefined flight trajectory
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 * @author Arthur Gay <arthur.gay@epfl.ch>
 */

#include "TrajectoryCtrl.hpp"
#include <drivers/drv_hrt.h>
#include <uORB/topics/dronecourse_waypoint.h>

TrajectoryCtrl::TrajectoryCtrl(GimbalCtrl &gimbal, WaypointNavigator &navigator) :
	PositionCtrl(gimbal),
	_navigator(&navigator),
	_dronecourse_waypoint_pub(nullptr),
	waypoint_published(false),
	_wp_index(0),
	_mission_size(0)
	// --------------------------------------------------------------------------
	// TODO: Initialize your member variables
	// --------------------------------------------------------------------------
{
	matrix::Vector3f waypoint;
	_navigator->waypoint_copy(_wp_index, &waypoint);
	set_position_command(waypoint);
}


void TrajectoryCtrl::update()
{
	// publist waypoint position for logging
	publish_waypoint_list();

	// --------------------------------------------------------------------------
	// TODO: Upate the position controller
	// --------------------------------------------------------------------------
	PositionCtrl::update();
	

	// --------------------------------------------------------------------------
	// TODO: Set the position command for the position controller
	// --------------------------------------------------------------------------
	if(PositionCtrl::is_goal_reached() && _wp_index < _mission_size) {
		_wp_index++;
		if(_wp_index < _mission_size) {
			matrix::Vector3f waypoint;
			
			if(_navigator->waypoint_copy(_wp_index, &waypoint) == _navigator->INVALID_WAYPOINT) {
				PX4_WARN("Unable to copy the waypoint.");
			}
			else {
				set_position_command(waypoint);
			}
		}
		else {
			PX4_WARN("Mission finished.");
		}
	}
}

bool TrajectoryCtrl::is_goal_reached()
{
	// --------------------------------------------------------------------------
	// TODO: Implement the logic for having visited all waypoints
	// --------------------------------------------------------------------------
	if(_wp_index == _mission_size) {
		return true;		
	}
	else {
		return false;
	}
}

void TrajectoryCtrl::publish_waypoint_list(void)
{
	if (not waypoint_published){
		dronecourse_waypoint_s way_p;
	
		way_p.timestamp = hrt_absolute_time();
		matrix::Vector3f waypoint;

		_navigator->waypoint_copy(0, &waypoint);
		way_p.x1 = waypoint(0);
		way_p.y1 = waypoint(1);
		way_p.z1 = waypoint(2);
	
		_navigator->waypoint_copy(1, &waypoint);
		way_p.x2 = waypoint(0);
		way_p.y2 = waypoint(1);
		way_p.z2 = waypoint(2);
	
		_navigator->waypoint_copy(2, &waypoint);
		way_p.x3 = waypoint(0);
		way_p.y3 = waypoint(1);
		way_p.z3 = waypoint(2);

		_mission_size = _navigator->waypoint_count();
	
		if (_dronecourse_waypoint_pub == nullptr) {
			_dronecourse_waypoint_pub = orb_advertise(ORB_ID(dronecourse_waypoint), &way_p);
		} else {
			orb_publish(ORB_ID(dronecourse_waypoint), _dronecourse_waypoint_pub, &way_p);
		}
		waypoint_published = true;
		PX4_INFO("published waypoint position");
	}
}