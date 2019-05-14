/**
 * @file TargetFollower.cpp
 * Class to follow moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "TargetFollower.hpp"
#include <drivers/drv_hrt.h>
#include <px4_time.h>

#include <iostream>

TargetFollower::TargetFollower(GimbalCtrl &gimbal) :
	PositionCtrl(gimbal),
	_target_detected(false),
	_force_landing(false)
{
	_target_position_ned_filtered_sub = orb_subscribe(ORB_ID(target_position_ned_filtered));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	_gimbal.set_command(0.0f, 0.0f);
	_last_iteration_time = hrt_absolute_time();
	_last_gimbal_yaw = M_PI_F;
}

#define APPROACH_ALTITUDE -5.0f

void TargetFollower::update()
{
	//We need this in order to use the local position
	PositionCtrl::update_subscriptions();
	PositionCtrl::update_parameters();

	update_subscriptions();

	float measure_pos_variance = _target_position_ned_filtered.var_x * _target_position_ned_filtered.var_y * _target_position_ned_filtered.var_z;
	
	if((measure_pos_variance < 6000.0f || _force_landing) && !_vehicle_land_detected.landed) {
		if(!_target_detected) {
			_gimbal.setAutomatic();
			_target_detected = true;
			PX4_INFO("Target detected !");
		}

		float h_distance = sqrtf(pow(_target_position_ned_filtered.x - _current_pos(0), 2) + pow(_target_position_ned_filtered.y - _current_pos(1), 2));

		matrix::Vector3f target(_target_position_ned_filtered.x, _target_position_ned_filtered.y, APPROACH_ALTITUDE);

		if(measure_pos_variance < 500.0f || _force_landing) {
			//PX4_INFO("%f, %f, %f\n", (double)_target_position_ned_filtered.var_vx, (double)_target_position_ned_filtered.var_vy, (double)_target_position_ned_filtered.var_vz);
			float measure_vel_variance = _target_position_ned_filtered.var_vx * _target_position_ned_filtered.var_vy * _target_position_ned_filtered.var_vz;
			
			if((measure_vel_variance < 5.0f && measure_pos_variance < 5.0f && h_distance < 4.0f) || (_force_landing)) {

				matrix::Vector3f position_error = compute_position_error(target, _current_pos);
				matrix::Vector3f velocity_command = compute_velocity_command(position_error, get_position_gain());

				velocity_command(0) += _target_position_ned_filtered.vx;
				velocity_command(1) += _target_position_ned_filtered.vy;

				if(h_distance < 0.5f || _force_landing) {
					float v_distance = fabsf(_target_position_ned_filtered.z - _current_pos(2));

					if(v_distance < 1.0f || _force_landing) {
						_force_landing = true;

						if(_vehicle_land_detected.ground_contact) {
							velocity_command(2) = 3.0f;
						}
						else {
							velocity_command(2) = 0.5f;
						}

						if(_vehicle_land_detected.landed) {
							_force_landing = false;
						}
					}
					else if(v_distance < 3.0f) {
						velocity_command(2) = 1.5f;
					}
					else {
						velocity_command(2) = 3.0f;
					}
				}

				send_velocity_command(velocity_command);
			}
			else {
				PositionCtrl::set_position_command(target);
				PositionCtrl::update();
			}
		}

		_gimbal.update();
	}
	else {
		if(_target_detected) {
			_target_detected = false;
			PX4_INFO("Target lost !");
		}
		float dt = (float)(hrt_absolute_time() - _last_iteration_time) * 1e-6f;

		float _gimbal_speed = M_PI_F / -10.0f;

		_gimbal.set_command(0.0f, _last_gimbal_yaw + _gimbal_speed * dt);
	}
}


bool TargetFollower::is_goal_reached() const
{
	return false;
}

void TargetFollower::update_subscriptions()
{
	// ------------------------------------------------------
	// TODO update subscriptions
	// ------------------------------------------------------
	bool updated;
	
	orb_check(_target_position_ned_filtered_sub, &updated);
	if(updated) {
		orb_copy(ORB_ID(target_position_ned_filtered), _target_position_ned_filtered_sub, &_target_position_ned_filtered);
	}

	orb_check(_vehicle_land_detected_sub, &updated);
	if(updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}

	orb_check(_local_pos_sub, &updated);
	if (updated) {
		vehicle_local_position_s local_pos;
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &local_pos);

		_current_pos(0) = local_pos.x;
		_current_pos(1) = local_pos.y;
		_current_pos(2) = local_pos.z;
	}
}

void TargetFollower::update_parameters()
{
	// ------------------------------------------------------
	// TODO update parameters
	// ------------------------------------------------------

}
