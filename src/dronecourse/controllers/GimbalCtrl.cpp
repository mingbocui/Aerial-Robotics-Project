/**
 * @file GimbalCtrl.cpp
 * Class to send to control camera gimbal
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 * @author Arthur Gay <arthur.gay@epfl.ch>
 */

#include "GimbalCtrl.hpp"
#include <uORB/topics/gimbal_command.h>
#include <uORB/topics/target_position_ned_filtered.h>
#include <uORB/topics/vehicle_attitude.h>
#include <drivers/drv_hrt.h>

#include <matrix/math.hpp>
#include <matrix/Euler.hpp>

#include <iostream>

GimbalCtrl::GimbalCtrl()
{
	// --------------------------------------------------
	// TODO subscribe to uORB messages:
	//      target_position_ned_filtered,
	//      vehicle_attitude and vehicle_local_position
	// --------------------------------------------------
	_target_position_ned_filtered_sub = orb_subscribe(ORB_ID(target_position_ned_filtered));
	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	// set publishing handle for gimbal_command to nullptr
	_gimbal_command_pub = nullptr;

	// Set inital angles downward facing
	set_command(-M_PI / 2, 0.0f);
}

GimbalCtrl::~GimbalCtrl()
{
	// --------------------------------------------------
	// TODO unsubscribe from uORB messages:
	//      target_position_ned_filtered,
	//      vehicle_attitude and vehicle_local_position
	// --------------------------------------------------
	orb_unsubscribe(_target_position_ned_filtered_sub);
	orb_unsubscribe(_vehicle_attitude_sub);
	orb_unsubscribe(_vehicle_local_position_sub);

	// Unadvertise
	if(_gimbal_command_pub != nullptr) {
		orb_unadvertise(_gimbal_command_pub);
		_gimbal_command_pub = nullptr;
	}

}

void GimbalCtrl::set_command(float pitch, float yaw)
{
	_mode = MODE::MANUAL;

	// Limit pitch
	if (pitch > 0.0f) {
		pitch = 0.0f;
	} else if (pitch < (float)(- M_PI)) {
		pitch = - M_PI;
	}

	publish_gimbal_command(pitch, yaw);
}

void GimbalCtrl::update()
{
	if (_mode == MODE::AUTOMATIC) {
		// target position in local frame
		matrix::Vector3f target_pos_lf;

		// drone's attitude
		matrix::Quaternion<float> att_vehicle;

		// drone's position
		matrix::Vector3f pos_vehicle;

		bool updated;
		// ------------------------------------------------
		// TODO set target position from uORB
		// ------------------------------------------------
		target_position_ned_filtered_s target_position_ned_filtered;
		orb_check(_target_position_ned_filtered_sub, &updated);
		if(updated) {
			orb_copy(ORB_ID(target_position_ned_filtered), _target_position_ned_filtered_sub, &target_position_ned_filtered);
			target_pos_lf(0) = target_position_ned_filtered.x;
			target_pos_lf(1) = target_position_ned_filtered.y;
			target_pos_lf(2) = target_position_ned_filtered.z;
		}

		// ------------------------------------------------
		// TODO set drone's attitude from uORB
		// ------------------------------------------------
		orb_check(_vehicle_attitude_sub, &updated);
		if(updated) {
			vehicle_attitude_s vehicle_attitude;
			orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &vehicle_attitude);
			att_vehicle = matrix::Quaternion<float>(vehicle_attitude.q);
		}

		// ------------------------------------------------
		// TODO set drone position from uORB
		// ------------------------------------------------
		orb_check(_vehicle_local_position_sub, &updated);
		if(updated) {
			vehicle_local_position_s vehicle_local_position;
			orb_copy(ORB_ID(vehicle_local_position), _vehicle_local_position_sub, &vehicle_local_position);
			pos_vehicle(0) = vehicle_local_position.x;
			pos_vehicle(1) = vehicle_local_position.y;
			pos_vehicle(2) = vehicle_local_position.z;
		}

		// create variable for target direction
		const matrix::Vector3f target_dir = compute_target_direction(target_pos_lf, pos_vehicle);

		// create variables for drone's Z and Y axis in local frame
		const matrix::Vector3f down_lf = compute_down_axis(att_vehicle);
		const matrix::Vector3f east_lf = compute_east_axis(att_vehicle);

		// create variable for normal vector of auxiliary plane
		// containing target direction and drone's Z axis in local frame
		const matrix::Vector3f n = compute_normal_vector(target_dir, down_lf);

		// create variable for yaw and pitch
		float yaw = compute_yaw(target_dir, n, down_lf, east_lf);
		float pitch = compute_pitch(target_dir, n, down_lf, east_lf);

		// publish uORB message
		publish_gimbal_command(pitch, yaw);
	}
}

matrix::Vector3f GimbalCtrl::compute_target_direction(const matrix::Vector3f target_pos_ned, const matrix::Vector3f pos_vehicle)
{
	// ------------------------------------------------
	// TODO compute target direction
	//      i.e., vector from drone to target
	// ------------------------------------------------
	return (target_pos_ned - pos_vehicle).normalized();
}

matrix::Vector3f GimbalCtrl::compute_down_axis(matrix::Quaternion<float> attitude)
{
	// ------------------------------------------------
	// TODO compute drone's Z axis in local frame
	// ------------------------------------------------
	matrix::Vector3f z(0.0f, 0.0f, 1.0f);
	return matrix::Vector3f(matrix::Dcm<float>(attitude) * z).normalized();
}

matrix::Vector3f GimbalCtrl::compute_east_axis(matrix::Quaternion<float> attitude)
{
	// ------------------------------------------------
	// TODO compute drone's Y axis in local frame
	// ------------------------------------------------
	matrix::Vector3f y(0.0f, 1.0f, 0.0f);
	return matrix::Vector3f(matrix::Dcm<float>(attitude) * y).normalized();
}

matrix::Vector3f GimbalCtrl::compute_normal_vector(const matrix::Vector3f target_direction, const matrix::Vector3f down_lf)
{
	// ---------------------------------------------------
	// TODO find normal vector of auxiliary plane
	//      containing target direction and
	//      drone's Z axis in local frame
	// ---------------------------------------------------
	return target_direction.cross(down_lf).normalized();
}

float GimbalCtrl::compute_yaw(matrix::Vector3f target_direction,
	matrix::Vector3f n,
	matrix::Vector3f down_lf,
	matrix::Vector3f east_lf)
{
	// ---------------------------------------------------------------------
	// TODO find desired yaw angle as angle between
	//      the plane's normal vector and the drone's Y axis in local frame
	// ---------------------------------------------------------------------
	if(n.cross(east_lf).dot(down_lf) > 0) {
		return -acosf(n.dot(east_lf));
	}
	else {
		return acosf(n.dot(east_lf));
	}
}

float GimbalCtrl::compute_pitch(matrix::Vector3f target_direction,
	matrix::Vector3f n,
	matrix::Vector3f down_lf,
	matrix::Vector3f east_lf)
{
	// ----------------------------------------------------------
	// TODO find desired pitch angle as angle between
	//      drone's Z axis in local frame and target direction
	// ----------------------------------------------------------
	if(target_direction.cross(down_lf).dot(n) > 0.0f) {
		return -(float) M_PI_2 - acosf(target_direction.dot(down_lf));
	}
	else {
		return -(float) M_PI_2 + acosf(target_direction.dot(down_lf));
	}
}

void GimbalCtrl::publish_gimbal_command(float pitch, float yaw)
{
	// --------------------------------------------------------
	// TODO publish gimbal_command over uORB
	// --------------------------------------------------------
	gimbal_command_s command;
	command.pitch = pitch;
	command.yaw = yaw;
	command.timestamp = hrt_absolute_time();

	if (_gimbal_command_pub == nullptr) {
		_gimbal_command_pub = orb_advertise(ORB_ID(gimbal_command), &command);
	} else {
		orb_publish(ORB_ID(gimbal_command), _gimbal_command_pub, &command);
	}
}
