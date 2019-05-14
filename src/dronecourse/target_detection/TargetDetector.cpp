/**
 * @file TargetDetector.hpp
 * Class to track moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "TargetDetector.hpp"
#include <matrix/math.hpp>
#include <uORB/topics/target_position_ned.h>
#include <uORB/topics/target_position_image.h>
#include <uORB/topics/vehicle_attitude.h>
#include <drivers/drv_hrt.h>

TargetDetector::TargetDetector(const float hfov_default,
	               const int image_width,
	               const int image_height) :
	               HFOV(hfov_default),
	               IMAGE_WIDTH(image_width),
	               IMAGE_HEIGHT(image_height)
{
	// --------------------------------------------------------------------------
	// TODO subscribe to uORB topics:
	//  target_position_image messages, vehicle_attitude, vehicle_local_position
	// --------------------------------------------------------------------------
	_target_position_image_sub = orb_subscribe(ORB_ID(target_position_image));
	_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	// --------------------------------------------------------------------------
	// TODO set uORB publishing handle '_target_position_pub' to nullptr
	// --------------------------------------------------------------------------
	_target_position_pub = nullptr;
}

TargetDetector::~TargetDetector()
{
	// --------------------------------------------------------------------------
	// TODO unsubscribe from uORB topics:
	//  target_position_image messages, vehicle_attitude, vehicle_local_position
	// --------------------------------------------------------------------------
	orb_unsubscribe(_target_position_image_sub);
	orb_unsubscribe(_attitude_sub);
	orb_unsubscribe(_position_sub);
}

void TargetDetector::update()
{
	update_subscriptions();

	bool new_measure = false;
	// ------------------------------------------------------------
	// TODO check if we have a new _target_position_image message
	// and set new_measure accordingly
	// ------------------------------------------------------------
	orb_check(_target_position_image_sub, &new_measure);

	if (new_measure) {
		// --------------------------------------------------------------------
		// TODO copy message content to a local variable
		// --------------------------------------------------------------------
		struct target_position_image_s target_pos;

		orb_copy(ORB_ID(target_position_image), _target_position_image_sub, &target_pos);

		// ~~~ Computations ~~~
		/** The target position in the image (u, v) */
		const matrix::Vector2f target_pos_image(target_pos.u, target_pos.v);
		/** The distance from the camera to the target*/
		const float distance = target_pos.dist;

		/** Focal length */
		const float focal_length = compute_focal_length(IMAGE_WIDTH, HFOV);

		/** The target position in centered image coordinates (x, y) */
		const matrix::Vector2f centered_image_coordinates = compute_centered_image_coordinates(target_pos_image, IMAGE_WIDTH, IMAGE_HEIGHT);

		/** The scale s */
		const float scale = compute_scale(centered_image_coordinates, distance, focal_length);

		/** The target location in image frame */
		const matrix::Vector3f target_pos_image_frame = compute_target_position_image_frame(centered_image_coordinates, focal_length, scale);

		/** Rotation matrix to convert from camera frame to gimbal frame from euler angles */
		const matrix::Dcm<float> image_rot = compute_rotation_camera_to_gimbal();

		/** Rotation matrix to convert from gimbal frame to  the drone's body frame using the gimbal angles */
		const matrix::Dcm<float> gimbal_rot = compute_rotation_gimbal_to_drone(target_pos.pitch, target_pos.yaw);

		/** Rotation matrix (dcm) from image frame to local frame using previous rotation matrices and the attitude of the vehicle */
		const matrix::Dcm<float> total_rot = compute_rotation_matrix(image_rot, gimbal_rot, _att_vehicle);

		/** Target position in local frame */
		const matrix::Vector3f target_pos_local_frame = compute_target_position_local_frame(total_rot, target_pos_image_frame, _pos_vehicle);

		/** The covariance matrix */
		const matrix::SquareMatrix<float, 3> var_image_frame = compute_covariance_image_frame(centered_image_coordinates,
			distance, focal_length, scale, target_pos.var_u, target_pos.var_v, target_pos.var_dist);

		/** Convert covariance matrix from image frame to local frame */
		const matrix::SquareMatrix<float, 3> var_local_frame = compute_covariance_local_frame(var_image_frame, total_rot);

		// Publish computed position in local frame
		struct target_position_ned_s target_pos_ned;
		memset(&target_pos_ned, 0, sizeof(target_pos_ned));     // set all fields to 0
		target_pos_ned.x = target_pos_local_frame(0);
		target_pos_ned.y = target_pos_local_frame(1);
		target_pos_ned.z = target_pos_local_frame(2);
		target_pos_ned.timestamp = hrt_absolute_time();
		memcpy(&target_pos_ned.var, var_local_frame.data(), sizeof(target_pos_ned.var));

		int instance;
		orb_publish_auto(ORB_ID(target_position_ned), &_target_position_pub, &target_pos_ned, &instance, ORB_PRIO_HIGH);
	}
}

/** Compute the focal length
 *  @param image_width
 *  @param hfov field of view
 *  @return computed focal length
 */
float TargetDetector::compute_focal_length(const int image_width, const float hfov)
{
	// -----------------------------------------------------------------
	// TODO compute the focal length f
	// -----------------------------------------------------------------
	return image_width / 2.0f / tanf(hfov / 2.0f);
}

/** Compute the centered image coordinates (x, y)
 *  @param  target_pos 	target location in image coordinates (u, v)
 *  @param  image_width image horizontal size
 *  @param  image_height image vertical size
 *  @return target location in centered image coordinates (x, y)
 */
matrix::Vector2f TargetDetector::compute_centered_image_coordinates(
	const matrix::Vector2f& target_pos,
	const int image_width,
	const int image_height)
{
	// -----------------------------------------------------------------
	// TODO compute target's position in centered image coordinates
	// -----------------------------------------------------------------

	return matrix::Vector2f(target_pos(0) - image_width / 2.0f, target_pos(1) - image_height / 2.0f);
}

/** Compute the scale s
 *	@param centered_image_coordinates 	target location (x, y)
 *  @param distance						distance from camera to target d
 *  @param focal_length 				the focal length f
 *  @return the scale s
 */
float TargetDetector::compute_scale(
	const matrix::Vector2f& centered_image_coordinates,
	const float distance,
	const float focal_length)
{
	// -----------------------------------------------------------------
	// TODO compute the scale s
	// -----------------------------------------------------------------
	float dist_px = sqrtf(powf(centered_image_coordinates(0), 2) + powf(centered_image_coordinates(1), 2));
	float dist_cx = sqrtf(powf(dist_px, 2) + powf(focal_length, 2));

	return distance / dist_cx;
}

/** Find the target location in image frame.
 *  @param  target_pos_image 	 target position in centered image coordinates (x, y)
 *  @param  focal_length the focal length f
 *  @param  scale        the scale s
 *  @return target location in camera coordinates (X, Y, Z)
 */
matrix::Vector3f TargetDetector::compute_target_position_image_frame(const matrix::Vector2f& target_pos_image, const float focal_length, const float scale)
{
	// -----------------------------------------------------------------
	// TODO compute target's position in image frame
	// -----------------------------------------------------------------
	float x = scale * target_pos_image(0);
	float y = scale * target_pos_image(1);
	float z = scale * focal_length;

	return matrix::Vector3f(x, y, z);
}

/** Compute covariance matrix
 *  @param target_pos_image target position in centered image coordinates (x, y)
 *  @param focal_length f
 *  @param scale s
 *  @param target_pos provides the variance measurements
 *  @return covariance matrix
 */
matrix::SquareMatrix<float, 3>  TargetDetector::compute_covariance_image_frame(
	const matrix::Vector2f& target_pos_image,
	const float distance,
	const float focal_length,
	const float scale,
	const float var_u,
	const float var_v,
	const float var_d)
{
	// -----------------------------------------------------------------
	// TODO compute covariance matrix in image frame
	// -----------------------------------------------------------------
	float l = sqrtf(powf(target_pos_image(0) ,2) + powf(target_pos_image(1) ,2) + powf(focal_length ,2));

	float var_s = var_d / powf(l, 2) + powf(distance, 2) / powf(l, 6) * (powf(target_pos_image(0), 2) * var_u + powf(target_pos_image(1), 2) * var_v);

	matrix::SquareMatrix<float, 3> covariance;
	covariance(0,0) = powf(target_pos_image(0) ,2) * var_s + powf(scale ,2) * var_u;
	covariance(0,1) = 0;
	covariance(0,2) = 0;

	covariance(1,0) = 0;
	covariance(1,1) = powf(target_pos_image(1) ,2) * var_s + powf(scale ,2) * var_v;
	covariance(1,2) = 0;

	covariance(2,0) = 0;
	covariance(2,1) = 0;
	covariance(2,2) = powf(focal_length ,2) * var_s;

	return covariance;
}

/** Compute a rotation matrix to convert from camera frame to gimbal frame
 *  @return the computed rotation matrix
 */
matrix::Dcm<float> TargetDetector::compute_rotation_camera_to_gimbal()
{
	// -----------------------------------------------------------------
	// TODO compute rotation matrix to convert from camera frame to gimbal frame
	// -----------------------------------------------------------------
	const float data[9] = { 0.0f, 	0.0f, 	1.0f,
							1.0f, 	0.0f, 	0.0f,
							0.0f,	1.0f, 	0.0f  };


	return matrix::Dcm<float>(data);
}

/** Compute a rotation matrix to convert from gimbal frame to the drone's body frame using the gimbal angles
 *  @param pitch angle of the gimble
 *  @param yaw angle of the gimble
 *  @return the computed rotation matrix
 */
matrix::Dcm<float> TargetDetector::compute_rotation_gimbal_to_drone(
	const float pitch,
	const float yaw)
{
	// -----------------------------------------------------------------
	// TODO compute rotation matrix to convert from gimbal frame to the
	//      drone's body frame using gimbal angles
	// -----------------------------------------------------------------
	/*
	const float data[9] = { 1.0f, 0.0f, 0.0f,
							0.0f, 1.0f, 0.0f,
							0.0f, 0.0f, 1.0f};

	*/
	return matrix::Dcm<float>(matrix::Euler<float>(0.0f, pitch, yaw));
}

/** Compute the rotation matrix from image frame to local frame.
 *  @param image_rot rotation matrix from image frame to camera NED frame
 *  @param gimbal_rot rotation matrix from camera NED frame to drone's body
 *  @param att_vehicle rotation matrix
 *  @return the computed rotation matrix
 */
matrix::Dcm<float> TargetDetector::compute_rotation_matrix(
	const matrix::Dcm<float>& image_rot,
	const matrix::Dcm<float>& gimbal_rot,
	const matrix::Dcm<float>& att_vehicle)
{
	// -----------------------------------------------------------------
	// TODO compute the rotation matrix to convert from image frame to
	//      local frame
	// -----------------------------------------------------------------
	return att_vehicle*gimbal_rot*image_rot;
}

/** Compute the target position in the local frame.
 *  @param total_rot the rotation matrix from image frame to local frame
 *  @param target_pos the position in image frame
 *  @param pos_vehicle the position of the drone
 *  @return target position in local frame
 */
matrix::Vector3f TargetDetector::compute_target_position_local_frame(
	const matrix::Dcm<float>& total_rot,
	const matrix::Vector3f& target_pos,
	const matrix::Vector3f& pos_vehicle)
{
	// -----------------------------------------------------------------
	// TODO convert target's position from image frame to local frame
	// -----------------------------------------------------------------
	return total_rot*target_pos + pos_vehicle;
}

/** Convert covariance matrix from image frame to local frame
 *  @param var_if covariance matrix in image frame
 *  @param total_rot rotation matrix from image frame to local frame
 *  @return covariance matrix in local frame
 */
matrix::SquareMatrix<float, 3> TargetDetector::compute_covariance_local_frame(
	const matrix::SquareMatrix<float, 3>& var_if,
	const matrix::Dcm<float>& total_rot)
{
	// -----------------------------------------------------------------
	// TODO convert covariance from image frame to local frame
	// -----------------------------------------------------------------
	matrix::SquareMatrix<float, 3> covariance = total_rot * var_if * total_rot.T();
	return covariance;
}

void TargetDetector::update_subscriptions()
{
	//--------------------------------------------------------------------------------------
	// TODO update subscriptions for vehicle_attitude as well as for vehicle_local_position
	//   and update the member variables _att_vehicle and _pos_vehicle
	// -------------------------------------------------------------------------------------
	bool updated = false;
	orb_check(_attitude_sub, &updated);

	if(updated) {
		struct vehicle_attitude_s attitude;
		orb_copy(ORB_ID(vehicle_attitude), _attitude_sub, &attitude);
		_att_vehicle = matrix::Dcm<float>(matrix::Quaternion<float>(attitude.q));
	}

	orb_check(_position_sub, &updated);
	if(updated) {
		struct vehicle_local_position_s position;
		orb_copy(ORB_ID(vehicle_local_position), _position_sub, &position);
		_pos_vehicle(0) = position.x;
		_pos_vehicle(1) = position.y;
		_pos_vehicle(2) = position.z;
	}
}
