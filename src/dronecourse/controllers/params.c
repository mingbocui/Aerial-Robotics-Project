/**
 * @file target_following_params.c
 * TargetFollowing parameters
 *
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */


/**
* Position controller gain
*
* Gain of the proportional position controller
*
* @min 0.0
* @max 20.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(POS_GAIN, 1.0f);

/**
* Position controller acceptance radius
*
* Acceptance radius used to validate a waypoint in the position controller
*
* @min 0.0
* @max 20.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(POS_ACCEPT_RAD, 1.0f);

/**
* Approach altitude
*
* Drone approach altitude in meters
*
* @min 0.0
* @max 20.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(APP_ALT, 4.0f);

