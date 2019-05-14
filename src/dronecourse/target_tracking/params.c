/**
 * @file params.c
 * TargetTracking parameters
 *
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */


/**
* System noise X state
*
* Noise of the system along the X axis
*
* @min 0.0
* @max 20.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(SYSTEM_NOISE_X, 0.75f);

/**
* System noise Y state
*
* Noise of the system along the Y axis
*
* @min 0.0
* @max 20.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(SYSTEM_NOISE_Y, 0.75f);

/**
* System noise Z state
*
* Noise of the system along the Z axis
*
* @min 0.0
* @max 20.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(SYSTEM_NOISE_Z, 0.75f);

/**
* System noise VX state
*
* Noise of the system velocity along the X axis
*
* @min 0.0
* @max 20.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(SYSTEM_NOISE_VX, 0.75f);

/**
* System noise VY state
*
* Noise of the system velocity along the Y axis
*
* @min 0.0
* @max 20.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(SYSTEM_NOISE_VY, 0.75f);

/**
* System noise VZ state
*
* Noise of the system velocity along the Z axis
*
* @min 0.0
* @max 20.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(SYSTEM_NOISE_VZ, 0.75f);


