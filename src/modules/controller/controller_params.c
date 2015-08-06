/**
 * @file controller_params.c
 * Parameters for UAVbook autopilot controller.
 *
 * @author Daniel Koch <daniel.koch@byu.edu>
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 */

#include <systemlib/param/param.h>

/**
 * Trim value for elevator
 *
 * @unit 
 * @min -1
 * @max 1
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_TRIM_E, 0.0f);

/**
 * Trim value for aileron
 *
 * @unit 
 * @min -1
 * @max 1
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_TRIM_A, 0.0f);

/**
 * Trim value for rudder
 *
 * @unit 
 * @min -1
 * @max 1
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_TRIM_R, 0.0f);

/**
 * Trim value for throttle
 *
 * @unit 
 * @min 0
 * @max 1
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_TRIM_T, 0.5f);

/**
 * Conversion from angle to servo command for elevator
 *
 * @unit PWM/radian
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_PWM_RAD_E, 1.0f);

/**
 * Conversion from angle to servo command for aileron
 *
 * @unit PWM/radian
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_PWM_RAD_A, 1.0f);

/**
 * Conversion from angle to servo command for rudder
 *
 * @unit PWM/radian
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_PWM_RAD_R, 1.0f);

/**
 * Altitude take off zone
 *
 * @unit m
 * @min 0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_ALT_TOZ, 20.0f);

/**
 * Altitude hold zone
 *
 * @unit m
 * @min 0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_ALT_HZ, 10.0f);

/**
 * tau for dirty derivative/low-pass fliter.
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_TAU, 5.0f);

/**
 * Course loop Propotional gain
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_COURSE_KP, 0.4657f);

/**
 * Course loop Derivative gain
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_COURSE_KD, 0.0f);

/**
 * Course loop Integral gain
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_COURSE_KI, 0.0596f);


/**
 * Roll loop Propotional gain
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_ROLL_KP, 1.2855f);

/**
 * Roll loop Derivative gain
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_ROLL_KD, 0.0934f);

/**
 * Roll loop Integral gain
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_ROLL_KI, 0.10f);


/**
 * Pitch loop Propotional gain
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_PITCH_KP, 1.0f);

/**
 * Pitch loop Derivative gain
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_PITCH_KD, -0.1168f);

/**
 * Pitch loop Integral gain
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_PITCH_KI, 0.0f);

/**
 * Pitch loop feed forward
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_PITCH_FF, 0.0f);

/**
 * Airspeed-Pitch loop Propotional gain
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_AIRSPEED_PITCH_KP, -0.0713f);

/**
 * Airspeed-Pitch loop Derivative gain
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_AIRSPEED_PITCH_KD, -0.0635f);

/**
 * Airspeed-Pitch loop Integral gain
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_AIRSPEED_PITCH_KI, 0.0f);

/**
 * Airspeed-Throttle loop Propotional gain
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_AIRSPEED_THROTTLE_KP, 5.25f);

/**
 * Airspeed-Throttle loop Derivative gain
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_AIRSPEED_THROTTLE_KD, 0.0f);

/**
 * Airspeed-Throttle loop Integral gain
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_AIRSPEED_THROTTLE_KI, 0.5f);

/**
 * Altitude loop Propotional gain
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_ALTITUDE_KP, 0.0546f);

/**
 * Altitude loop Derivative gain
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_ALTITUDE_KD, 0.0f);

/**
 * Altitude loop Integral gain
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_ALTITUDE_KI, 0.0120f);

/**
 * Beta loop Propotional gain
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_BETA_KP, -0.1164f);

/**
 * Beta loop Derivative gain
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_BETA_KD, 0.0f);

/**
 * Beta loop Integral gain
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_BETA_KI, -3.7111f);

/**
 * Max deflection elevator
 *
 * @unit rad
 * @max 1
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_MAX_E, 0.610f);

/**
 * Max deflection aileron
 *
 * @unit rad
 * @max 1
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_MAX_A, 0.523f);

/**
 * Max deflection rudder
 *
 * @unit rad
 * @max 1
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_MAX_R, 0.523f);

/**
 * Max deflection throttle
 *
 * @unit
 * @max 1
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_MAX_T, 1.0f);




