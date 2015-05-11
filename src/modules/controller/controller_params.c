/**
 * @file controller_params.c
 * Parameters for UAVbook autopilot controller.
 *
 * @author Daniel Koch <daniel.koch@byu.edu>
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 */

#include <systemlib/param/param.h>

/**
 * Roll P gain
 *
 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_ROLL_P, 6.0f);

/**
 * Roll rate P gain
 *
 * Roll rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_ROLLRATE_P, 0.1f);

/**
 * Roll rate I gain
 *
 * Roll rate integral gain. 
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_ROLLRATE_I, 0.0f);

/**
 * Roll rate D gain
 *
 * Roll rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_ROLLRATE_D, 0.002f);

/**
 * Pitch P gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_PITCH_P, 6.0f);

/**
 * Pitch rate P gain
 *
 * Pitch rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_PITCHRATE_P, 0.1f);

/**
 * Pitch rate I gain
 *
 * Pitch rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_PITCHRATE_I, 0.0f);

/**
 * Pitch rate D gain
 *
 * Pitch rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_PITCHRATE_D, 0.002f);

/**
 * Yaw P gain
 *
 * Yaw proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_YAW_P, 2.0f);

/**
 * Yaw rate P gain
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_YAWRATE_P, 0.3f);

/**
 * Yaw rate I gain
 *
 * Yaw rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_YAWRATE_I, 0.0f);

/**
 * Yaw rate D gain
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_YAWRATE_D, 0.0f);

/**
 * Max yaw rate
 *
 * Limit for yaw rate, has effect for large rotations in autonomous mode, to avoid large control output and mixer saturation.
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_YAWRATE_MAX, 120.0f);

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
 * Altitude take off zone
 *
 * @unit m
 * @min 0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_ALT_TOZ, 30.0f);

/**
 * Altitude hold zone
 *
 * @unit m
 * @min 0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_ALT_HZ, 90.0f);

/**
 * Max deflection elevator
 *
 * @unit 
 * @max 1
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_MAX_E, 1.0f);

/**
 * Max deflection aileron
 *
 * @unit 
 * @max 1
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_MAX_A, 1.0f);

/**
 * Max deflection rudder
 *
 * @unit 
 * @max 1
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_MAX_R, 1.0f);

/**
 * Max deflection throttle
 *
 * @unit 
 * @max 1
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_MAX_T, 1.0f);




