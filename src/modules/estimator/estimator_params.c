/**
 * @file estimator_params.c
 * Parameters for UAVbook autopilot estimator.
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 */

#include <systemlib/param/param.h>

/**
 * Gravity
 *
 * @unit m/s^2
 * @min 9.8
 * @max 9.81
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_GRAVITY, 9.81f);

/**
 * Air density
 *
 * @unit kg/m^3
 * @min 0
 * @max 1.40
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_RHO, 1.15f);

/**
 * Standard deviation of accelerameter nosie
 *
 * @unit m/s^2
 * @min 0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_SIGMA_ACCEL, 0.0245f);


/**
 * Standard deviation of gps nosie
 *
 * @unit m
 * @min 0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_SIGMA_N_GPS, 0.21f);

/**
 * Standard deviation of gps nosie
 *
 * @unit m
 * @min 0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_SIGMA_E_GPS, 0.21f);

/**
 * Standard deviation of gps nosie for ground speed
 *
 * @unit m/s
 * @min 0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_SIGMA_VG_GPS, 0.05f);

/**
 * Standard deviation of gps nosie for course angle
 *
 * @unit rad
 * @min 0
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_SIGMA_COURSE_GPS, 0.0038f);
