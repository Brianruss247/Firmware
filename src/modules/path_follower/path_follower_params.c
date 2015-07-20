/**
 * @file path_follower_params.c
 * Parameters for UAVbook autopilot path follower (chapter 10).
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 */

#include <systemlib/param/param.h>

/**
 * Desired course to track striaght line path
 *
 * @unit rad
 * @min 0
 * @max 1.5708
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_CHI_INFTY, 1.5708f);

/**
 * Control gian for tracking a striaght line path
 *
 * @unit 
 * @min 0
 * @max 1
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_K_PATH, 0.025f);

/**
 * Control gian for tracking an orbital path
 *
 * @unit 
 * @min 0
 * @max 1
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_K_ORBIT, 0.05f);
