/**
 * @file path_manager_params.c
 * Parameters for UAVbook autopilot path manager (chapter 11).
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 */

#include <systemlib/param/param.h>

/**
 * minimum turn radius
 *
 * @unit m
 * @min 1
 * @max 50
 * @group UAVbook
 */
PARAM_DEFINE_FLOAT(UAVBOOK_R_MIN, 20.0f);
