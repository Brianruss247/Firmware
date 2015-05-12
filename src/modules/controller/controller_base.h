/**
 * @file controller_base.h
 *
 * Base class definition for autopilot controller in chapter 6 of UAVbook, see http://uavbook.byu.edu/doku.php
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 */

#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <fcntl.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <float.h>

#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_state.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>

#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>

#define MANUAL_THRESHOLD 0.05f

class controller_base
{
public:
    controller_base();
    void spin();

protected:

    struct input_s{
        float h;
        float v_a;
        float phi;
        float theta;
        float chi;
        float p;
        float q;
        float r;
    };

    struct output_s{
        float delta_e;
        float delta_a;
        float delta_r;
        float delta_t;
    };

    struct params_s {
        math::Vector<3> att_p;  /**< P gain for angular error */
        math::Vector<3> rate_p; /**< P gain for angular rate error */
        math::Vector<3> rate_i; /**< I gain for angular rate error */
        math::Vector<3> rate_d; /**< D gain for angular rate error */
        float yaw_ff;           /**< yaw control feed-forward */
        float yaw_rate_max;     /**< max yaw rate */
        math::Vector<4> trims;  /**< e,a,r,t */
        float alt_toz;          /**< altitude takeoff zone */
        float alt_hz;          /**< altitude hold zone */
        math::Vector<4> max;    /**< e,a,r,t */
    };

    virtual void control(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

private:
    int _params_sub;            /**< parameter updates subscription */
    int _vehicle_state_sub;     /**< vehicle state subscription */
    int _manual_control_sp_sub; /**< manual control setpoint subscription */
    struct pollfd fds[1];
    int poll_error_counter;

    orb_advert_t _actuators_0_pub; /**< attitude actuator controls publication */

    struct {
        param_t roll_p;
        param_t roll_rate_p;
        param_t roll_rate_i;
        param_t roll_rate_d;
        param_t pitch_p;
        param_t pitch_rate_p;
        param_t pitch_rate_i;
        param_t pitch_rate_d;
        param_t yaw_p;
        param_t yaw_rate_p;
        param_t yaw_rate_i;
        param_t yaw_rate_d;
        param_t yaw_ff;
        param_t yaw_rate_max;

        param_t trim_e;
        param_t trim_a;
        param_t trim_r;
        param_t trim_t;
        param_t alt_toz;
        param_t alt_hz;
        param_t max_e;
        param_t max_a;
        param_t max_r;
        param_t max_t;
    } _params_handles; /**< handles for interesting parameters */

    struct vehicle_state_s             _vehicle_state;     /**< vehicle state */
    struct manual_control_setpoint_s   _manual_control_sp; /**< manual control setpoint */
    struct actuator_controls_s         _actuators;         /**< actuator controls */
    struct params_s                    _params;            /**< params */

    /**
    * Update our local parameter cache.
    */
    int parameters_update();

    /**
    * Check for parameter update and handle it.
    */
    void parameter_update_poll();

    /**
    * Check for changes in vehicle state.
    */
    void vehicle_state_poll();

    /**
    * Check for changes in manual inputs.
    */
    void manual_control_poll();

    /**
    * RC override
    */
    void pilot_override(struct output_s &output);

    /**
    * Publish the outputs
    */
    void actuator_controls_publish(struct output_s &output);
};

#endif // CONTROLLER_BASE_H
