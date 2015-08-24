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
#include <uORB/topics/controller_commands.h>
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
    float spin();
    virtual int getstate() = 0;

protected:

    struct input_s{
        float Ts;               /** time step */
        float h;                /** altitude */
        float va;               /** airspeed */
        float phi;              /** roll angle */
        float theta;            /** pitch angle */
        float chi;              /** course angle */
        float p;                /** body frame roll rate */
        float q;                /** body frame pitch rate */
        float r;                /** body frame yaw rate */
        float Va_c;             /** commanded airspeed (m/s) */
        float h_c;              /** commanded altitude (m) */
        float chi_c;            /** commanded course (rad) */
    };

    struct output_s{
        float theta_c;
        float delta_e;
        float phi_c;
        float delta_a;
        float delta_r;
        float delta_t;
    };

    struct params_s {
        float alt_hz;           /**< altitude hold zone */
        float alt_toz;          /**< altitude takeoff zone */
        float tau;
        float c_kp;
        float c_kd;
        float c_ki;
        float r_kp;
        float r_kd;
        float r_ki;
        float p_kp;
        float p_kd;
        float p_ki;
        float p_ff;
        float a_p_kp;
        float a_p_kd;
        float a_p_ki;
        float a_t_kp;
        float a_t_kd;
        float a_t_ki;
        float a_kp;
        float a_kd;
        float a_ki;
        float b_kp;
        float b_kd;
        float b_ki;
        float trim_e;
        float trim_a;
        float trim_r;
        float trim_t;
        float max_e;
        float max_a;
        float max_r;
        float max_t;
        float pwm_rad_e;
        float pwm_rad_a;
        float pwm_rad_r;
    };

    virtual void control(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

private:
    int _params_sub;            /**< parameter updates subscription */
    int _vehicle_state_sub;     /**< vehicle state subscription */
    int _controller_commands_sub;/**< controller commands subscription */
    int _manual_control_sp_sub; /**< manual control setpoint subscription */
    struct pollfd fds[1];
    int poll_error_counter;

    orb_advert_t _actuators_0_pub; /**< attitude actuator controls publication */

    struct {
        param_t trim_e;
        param_t trim_a;
        param_t trim_r;
        param_t trim_t;
        param_t pwm_rad_e;
        param_t pwm_rad_a;
        param_t pwm_rad_r;
        param_t alt_toz;
        param_t alt_hz;
        param_t tau;
        param_t course_kp;
        param_t course_kd;
        param_t course_ki;
        param_t roll_kp;
        param_t roll_kd;
        param_t roll_ki;
        param_t pitch_kp;
        param_t pitch_kd;
        param_t pitch_ki;
        param_t pitch_ff;
        param_t airspeed_pitch_kp;
        param_t airspeed_pitch_kd;
        param_t airspeed_pitch_ki;
        param_t airspeed_throttle_kp;
        param_t airspeed_throttle_kd;
        param_t airspeed_throttle_ki;
        param_t altitude_kp;
        param_t altitude_kd;
        param_t altitude_ki;
        param_t beta_kp;
        param_t beta_kd;
        param_t beta_ki;
        param_t max_e;
        param_t max_a;
        param_t max_r;
        param_t max_t;
    } _params_handles; /**< handles for interesting parameters */

    struct vehicle_state_s             _vehicle_state;     /**< vehicle state */
    struct controller_commands_s       _controller_commands;/**< controller commands */
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
    * Check for changes in controller commands.
    */
    void controller_commads_poll();

    /**
    * Check for changes in manual inputs.
    */
    void manual_control_poll();

    /**
    * Convert from deflection angle to pwm
    */
    void convert_to_pwm(struct output_s &output);

    /**
    * RC override
    */
    void pilot_override(struct output_s &output);

    /**
    * Publish the outputs
    */
    void actuator_controls_publish(struct output_s &output);

    hrt_abstime prev_time_;
};

#endif // CONTROLLER_BASE_H
