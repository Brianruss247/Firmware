/**
 * @file estimator_base.h
 *
 * Base class definition for autopilot estimator in chapter 8 of UAVbook, see http://uavbook.byu.edu/doku.php
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 */

#ifndef ESTIMATOR_BASE_H
#define ESTIMATOR_BASE_H

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

#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <lib/mathlib/mathlib.h>

class estimator_base
{
public:
    estimator_base();
    float spin();

protected:

    struct input_s{
        float gyro_x;
        float gyro_y;
        float gyro_z;
        float accel_x;
        float accel_y;
        float accel_z;
        float static_pres;
        float diff_pres;
        float gps_n;
        float gps_e;
        float gps_h;
        float gps_Vg;
        float gps_course;
        float Ts;
    };

    struct output_s{
        float pn;
        float pe;
        float h;
        float Va;
        float alpha;
        float beta;
        float phi;
        float theta;
        float psi;
        float chi;
        float p;
        float q;
        float r;
        float Vg;
        float wn;
        float we;
    };

    struct params_s{
        float gravity;
        float rho;
        float sigma_accel;
        float sigma_n_gps;
        float sigma_e_gps;
        float sigma_Vg_gps;
        float sigma_course_gps;
    };

    virtual void estimate(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

private:
    int _params_sub;            /**< parameter updates subscription */



    struct {
        param_t gravity;
        param_t rho;
        param_t sigma_accel;
        param_t sigma_n_gps;
        param_t sigma_e_gps;
        param_t sigma_Vg_gps;
        param_t sigma_course_gps;
    } _params_handles; /**< handles for interesting parameters */

    struct params_s _params;

    /**
    * Update our local parameter cache.
    */
    int parameters_update();
};

#endif // ESTIMATOR_BASE_H
