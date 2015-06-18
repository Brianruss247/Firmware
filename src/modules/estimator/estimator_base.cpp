#include "estimator_base.h"

estimator_base::estimator_base()
{
    _params_sub = orb_subscribe(ORB_ID(parameter_update));

    memset(&_params, 0, sizeof(_params));

    _params_handles.gravity        = param_find("UAVBOOK_GRAVITY");
    _params_handles.rho            = param_find("UAVBOOK_RHO");
    _params_handles.sigma_accel    = param_find("UAVBOOK_SIGMA_ACCEL");
    _params_handles.sigma_n_gps    = param_find("UAVBOOK_SIGMA_N_GPS");
    _params_handles.sigma_e_gps    = param_find("UAVBOOK_SIGMA_E_GPS");
    _params_handles.sigma_Vg_gps   = param_find("UAVBOOK_SIGMA_VG_GPS");
    _params_handles.sigma_course_gps       = param_find("UAVBOOK_SIGMA_COURSE_GPS");

    parameters_update();

}

float estimator_base::spin()
{

    struct input_s input;
    input.gyro_x = 0;
    input.gyro_y = 0;
    input.gyro_z = 0;
    input.accel_x = 0;
    input.accel_y = 0;
    input.accel_z = 0;
    input.static_pres = 0;
    input.diff_pres = 0;
    input.gps_n = 0;
    input.gps_e = 0;
    input.gps_h = 0;
    input.gps_Vg = 0;
    input.gps_course = 0;
    input.Ts = 0;

    struct output_s output;

    estimate(_params, input, output);

    return 1;
}

int estimator_base::parameters_update()
{
    param_get(_params_handles.gravity, &_params.gravity);
    param_get(_params_handles.rho, &_params.rho);
    param_get(_params_handles.sigma_accel, &_params.sigma_accel);
    param_get(_params_handles.sigma_n_gps, &_params.sigma_n_gps);
    param_get(_params_handles.sigma_e_gps, &_params.sigma_e_gps);
    param_get(_params_handles.sigma_Vg_gps, &_params.sigma_Vg_gps);
    param_get(_params_handles.sigma_course_gps, &_params.sigma_course_gps);

    return OK;
}
