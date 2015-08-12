#include "estimator_base.h"

estimator_base::estimator_base()
{
    _time_to_run = -1;

    _params_sub = orb_subscribe(ORB_ID(parameter_update));
    _sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
//    _airspeed_sub = orb_subscribe(ORB_ID(airspeed));
    _gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
    _gps_init = false;
    _baro_init = false;
    fds[0].fd = _sensor_combined_sub;
    fds[0].events = POLLIN;
    poll_error_counter = 0;

    memset(&_params, 0, sizeof(_params));
    memset(&_sensor_combined, 0, sizeof(_sensor_combined));
    memset(&_gps, 0, sizeof(_gps));
    memset(&_vehicle_state, 0, sizeof(_vehicle_state));

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
    /* wait for sensor update of 2 file descriptor for 10 ms */
    int poll_ret = poll(fds, 1, 50);

    if (poll_ret < 0) {
        /* this is seriously bad - should be an emergency */
        if (poll_error_counter < 10 || poll_error_counter % 50 == 0) {
            /* use a counter to prevent flooding (and slowing us down) */
            printf("[controller] ERROR return value from poll(): %d\n", poll_ret);
        }

        poll_error_counter++;
        return 0;

    } else {

        parameter_update_poll();
        sensor_combined_poll();
        gps_poll();

        struct input_s input;
        input.gyro_x = _sensor_combined.gyro_rad_s[0];
        input.gyro_y = _sensor_combined.gyro_rad_s[1];
        input.gyro_z = _sensor_combined.gyro_rad_s[2];
        input.accel_x = _sensor_combined.accelerometer_m_s2[0];
        input.accel_y = _sensor_combined.accelerometer_m_s2[1];
        input.accel_z = _sensor_combined.accelerometer_m_s2[2];
        input.static_pres = (_init_static - _sensor_combined.baro_pres_mbar)*100; // 1 mbar == 100 pa
        input.diff_pres = (_sensor_combined.differential_pressure_pa < 0 ? 0 : _sensor_combined.differential_pressure_pa);

        if(_gps_init && _gps_new)
        {
            float conversion = (3.14159/(180*1e7)); //from 1/10th of a micro degree to radian
            input.gps_n = EARTH_RADIUS * (float)(_gps.lat - _init_lat) * conversion;
            input.gps_e = EARTH_RADIUS * cosf((float)_init_lat * conversion) * (float)(_gps.lon - _init_lon) * conversion;
            input.gps_h = (_gps.alt - _init_alt) / 1e3f;
            input.gps_Vg = _gps.vel_m_s;
            input.gps_course = _gps.cog_rad;
        }

        if(_time_to_run == 1)//_gps_init) // don't estimate unless you have gps
        {
            hrt_abstime curr_time = hrt_absolute_time();
            input.Ts = (prev_time_ != 0) ? (curr_time - prev_time_) * 0.000001f : 0.0f;
            prev_time_ = curr_time;

            struct output_s output;

            estimate(_params, input, output);

            vehicle_state_publish(output);
            _time_to_run = -1;
        }
        _time_to_run++;

        return input.gps_n;
    }
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

void estimator_base::parameter_update_poll()
{
  bool updated;

  /* Check if param status has changed */
  orb_check(_params_sub, &updated);

  if (updated) {
    struct parameter_update_s param_update;
    orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
    parameters_update();
  }
}

void estimator_base::sensor_combined_poll()
{
    bool updated;

    orb_check(_sensor_combined_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);
    }

    if(!_baro_init)
    {
        _init_static = _sensor_combined.baro_pres_mbar;
        _baro_init = true;
    }
}

//void estimator_base::airspeed_poll()
//{
//    _airspeed_new = false;

//    orb_check(_airspeed_sub, &_airspeed_new);

//    if (_airspeed_new) {
//        orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);
//    }
//}

void estimator_base::gps_poll()
{
    _gps_new = false;

    orb_check(_gps_sub, &_gps_new);

    if (_gps_new) {
        orb_copy(ORB_ID(vehicle_gps_position), _gps_sub, &_gps);
    }

    if (_gps.fix_type < 3) {
        _gps_new = false;
    }

    if (_gps_new && !_gps_init) {
        _gps_init = true;
        _init_lon = _gps.lon;
        _init_lat = _gps.lat;
        _init_alt = _gps.alt;
    }
}

void estimator_base::vehicle_state_publish(output_s &output)
{
    _vehicle_state.position[0] = output.pn;
    _vehicle_state.position[1] = output.pe;
    _vehicle_state.position[2] = output.h;
    _vehicle_state.Va = output.Va;
    _vehicle_state.alpha = output.alpha;
    _vehicle_state.beta = output.beta;
    _vehicle_state.phi = output.phi;
    _vehicle_state.theta = output.theta;
    _vehicle_state.psi = output.psi;
    _vehicle_state.chi = output.chi;
    _vehicle_state.p = output.p;
    _vehicle_state.q = output.q;
    _vehicle_state.r = output.r;
    _vehicle_state.Vg = output.Vg;
    _vehicle_state.wn = output.wn;
    _vehicle_state.we = output.we;

    _vehicle_state.timestamp = hrt_absolute_time();

    if (_vehicle_state_pub > 0) {
        orb_publish(ORB_ID(vehicle_state), _vehicle_state_pub, &_vehicle_state);

    } else {
        _vehicle_state_pub = orb_advertise(ORB_ID(vehicle_state), &_vehicle_state);
    }
}
