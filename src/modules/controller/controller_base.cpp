#include "controller_base.h"

controller_base::controller_base()
{
    _params_sub = orb_subscribe(ORB_ID(parameter_update));
    _vehicle_state_sub = orb_subscribe(ORB_ID(vehicle_state));
    _manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

    memset(&_vehicle_state, 0, sizeof(_vehicle_state));
    memset(&_manual_control_sp, 0, sizeof(_manual_control_sp));
    memset(&_actuators, 0, sizeof(_actuators));
    memset(&_params, 0, sizeof(_params));

//    _params_handles.roll_p         = param_find("UAVBOOK_ROLL_P");
//    _params_handles.roll_rate_p    = param_find("PF_ROLLRATE_P");
//    _params_handles.roll_rate_i    = param_find("PF_ROLLRATE_I");
//    _params_handles.roll_rate_d    = param_find("PF_ROLLRATE_D");
//    _params_handles.pitch_p        = param_find("PF_PITCH_P");
//    _params_handles.pitch_rate_p   = param_find("PF_PITCHRATE_P");
//    _params_handles.pitch_rate_i   = param_find("PF_PITCHRATE_I");
//    _params_handles.pitch_rate_d   = param_find("PF_PITCHRATE_D");
//    _params_handles.yaw_p          = param_find("PF_YAW_P");
//    _params_handles.yaw_rate_p     = param_find("PF_YAWRATE_P");
//    _params_handles.yaw_rate_i     = param_find("PF_YAWRATE_I");
//    _params_handles.yaw_rate_d     = param_find("PF_YAWRATE_D");
//    _params_handles.yaw_ff         = param_find("PF_YAW_FF");
//    _params_handles.yaw_rate_max   = param_find("PF_YAWRATE_MAX");
    _params_handles.trim_e         = param_find("UAVBOOK_TRIM_E");
    _params_handles.trim_a         = param_find("UAVBOOK_TRIM_A");
    _params_handles.trim_r         = param_find("UAVBOOK_TRIM_R");
    _params_handles.trim_t         = param_find("UAVBOOK_TRIM_T");
    _params_handles.alt_toz        = param_find("UAVBOOK_ALT_TOZ");
    _params_handles.alt_hz         = param_find("UAVBOOK_ALT_HZ");
    _params_handles.max_e          = param_find("UAVBOOK_MAX_E");
    _params_handles.max_a          = param_find("UAVBOOK_MAX_A");
    _params_handles.max_r          = param_find("UAVBOOK_MAX_R");
    _params_handles.max_t          = param_find("UAVBOOK_MAX_T");

    parameters_update();
}

void controller_base::spin()
{
    parameter_update_poll();
    vehicle_state_poll();
    manual_control_poll();

    struct input_s input;
    input.h = _vehicle_state.position[2];
    input.v_a = _vehicle_state.Va;
    input.phi = _vehicle_state.phi;
    input.theta = _vehicle_state.theta;
    input.chi = _vehicle_state.chi;
    input.p = _vehicle_state.p;
    input.q = _vehicle_state.q;
    input.r = _vehicle_state.r;

    struct output_s output;

    control(_params, input, output);

    pilot_override(output);

    actuator_controls_publish(output);
}

int controller_base::parameters_update()
{
  float v;

//  /* roll gains */
//  param_get(_params_handles.roll_p, &v);
//  _params.att_p(0) = v;
//  param_get(_params_handles.roll_rate_p, &v);
//  _params.rate_p(0) = v;
//  param_get(_params_handles.roll_rate_i, &v);
//  _params.rate_i(0) = v;
//  param_get(_params_handles.roll_rate_d, &v);
//  _params.rate_d(0) = v;

//  /* pitch gains */
//  param_get(_params_handles.pitch_p, &v);
//  _params.att_p(1) = v;
//  param_get(_params_handles.pitch_rate_p, &v);
//  _params.rate_p(1) = v;
//  param_get(_params_handles.pitch_rate_i, &v);
//  _params.rate_i(1) = v;
//  param_get(_params_handles.pitch_rate_d, &v);
//  _params.rate_d(1) = v;

//  /* yaw gains */
//  param_get(_params_handles.yaw_p, &v);
//  _params.att_p(2) = v;
//  param_get(_params_handles.yaw_rate_p, &v);
//  _params.rate_p(2) = v;
//  param_get(_params_handles.yaw_rate_i, &v);
//  _params.rate_i(2) = v;
//  param_get(_params_handles.yaw_rate_d, &v);
//  _params.rate_d(2) = v;

//  param_get(_params_handles.yaw_ff, &_params.yaw_ff);
//  param_get(_params_handles.yaw_rate_max, &_params.yaw_rate_max);
//  _params.yaw_rate_max = math::radians(_params.yaw_rate_max);

    param_get(_params_handles.trim_e, &v);
    _params.trims(0) = v;
    param_get(_params_handles.trim_a, &v);
    _params.trims(1) = v;
    param_get(_params_handles.trim_r, &v);
    _params.trims(2) = v;
    param_get(_params_handles.trim_t, &v);
    _params.trims(3) = v;
    param_get(_params_handles.alt_toz, &_params.alt_toz);
    param_get(_params_handles.alt_hz, &_params.alt_hz);
    param_get(_params_handles.max_e, &v);
    _params.max(0) = v;
    param_get(_params_handles.max_a, &v);
    _params.max(1) = v;
    param_get(_params_handles.max_r, &v);
    _params.max(2) = v;
    param_get(_params_handles.max_t, &v);
    _params.max(3) = v;


    return OK;
}

void controller_base::parameter_update_poll()
{
  bool updated;

  /* Check HIL state if vehicle status has changed */
  orb_check(_params_sub, &updated);

  if (updated) {
    struct parameter_update_s param_update;
    orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
    parameters_update();
  }
}

void controller_base::vehicle_state_poll()
{
    bool updated;

    /* get the state */
    orb_check(_vehicle_state_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_state), _vehicle_state_sub, &_vehicle_state);
    }
}

void controller_base::manual_control_poll()
{
  bool updated;

  /* get pilots inputs */
  orb_check(_manual_control_sp_sub, &updated);

  if (updated) {
    orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
  }
}

void controller_base::pilot_override(controller_base::output_s &output)
{
    if (_manual_control_sp.y > MANUAL_THRESHOLD || _manual_control_sp.y < -MANUAL_THRESHOLD)
    {
        output.delta_a = _manual_control_sp.y;
    }

    if (_manual_control_sp.x > MANUAL_THRESHOLD || _manual_control_sp.x < -MANUAL_THRESHOLD)
    {
        output.delta_e = -_manual_control_sp.x;
    }

    if (_manual_control_sp.r > MANUAL_THRESHOLD || _manual_control_sp.r < -MANUAL_THRESHOLD)
    {
        output.delta_r = _manual_control_sp.r;
    }

    // take min of commanded throttle and stick throttle
    output.delta_t = math::min(output.delta_t, _manual_control_sp.z);
}

void controller_base::actuator_controls_publish(output_s &output)
{
    /* publish actuator controls */
    _actuators.control[0] = (isfinite(output.delta_a)) ? output.delta_a : 0.0f;
    _actuators.control[1] = (isfinite(output.delta_e)) ? output.delta_e : 0.0f;
    _actuators.control[2] = (isfinite(output.delta_r)) ? output.delta_r : 0.0f;
    _actuators.control[3] = (isfinite(output.delta_t)) ? output.delta_t : 0.0f;
    _actuators.timestamp = hrt_absolute_time();

    if (_actuators_0_pub > 0) {
        orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);

    } else {
        _actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
    }
}
