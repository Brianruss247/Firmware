#include "controller_base.h"

controller_base::controller_base()
{
    _params_sub = orb_subscribe(ORB_ID(parameter_update));
    _vehicle_state_sub = orb_subscribe(ORB_ID(vehicle_state));
    _controller_commands_sub = orb_subscribe(ORB_ID(controller_commands));
    _manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    fds[0].fd = _vehicle_state_sub;
    fds[0].events = POLLIN;
    poll_error_counter = 0;

    memset(&_vehicle_state, 0, sizeof(_vehicle_state));
    memset(&_controller_commands, 0, sizeof(_controller_commands));
    memset(&_manual_control_sp, 0, sizeof(_manual_control_sp));
    memset(&_actuators, 0, sizeof(_actuators));
    memset(&_params, 0, sizeof(_params));

    _params_handles.trim_e         = param_find("UAVBOOK_TRIM_E");
    _params_handles.trim_a         = param_find("UAVBOOK_TRIM_A");
    _params_handles.trim_r         = param_find("UAVBOOK_TRIM_R");
    _params_handles.trim_t         = param_find("UAVBOOK_TRIM_T");
    _params_handles.pwm_rad_e      = param_find("UABBOOK_PWM_RAD_E");
    _params_handles.pwm_rad_a      = param_find("UABBOOK_PWM_RAD_A");
    _params_handles.pwm_rad_r      = param_find("UABBOOK_PWM_RAD_R");
    _params_handles.alt_toz        = param_find("UAVBOOK_ALT_TOZ");
    _params_handles.alt_hz         = param_find("UAVBOOK_ALT_HZ");
    _params_handles.tau            = param_find("UAVBOOK_TAU");
    _params_handles.course_kp      = param_find("UAVBOOK_COURSE_KP");
    _params_handles.course_kd      = param_find("UAVBOOK_COURSE_KD");
    _params_handles.course_ki      = param_find("UAVBOOK_COURSE_KI");
    _params_handles.roll_kp        = param_find("UAVBOOK_ROLL_KP");
    _params_handles.roll_kd        = param_find("UAVBOOK_ROLL_KD");
    _params_handles.roll_ki        = param_find("UAVBOOK_ROLL_KI");
    _params_handles.pitch_kp       = param_find("UAVBOOK_PITCH_KP");
    _params_handles.pitch_kd       = param_find("UAVBOOK_PITCH_KD");
    _params_handles.pitch_ki       = param_find("UAVBOOK_PITCH_KI");
    _params_handles.pitch_ff       = param_find("UAVBOOK_PITCH_FF");
    _params_handles.airspeed_pitch_kp       = param_find("UAVBOOK_AIRSPEED_PITCH_KP");
    _params_handles.airspeed_pitch_kd       = param_find("UAVBOOK_AIRSPEED_PITCH_KD");
    _params_handles.airspeed_pitch_ki       = param_find("UAVBOOK_AIRSPEED_PITCH_KI");
    _params_handles.airspeed_throttle_kp    = param_find("UAVBOOK_AIRSPEED_THROTTLE_KP");
    _params_handles.airspeed_throttle_kd    = param_find("UAVBOOK_AIRSPEED_THROTTLE_KD");
    _params_handles.airspeed_throttle_ki    = param_find("UAVBOOK_AIRSPEED_THROTTLE_KI");
    _params_handles.altitude_kp    = param_find("UAVBOOK_ALTITUDE_KP");
    _params_handles.altitude_kd    = param_find("UAVBOOK_ALTITUDE_KD");
    _params_handles.altitude_ki    = param_find("UAVBOOK_ALTITUDE_KI");
    _params_handles.beta_kp        = param_find("UAVBOOK_BETA_KP");
    _params_handles.beta_kd        = param_find("UAVBOOK_BETA_KD");
    _params_handles.beta_ki        = param_find("UAVBOOK_BETA_KI");
    _params_handles.max_e          = param_find("UAVBOOK_MAX_E");
    _params_handles.max_a          = param_find("UAVBOOK_MAX_A");
    _params_handles.max_r          = param_find("UAVBOOK_MAX_R");
    _params_handles.max_t          = param_find("UAVBOOK_MAX_T");

    parameters_update();
}

float controller_base::spin()
{
    /* wait for state update of 2 file descriptor for 20 ms */
    int poll_ret = poll(fds, 1, 20);

    if (poll_ret < 0) {
        /* this is seriously bad - should be an emergency */
        if (poll_error_counter < 10 || poll_error_counter % 50 == 0) {
            /* use a counter to prevent flooding (and slowing us down) */
            printf("[controller] ERROR return value from poll(): %d\n", poll_ret);
        }

        poll_error_counter++;
        return -1;
    } else {

        parameter_update_poll();
        vehicle_state_poll();
        controller_commads_poll();
        manual_control_poll();

        struct input_s input;
        input.h = _vehicle_state.position[2];
        input.va = _vehicle_state.Va;
        input.phi = _vehicle_state.phi;
        input.theta = _vehicle_state.theta;
        input.chi = _vehicle_state.chi;
        input.p = _vehicle_state.p;
        input.q = _vehicle_state.q;
        input.r = _vehicle_state.r;
        input.Va_c = _controller_commands.Va_c;
        input.h_c = _controller_commands.h_c;
        input.chi_c = _controller_commands.chi_c;
        input.Ts = 0.01f;  //todo: fix this to be the actual time step

        struct output_s output;

        control(_params, input, output);

        convert_to_pwm(output);

        pilot_override(output);

        actuator_controls_publish(output);
        return input.q;
    }
}

int controller_base::parameters_update()
{
    param_get(_params_handles.trim_e, &_params_extra.trim_e);
    param_get(_params_handles.trim_a, &_params_extra.trim_a);
    param_get(_params_handles.trim_r, &_params_extra.trim_r);
    param_get(_params_handles.trim_t, &_params_extra.trim_t);

    param_get(_params_handles.pwm_rad_e, &_params_extra.pwm_rad_e);
    param_get(_params_handles.pwm_rad_a, &_params_extra.pwm_rad_a);
    param_get(_params_handles.pwm_rad_r, &_params_extra.pwm_rad_r);

    param_get(_params_handles.alt_toz, &_params.alt_toz);
    param_get(_params_handles.alt_hz, &_params.alt_hz);
    param_get(_params_handles.tau, &_params.tau);

    param_get(_params_handles.course_kp, &_params.c_kp);
    param_get(_params_handles.course_kd, &_params.c_kd);
    param_get(_params_handles.course_ki, &_params.c_ki);

    param_get(_params_handles.roll_kp, &_params.r_kp);
    param_get(_params_handles.roll_kd, &_params.r_kd);
    param_get(_params_handles.roll_ki, &_params.r_ki);

    param_get(_params_handles.pitch_kp, &_params.p_kp);
    param_get(_params_handles.pitch_kd, &_params.p_kd);
    param_get(_params_handles.pitch_ki, &_params.p_ki);
    param_get(_params_handles.pitch_ff, &_params.p_ff);

    param_get(_params_handles.airspeed_pitch_kp, &_params.a_p_kp);
    param_get(_params_handles.airspeed_pitch_kd, &_params.a_p_kd);
    param_get(_params_handles.airspeed_pitch_ki, &_params.a_p_ki);

    param_get(_params_handles.airspeed_throttle_kp, &_params.a_t_kp);
    param_get(_params_handles.airspeed_throttle_kd, &_params.a_t_kd);
    param_get(_params_handles.airspeed_throttle_ki, &_params.a_t_ki);

    param_get(_params_handles.altitude_kp, &_params.a_kp);
    param_get(_params_handles.altitude_kd, &_params.a_kd);
    param_get(_params_handles.altitude_ki, &_params.a_ki);

    param_get(_params_handles.beta_kp, &_params.b_kp);
    param_get(_params_handles.beta_kd, &_params.b_kd);
    param_get(_params_handles.beta_ki, &_params.b_ki);

    param_get(_params_handles.max_e, &_params.max_e);
    param_get(_params_handles.max_a, &_params.max_a);
    param_get(_params_handles.max_r, &_params.max_r);
    param_get(_params_handles.max_t, &_params.max_t);

    return OK;
}

void controller_base::parameter_update_poll()
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

void controller_base::vehicle_state_poll()
{
    bool updated;

    /* get the state */
    orb_check(_vehicle_state_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_state), _vehicle_state_sub, &_vehicle_state);
    }
}

void controller_base::controller_commads_poll()
{
    bool updated;

    /* get the state */
    orb_check(_controller_commands_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(controller_commands), _controller_commands_sub, &_controller_commands);
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

void controller_base::convert_to_pwm(controller_base::output_s &output)
{
    output.delta_e = _params_extra.trim_e + output.delta_e*_params_extra.pwm_rad_e;
    output.delta_a = _params_extra.trim_a + output.delta_a*_params_extra.pwm_rad_a;
    output.delta_r = _params_extra.trim_r + output.delta_r*_params_extra.pwm_rad_r;
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
    /* for HIL purposes output the intermetiate commands */
    _actuators.control[4] = (isfinite(output.phi_c)) ? output.phi_c : 0.0f;
    _actuators.control[5] = (isfinite(output.theta_c)) ? output.theta_c : 0.0f;

    _actuators.timestamp = hrt_absolute_time();
    //printf("%d \n", (int)(100*_actuators.control[3]));

    if (_actuators_0_pub > 0) {
        orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);

    } else {
        _actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
    }
}
