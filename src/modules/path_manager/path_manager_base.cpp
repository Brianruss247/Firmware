#include "path_manager_base.h"

path_manager_base::path_manager_base()
{
    _params_sub = orb_subscribe(ORB_ID(parameter_update));
    _vehicle_state_sub = orb_subscribe(ORB_ID(vehicle_state));
    _new_waypoint_sub = orb_subscribe(ORB_ID(new_waypoint));
    fds[0].fd = _vehicle_state_sub;
    fds[0].events = POLLIN;
    poll_error_counter = 0;

    memset(&_vehicle_state, 0, sizeof(_vehicle_state));
    memset(&_current_path, 0, sizeof(_current_path));
    memset(&_params, 0, sizeof(_params));

    for(int i=0;i<SIZE_WAYPOINT_ARRAY;i++)
    {
        memset(&_waypoints[i], 0, sizeof(_waypoints[i]));
        _waypoints[i].w[2] = 30.0f;
    }
    _num_waypoints = 0;
    _ptr_a = &_waypoints[0];

    _params_handles.R_min      = param_find("UAVBOOK_R_MIN");

    parameters_update();

    _current_path_pub = orb_advertise(ORB_ID(current_path), &_current_path);
}

float path_manager_base::spin()
{
    /* wait for state update of 2 file descriptor for 20 ms */
    int poll_ret = poll(fds, 1, 100);//20);

    if (poll_ret < 0) {
        /* this is seriously bad - should be an emergency */
        if (poll_error_counter < 10 || poll_error_counter % 50 == 0) {
            /* use a counter to prevent flooding (and slowing us down) */
            printf("[path_manager] ERROR return value from poll(): %d\n", poll_ret);
        }

        poll_error_counter++;
        return -1;
    } else {

        parameter_update_poll();
        vehicle_state_poll();
        new_waypoint_poll();

        struct input_s input;
        input.pn = _vehicle_state.position[0];               /** position north */
        input.pe = _vehicle_state.position[1];               /** position east */
        input.h = _vehicle_state.position[2];                /** altitude */
//        input.chi = _vehicle_state.chi;

        struct output_s output;

        manage(_params, input, output);

        current_path_publish(output);
        return input.h;
    }
}

int path_manager_base::parameters_update()
{
    param_get(_params_handles.R_min, &_params.R_min);

    return OK;
}

void path_manager_base::parameter_update_poll()
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

void path_manager_base::vehicle_state_poll()
{
    bool updated;

    /* get the state */
    orb_check(_vehicle_state_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_state), _vehicle_state_sub, &_vehicle_state);
    }
}

void path_manager_base::new_waypoint_poll()
{
    bool updated;

    /* get the state */
    orb_check(_new_waypoint_sub, &updated);

    if (updated) {
        struct new_waypoint_s new_waypoint;
        orb_copy(ORB_ID(new_waypoint), _new_waypoint_sub, &new_waypoint);
//        if(new_waypoint.set_current)
//        { ; } //set this waypoint to be exicuted now by placing it in the array and moving ptr_a
//        else {;}
        _waypoints[_num_waypoints].w[0] = new_waypoint.w[0];
        _waypoints[_num_waypoints].w[1] = new_waypoint.w[1];
        _waypoints[_num_waypoints].w[2] = new_waypoint.w[2];
        _waypoints[_num_waypoints].chi_d = new_waypoint.chi_d;
        _waypoints[_num_waypoints].chi_valid = new_waypoint.chi_valid;
        _waypoints[_num_waypoints].Va_d = new_waypoint.Va_d;
        _num_waypoints++;
        warnx("new waypoint received");
    }
}

void path_manager_base::current_path_publish(output_s &output)
{
    /* publish actuator controls */
    _current_path.flag = output.flag;
    _current_path.Va_d = output.Va_d;
    for(int i=0;i<3;i++)
    {
        _current_path.r[i] = output.r[i];
        _current_path.q[i] = output.q[i];
        _current_path.c[i] = output.c[i];
    }
    _current_path.rho = output.rho;
    _current_path.lambda = output.lambda;

    _current_path.timestamp = hrt_absolute_time();

    if (_current_path_pub > 0) {
        orb_publish(ORB_ID(current_path), _current_path_pub, &_current_path);

    } else {
        _current_path_pub = orb_advertise(ORB_ID(current_path), &_current_path);
    }
}
