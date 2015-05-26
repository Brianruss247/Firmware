#include "controller_example.h"

controller_example::controller_example()
{
}

void controller_example::control(const params_s &params, const input_s &input, output_s &output)
{
    output.delta_a = 0;
    output.delta_e = 0.05;
    output.delta_r = 0;
    output.delta_t = 0.75;
}
