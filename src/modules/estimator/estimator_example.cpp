#include "estimator_example.h"

estimator_example::estimator_example()
{

}

void estimator_example::estimate(const params_s &params, const input_s &input, output_s &output)
{

    output.alpha = 0;
    output.beta = 0;
    output.pn = 70;
}
