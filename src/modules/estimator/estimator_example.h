#ifndef ESTIMATOR_EXAMPLE_H
#define ESTIMATOR_EXAMPLE_H

#include "estimator_base.h"

class estimator_example : public estimator_base
{
public:
    estimator_example();
private:
    virtual void estimate(const params_s &params, const input_s &input, output_s &output);
};

#endif // ESTIMATOR_EXAMPLE_H
