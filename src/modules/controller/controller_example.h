#ifndef CONTROLLER_EXAMPLE_H
#define CONTROLLER_EXAMPLE_H

#include "controller_base.h"

class controller_example : public controller_base
{
public:
    controller_example();
private:
    virtual void control(const struct params_s &params, const struct input_s &input, struct output_s &output);
};

#endif // CONTROLLER_EXAMPLE_H
