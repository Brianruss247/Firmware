#ifndef PATH_MANAGER_EXAMPLE_H
#define PATH_MANAGER_EXAMPLE_H

#include "path_manager_base.h"

class path_manager_example : public path_manager_base
{
public:
    path_manager_example();
private:
    virtual void manage(const struct params_s &params, const struct input_s &input, struct output_s &output);
};

#endif // PATH_MANAGER_EXAMPLE_H
