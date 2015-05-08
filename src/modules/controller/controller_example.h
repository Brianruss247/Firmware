#ifndef CONTROLLER_EXAMPLE_H
#define CONTROLLER_EXAMPLE_H

#include "controller_base.h"

class controller_example: public controller_base
{
public:
    controller_example();
private:
    void virtual control();
};

#endif // CONTROLLER_EXAMPLE_H
