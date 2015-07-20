#include "path_follower_example.h"

path_follower_example::path_follower_example()
{

}

void path_follower_example::follow(const params_s &params, const input_s &input, output_s &output)
{
    output.Va_c = input.Va_d;
}
