#include <vector>
#include "pancake.h"

static double x_spatula[4];
static double x_object[4];


void get_object_state(double xs[4], double xd[4])
{
    int i;

    for (i=0; i<4; i++)
    {
        xs[i] = x_spatula[i];
        xd[i] = x_object[i] - x_spatula[i];
    }
}


void set_object_state(const std::vector<double> xs, const std::vector<double> xo)
{
    int i;

    x_spatula[0] = xs[0];
    x_spatula[1] = xs[1];
    x_spatula[2] = xs[3];
    x_spatula[3] = xs[4];

    x_object[0] = xo[0];
    x_object[1] = xo[1];
    x_object[2] = xo[3];
    x_object[3] = xo[4];
}
