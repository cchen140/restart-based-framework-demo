// Dynamics header file for real-time reachability

#ifndef DYNAMICS_H_

#define DYNAMICS_H_

#include <stdbool.h>
#include "geometry.h"

/**
 * Get the bounds on the derivative in a region of space at a range of times
 */

double get_derivative_bounds(HyperRectangle *rect, int faceIndex, int controller_type);
double find_F_max(HyperRectangle* rect);

#endif
