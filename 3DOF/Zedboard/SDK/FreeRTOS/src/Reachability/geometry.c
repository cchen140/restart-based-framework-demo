/*
 * geometry.c
 *
 *  Created on: Mar 25, 2014
 */

#include "geometry.h"
#include <stdio.h>
#include <float.h>
#include <math.h>

double interval_width(Interval* i)
{
	return i->max - i->min;
}

double hyperrectange_max_width(HyperRectangle* rect)
{
	double rv = 0;

	for (int d = 0; d < NUM_DIMS; ++d)
	{
		double min = rect->dims[d].min;
		double max = rect->dims[d].max;
		double dif = max - min;

		if (!isfinite(min) || !isfinite(max) || !isfinite(dif))
		{
			rv = DBL_MAX;
			break;
		}

		if (dif > rv)
			rv = dif;
	}

	return rv;
}

bool hyperrectangle_contains(HyperRectangle* outside, HyperRectangle* inside, bool printErrors)
{
	bool rv = true;

	for (int d = 0; d < NUM_DIMS; ++d)
	{
		if ((inside->dims[d].min < outside->dims[d].min) || (inside->dims[d].max > outside->dims[d].max))
		{
			if (printErrors && (inside->dims[d].min < outside->dims[d].min))
				printf("inside->dim[%d].min (%f) < outside->dim[%d].min (%f)\n",
						d, inside->dims[d].min, d, outside->dims[d].min);
			else if (printErrors)
				printf("inside->dim[%d].max (%f) < outside->dim[%d].max (%f)\n",
						d, inside->dims[d].max, d, outside->dims[d].max);

			rv = false;
			break;
		}
	}

	return rv;
}

void hyperrectangle_grow_to_convex_hull(HyperRectangle* grower, HyperRectangle* contained)
{
	for (int d = 0; d < NUM_DIMS; ++d)
	{
		if (contained->dims[d].min < grower->dims[d].min)
			grower->dims[d].min = contained->dims[d].min;

		if (contained->dims[d].max > grower->dims[d].max)
			grower->dims[d].max = contained->dims[d].max;
	}
}

void print(HyperRectangle* r)
{
	printf("[HyperRectangle");

	for (int d=0; d < NUM_DIMS; ++d)
		printf(" (%f, %f)", r->dims[d].min, r->dims[d].max);

	printf("]");
}

void println(HyperRectangle* r)
{
	print(r);
	printf("\n");
}

void hyperrectangle_bloat(HyperRectangle* out, double from[NUM_DIMS], double width)
{
	for (int d = 0; d < NUM_DIMS; ++d)
	{
		out->dims[d].min = from[d] - width;
		out->dims[d].max = from[d] + width;
	}
}
