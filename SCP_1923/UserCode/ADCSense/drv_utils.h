/*
    Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

    This file is part of the VESC firmware.

    The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef UTILS_H_
#define UTILS_H_

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#ifndef PI
#define PI (3.1415926535f)
#endif

float utils_clamp(float preferred, float min, float max);
float utils_min_of_2(float a, float b);
float utils_max_of_2(float a, float b);
int utils_truncate_number(float *number, float min, float max);
int utils_truncate_number_int(int *number, int min, int max);

// Return the sign of the argument. -1 if negative, 1 if zero or positive.
#define SIGN_F(x) (((x) < 0.f) ? -1 : 1)

// Squared
#define SQUARE(x) ((x) * (x))

// nan and infinity check for floats
#define UTILS_IS_INF_F(x) ((x) == (1.0f / 0.0f) || (x) == (-1.0f / 0.0f))
#define UTILS_IS_NAN(x) isnan(x)
#define UTILS_NAN_ZERO_F(x) (x = UTILS_IS_NAN(x) ? 0.f : x)

/**
 * A simple low pass filter.
 *
 * @param value
 * The filtered value.
 *
 * @param sample
 * Next sample.
 *
 * @param filter_constant
 * Filter constant. Range 0.0 to 1.0, where 1.0 gives the unfiltered value.
 */
#define UTILS_LP_FAST(value, sample, filter_constant) (value - ((filter_constant) * ((value) - (sample))))

#endif /* UTILS_H_ */
