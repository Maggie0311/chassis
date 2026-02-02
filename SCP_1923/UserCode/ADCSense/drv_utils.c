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

#include "drv_utils.h"

/**
 * @brief *value向goal的方向移动step，如果超过了就直接等于目标值。
 *
 * @param value
 * @param goal
 * @param step
 */
void utils_step_towards(float *value, float goal, float step)
{
    if (*value < goal)
    {
        if ((*value + step) < goal)
        {
            *value += step;
        }
        else
        {
            *value = goal;
        }
    }
    else if (*value > goal)
    {
        if ((*value - step) > goal)
        {
            *value -= step;
        }
        else
        {
            *value = goal;
        }
    }
}
float utils_calc_ratio(float low, float high, float val)
{
    return (val - low) / (high - low);
}

/**
 * Make sure that 0 <= angle < 360
 *
 * @param angle
 * The angle to normalize.
 */
void utils_norm_angle(float *angle)
{
    *angle = fmodf(*angle, 360.0f);

    if (*angle < 0.0f)
    {
        *angle += 360.0f;
    }
}

/**
 * Make sure that -pi <= angle < pi,
 *
 * TODO: Maybe use fmodf instead?
 *
 * @param angle
 * The angle to normalize in radians.
 * WARNING: Don't use too large angles.
 */
void utils_norm_angle_rad(float *angle)
{
    while (*angle < -PI)
    {
        *angle += 2.0f * PI;
    }

    while (*angle > PI)
    {
        *angle -= 2.0f * PI;
    }
}
/** @brief 将number限制在最大最小值内，如果限制了就返回1  */

int utils_truncate_number(float *number, float min, float max)
{
    int did_trunc = 0;

    if (*number > max)
    {
        *number = max;
        did_trunc = 1;
    }
    else if (*number < min)
    {
        *number = min;
        did_trunc = 1;
    }

    return did_trunc;
}

int utils_truncate_number_int(int *number, int min, int max)
{
    int did_trunc = 0;

    if (*number > max)
    {
        *number = max;
        did_trunc = 1;
    }
    else if (*number < min)
    {
        *number = min;
        did_trunc = 1;
    }

    return did_trunc;
}
/**
 * @brief 判断number对应地址的数据的绝对值是否超过max
 * 		  超过了就把number限制，并返回1
 *
 * @param number
 * @param max
 * @return int
 */
int utils_truncate_number_abs(float *number, float max)
{
    int did_trunc = 0;

    if (*number > max)
    {
        *number = max;
        did_trunc = 1;
    }
    else if (*number < -max)
    {
        *number = -max;
        did_trunc = 1;
    }

    return did_trunc;
}

float utils_map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int utils_map_int(int x, int in_min, int in_max, int out_min, int out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Limit a value between an upper and lower bound, and when this value exceeds the range of minimum and maximum values,
 * choose a value between the minimum and maximum values to use.
 * @param preferred
 * The preferred value
 * @param min
 * The lower bound
 * @param max
 * The upper bound
 * @return
 * The clamped value
 */
float utils_clamp(float preferred, float min, float max)
{
    if ((min < preferred) && (preferred < max))
    {
        return preferred;
    }
    else if (preferred >= max)
    {
        return max;
    }
    else if (preferred <= min)
    {
        return min;
    }
    return 0.f;
}

/**
 * Truncate absolute values less than tres to zero. The value
 * tres will be mapped to 0 and the value max to max.
 */
void utils_deadband(float *value, float tres, float max)
{
    if (fabsf(*value) < tres)
    {
        *value = 0.0f;
    }
    else
    {
        float k = max / (max - tres);
        if (*value > 0.0f)
        {
            *value = k * *value + max * (1.0f - k);
        }
        else
        {
            *value = -(k * -*value + max * (1.0f - k));
        }
    }
}

/**
 * Get the difference between two angles. Will always be between -180 and +180 degrees.
 * @param angle1
 * The first angle
 * @param angle2
 * The second angle
 * @return
 * The difference between the angles
 */
float utils_angle_difference(float angle1, float angle2)
{
    //	utils_norm_angle(&angle1);
    //	utils_norm_angle(&angle2);
    //
    //	if (fabsf(angle1 - angle2) > 180.0) {
    //		if (angle1 < angle2) {
    //			angle1 += 360.0;
    //		} else {
    //			angle2 += 360.0;
    //		}
    //	}
    //
    //	return angle1 - angle2;

    // Faster in most cases
    float difference = angle1 - angle2;
    while (difference < -180.0f)
        difference += 2.0f * 180.0f;
    while (difference > 180.0f)
        difference -= 2.0f * 180.0f;
    return difference;
}

/**
 * Get the difference between two angles. Will always be between -pi and +pi radians.
 * @param angle1
 * The first angle in radians
 * @param angle2
 * The second angle in radians
 * @return
 * The difference between the angles in radians
 */
float utils_angle_difference_rad(float angle1, float angle2)
{
    float difference = angle1 - angle2;
    while (difference < -PI)
        difference += 2.0f * PI;
    while (difference > PI)
        difference -= 2.0f * PI;
    return difference;
}

/**
 * Get the min value of two integer values
 * @param a
 * First value
 * @param b
 * Second value
 * @return
 * The min value
 */
int utils_min_2_int(int a, int b)
{
    return (a < b) ? a : b;
}

/**
 * Get the max value of two integer values
 * @param a
 * First value
 * @param b
 * Second value
 * @return
 * The max value
 */
int utils_max_2_int(int a, int b)
{
    return (a > b) ? a : b;
}

/**
 * Get the min value of two values
 * @param a
 * First value
 * @param b
 * Second value
 * @return
 * The min value
 */
float utils_min_of_2(float a, float b)
{
    float min = a;
    min = (min < b) ? min : b;
    return min;
}

/**
 * Get the max value of two values
 * @param a
 * First value
 * @param b
 * Second value
 * @return
 * The max value
 */
float utils_max_of_2(float a, float b)
{
    float max = a;
    max = (max > b) ? max : b;
    return max;
}

/**
 * Get the max value of four values
 * @param a
 * First value
 * @param b
 * Second value
 * @param c
 * Third value
 * @param d
 * Fourth value
 * @return
 * The max value
 */
float utils_max_of_4(float a, float b, float c, float d)
{
    float max = a;

    max = (max > b) ? max : b;
    max = (max > c) ? max : c;
    max = (max > d) ? max : d;

    return max;
}

/**
 * Get the middle value of three values
 *
 * @param a
 * First value
 *
 * @param b
 * Second value
 *
 * @param c
 * Third value
 *
 * @return
 * The middle value
 */
float utils_middle_of_3(float a, float b, float c)
{
    float middle;

    if ((a <= b) && (a <= c))
    {
        middle = (b <= c) ? b : c;
    }
    else if ((b <= a) && (b <= c))
    {
        middle = (a <= c) ? a : c;
    }
    else
    {
        middle = (a <= b) ? a : b;
    }
    return middle;
}

/**
 * Get the middle value of three values
 *
 * @param a
 * First value
 *
 * @param b
 * Second value
 *
 * @param c
 * Third value
 *
 * @return
 * The middle value
 */
int utils_middle_of_3_int(int a, int b, int c)
{
    int middle;

    if ((a <= b) && (a <= c))
    {
        middle = (b <= c) ? b : c;
    }
    else if ((b <= a) && (b <= c))
    {
        middle = (a <= c) ? a : c;
    }
    else
    {
        middle = (a <= b) ? a : b;
    }
    return middle;
}

/**
 * Truncate the magnitude of a vector.
 * 向量的缩放，x为横向，y为纵向，max为合成向量最大的模，这个函数是判断合成是否超过了最大值，超过了就成比例放缩
 *
 * @param x
 * The first component.
 *
 * @param y
 * The second component.
 *
 * @param max
 * The maximum magnitude.
 *
 * @return
 * True if saturation happened, false otherwise
 */
bool utils_saturate_vector_2d(float *x, float *y, float max)
{
    bool retval = false;
    float mag = sqrtf(SQUARE(*x) + SQUARE(*y));
    max = fabsf(max);

    if (mag < 1e-10f)
    {
        mag = 1e-10f;
    }

    if (mag > max)
    {
        const float f = max / mag;
        *x *= f;
        *y *= f;
        retval = true;
    }

    return retval;
}

/**
 * Calculate the values with the lowest magnitude.
 *
 * @param va
 * The first value.
 *
 * @param vb
 * The second value.
 *
 * @return
 * The value with the lowest magnitude.
 */
float utils_min_abs(float va, float vb)
{
    float res;
    if (fabsf(va) < fabsf(vb))
    {
        res = va;
    }
    else
    {
        res = vb;
    }

    return res;
}

/**
 * Calculate the values with the highest magnitude.
 *
 * @param va
 * The first value.
 *
 * @param vb
 * The second value.
 *
 * @return
 * The value with the highest magnitude.
 */
float utils_max_abs(float va, float vb)
{
    float res;
    if (fabsf(va) > fabsf(vb))
    {
        res = va;
    }
    else
    {
        res = vb;
    }

    return res;
}

/**
 * Write the flag to the specific bit of a number.
 * @param FlagChar
 * The numebr to be writed in.
 * @param Number
 * The bit specified to writed in.
 * @param Flag
 * The number to write into the bit.
 */
void utils_write_bit(char *FlagChar, unsigned char Num, char Flag)
{
    if (Num > 7)
    {
        return;
    }
    else
    {
        if (Flag)
        {
            *FlagChar = *FlagChar | (1 << Num);
        }
        else
        {
            *FlagChar = *FlagChar & (~(1 << Num));
        }
    }
}

/**
 * Read the specific bit of a number.
 * @param Flags_IN
 * The numebr to read.
 * @param Num
 * The bit specified to read.
 * @return
 * The value on the specfied bit.
 */
char utils_read_bit(char Flags_IN, unsigned char Num)
{
    if (Num > 7)
    {
        return 0;
    }
    else
    {
        if (Flags_IN & (1 << Num))
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
}

/**
 * Make sure that min <= number < max
 *
 * @param number
 * The number to normalize.
 * @param min
 * The min number in the circle.
 * @param len
 * The length of the circle.
 */
void utils_norm_circle_number(float *number, float min, float len)
{
    *number = fmodf(*number - min, len) + min;

    while (*number < min)
    {
        *number += len;
    }
}

/**
 * Get the difference between two numbers. Will always be between -len/2 and +len/2.
 * @param number1
 * The first number
 * @param number2
 * The second number
 * @param min
 * The min number in the circle.
 * @param len
 * The length of the circle.
 * @return
 * The difference between the numbers
 */
float utils_circle_number_difference(float number1, float number2, float min, float len)
{
    float difference = number1 - number2;
    while (difference < -len / 2)
        difference += len;
    while (difference > len / 2)
        difference -= len;
    return difference;
}

/**
 * A low pass filter for circled number.
 * @param value
 * The filtered value.
 * @param sample
 * Next sample.
 * @param filter_constant
 * Filter constant. Range 0.0 to 1.0, where 1.0 gives the unfiltered value.
 * @param min
 * The min number in the circle.
 * @param len
 * The length of the circle.
 * @return
 * The output number of the filter.
 */
float utils_circle_number_LowPass_Filter(float value, float sample, float filter_constant, float min, float len)
{
    float middle = min + len / 2;
    float out;
    if ((sample < middle) && (value > middle))
        out = UTILS_LP_FAST(value, sample + len, filter_constant);
    else if ((sample > middle) && (value < middle))
        out = UTILS_LP_FAST(value, sample - len, filter_constant);
    else
        out = UTILS_LP_FAST(value, sample, filter_constant);
    utils_norm_circle_number(&out, min, len);
    return out;
}

/**
 * Get where the first "1" bit appears from the left side.
 * @param value
 * The number to be search.
 * @param pos
 * Where the result will be stored.
 * @return
 * Whether the "1" bit is found.
 */
bool utils_get_least_significant_setbit_pos(uint32_t value, int *pos)
{
    if (!pos || !value)
        return false;
    uint8_t Pos = 0;
    while (!(value & (1 << Pos)))
    {
        if (Pos == 31)
            break;
        else
            ++Pos;
    }
    if (value & (1 << Pos))
    {
        *pos = Pos;
        return true;
    }
    else
        return false;
}
