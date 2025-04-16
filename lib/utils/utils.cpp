#include "utils.h"

#include <math.h>

utils::utils()
{
    // cuntructor will be called each time an object is created
}

long utils::Pyth(long x, long y, long z)
{
    return sqrt((x * x) + (y * y) + (z * z));
}

int utils::ConvertStick(int in)
{
    int in2 = in - 1500;
    int out = 0;

    if (in > 1500)
        out = ((in2 * in2) / 1000 + (in2) / 2 + 1500);
    else if (in < 1500)
        out = (1500 - (in2 * in2) / 1000 + in2 / 2);
    return out;
}

int utils::ConvertThrottle(int in)
{
    return sqrt(in - 1000) * 31.63 + 1000;;
}

float utils::Gauss(float x, float a)
{
    return exp((-1 * (x * x)) / (2 * a)); // centre is 0
}
float utils::Negative(float x, float a)
{
    return (a - x) / (2 * a); // centre is 0.5
}
float utils::Positive(float x, float a)
{
    return (a + x) / (2 * a); // centre is 0.5
}
float utils::positiveNL(float x) // from -10 to 10
{
    return (((x * x * x + x) / 2020.0) + 0.5); // centre is 0.5
}
float utils::negativeNL(float x) // from -10 to 10
{
    return (((-x * x * x - x) / 2020.0) + 0.5); // centre is 0.5
}
float utils::Limiter(float input, float upperLimit, float lowerLimit)
{
    if (input >= upperLimit)
        return upperLimit;
    else if (input <= (lowerLimit))
        return lowerLimit;
    return input;
}