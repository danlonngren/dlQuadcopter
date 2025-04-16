#ifndef UTILS_h
#define UTILS_h

class utils
{

private:
    int out;

public:
    utils();
    long Pyth(long x, long y, long z);
    int ConvertStick(int in);
    int ConvertThrottle(int in);
    float Gauss(float x, float a);
    float Positive(float x, float a);
    float Negative(float x, float a);
    float positiveNL(float x);
    float negativeNL(float x);
    float Limiter(float input, float upperLimit, float lowerLimit);
};

#endif
