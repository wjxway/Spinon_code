#ifndef _FASTMATH_
#define _FASTMATH_

#include "Arduino.h"
#include <math.h>

//Accuracy @ 6%
//sufficient for FFT usage
inline float Fast_cos(float x) noexcept
{
    constexpr float tp = 1.0f/(2.0f*PI);
    
    x *= tp;
    x -= 0.25f + (float)((int)(x + 0.25f));
    x *= 16.0f * (abs(x) - 0.5f);
    #if EXTRA_PRECISION
    x += 0.225f * x * (abs(x) - 1.0f);
    #endif
    return x;
}

//T could be float or double
template<class T>
inline int16_t Fast_floor_int(T x) noexcept
{
    int16_t val=(int16_t)x;
    if(val==x) return x;
    else if(x>0) return val;
    else return val-1;
}

//T could be float or double
template<class T>
inline T Fast_floor(T x) noexcept
{
    return T(Fast_floor_int(x));
}

//accuracy @ 0.4%
inline float Fast_exp(float x) noexcept
{
    constexpr float DIVLN2=1.4426950408889634074f;

    union
    {
        float f;
        uint32_t i;
    } u={0}, v={0};

    float frac=x*DIVLN2;
    int16_t fl=Fast_floor_int(frac);
    frac-=fl;

    //exponent
    u.i=(fl+127)<<23;

    //value
    v.f=(0.6655544926090462f + 0.33284502591696163f*frac)*frac+0.9984f;

    //sum up
    u.i=v.i+u.i-(127<<23);

    return u.f;
}


#endif