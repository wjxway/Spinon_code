#include "FastMath.hpp"

constexpr float math::fast::cos(float x)
{
    constexpr float tp = 1.0f / (2.0f * M_PI);
    
    x *= tp;
    x -= 0.25f + (float)((int)(x + 0.25f));
    x *= 16.0f * (std::abs(x) - 0.5f);
    #if EXTRA_PRECISION
    x += 0.225f * x * (std::abs(x) - 1.0f);
    #endif
    return x;
}

constexpr float math::fast::exp(float x) noexcept
{
    constexpr float DIVLN2=1.4426950408889634074f;

    union
    {
        float f;
        uint32_t i;
    } u={0}, v={0};

    float frac=x*DIVLN2;
    int32_t fl= math::fast::floor_int(frac);
    frac-=fl;

    //exponent
    u.i=(fl+127)<<23;

    //value
    v.f=(0.6655544926090462f + 0.33284502591696163f*frac)*frac+0.9984f;

    //sum up
    u.i=v.i+u.i-(127<<23);

    return u.f;
}
