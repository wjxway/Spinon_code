#include "FastMath.hpp"

float math::fast::cos(const float x) noexcept
{
    float x1=x;

    constexpr float tp = 1.0f / (2.0f * M_PI);

    x1 *= tp;
    x1 -= 0.25f + (float)((int)(x1 + 0.25f));
    x1 *= 16.0f * (abs(x1) - 0.5f);
#if EXTRA_PRECISION
    x1 += 0.225f * x1 * (abs(x1) - 1.0f);
#endif
    return x1;
}

float math::fast::exp(const float x) noexcept
{
    constexpr float DIVLN2 = 1.4426950408889634074f;

    union
    {
        float f;
        uint32_t i;
    } u = {0}, v = {0};

    float frac = x * DIVLN2;
    int32_t fl = floor_int(frac);
    frac -= fl;

    // exponent
    u.i = (fl + 127) << 23;

    // value
    v.f = (0.6655544926090462f + 0.33284502591696163f * frac) * frac + 0.9984f;

    // sum up
    u.i = v.i + u.i - (127 << 23);

    return u.f;
}