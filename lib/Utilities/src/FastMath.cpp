#include "FastMath.hpp"

float math::fast::cos(const float x) noexcept
{
    float x1=x;

    constexpr float tp = 1.0F / (2.0F * M_PI);

    x1 *= tp;
    x1 -= 0.25F + (float)((int)(x1 + 0.25F));
    x1 *= 16.0F * (abs(x1) - 0.5F);
#if EXTRA_PRECISION
    x1 += 0.225F * x1 * (abs(x1) - 1.0F);
#endif
    return x1;
}

float math::fast::exp(const float x) noexcept
{
  constexpr float DIVLN2 = 1.4426950408889634074F;

  union {
    float f;
    uint32_t i;
    } u = {0}, v = {0};

    float frac = x * DIVLN2;
    int32_t fl = floor_int(frac);
    frac -= fl;

    // exponent
    u.i = (fl + 127) << 23;

    // value
    v.f = (0.6655544926090462F + 0.33284502591696163F * frac) * frac + 0.9984F;

    // sum up
    u.i = v.i + u.i - (127 << 23);

    return u.f;
}