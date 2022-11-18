#ifndef _FASTMATH_HPP_
#define _FASTMATH_HPP_

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdint>

#define EXTRA_PRECISION 1

namespace math {
namespace fast {

//Accuracy @ 6%
//sufficient for FFT usage
constexpr float cos(float x);

//T could be float or double
template<class T>
constexpr int32_t floor_int(T x) noexcept
{
    int32_t val=(int32_t)x;
    if(val==x) return x;
    else if(x>0) return val;
    else return val-1;
}

//T could be float or double
template<class T>
constexpr T floor(T x) noexcept
{
    return T(Fast_floor_int(x));
}

//accuracy @ 0.4%
constexpr float exp(float x) noexcept;

} // namespace fast
} // namespace math

#endif
