/**
 * @file FastMath.hpp
 * @brief Fast math functions
 */
#ifndef _FASTMATH_HPP_
#define _FASTMATH_HPP_

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdint>

#define EXTRA_PRECISION 1

namespace math
{
    namespace fast
    {
        // Accuracy @ 6%
        // sufficient for FFT usage
        float cos(const float x) noexcept;

        // T could be float or double
        template <class T>
        int32_t floor_int(const T x) noexcept
        {
            const int32_t val = (int32_t)x;
            if (val == x)
            {
                return x;
            }
            else if (x > 0)
            {
                return val;
            }
            else
            {
                return val - 1;
            }
        }

        // T could be float or double
        template <class T>
        T floor(const T x) noexcept
        {
            return T(floor_int(x));
        }

        // accuracy @ 0.4%
        float exp(const float x) noexcept;
    }
}

#endif