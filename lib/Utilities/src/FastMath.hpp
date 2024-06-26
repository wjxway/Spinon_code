/**
 * @file FastMath.hpp
 * @brief Fast math functions
 */
#ifndef FASTMATH_HPP__
#define FASTMATH_HPP__

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

        template <class T>
        constexpr T square(const T x) noexcept
        {
            return x * x;
        }

        template <class T>
        constexpr T norm(const T x, const T y) noexcept
        {
            return sqrt(square(x) + square(y));
        }

        template <class T>
        constexpr T norm(const T x, const T y, const T z) noexcept
        {
            return sqrt(square(x) + square(y) + square(z));
        }

        template <class T>
        constexpr T clip(const T x, const T low_bound, const T high_bound) noexcept
        {
            return (x < low_bound) ? low_bound : ((x > high_bound) ? high_bound : x);
        }
    } // namespace fast
} // namespace math

#endif