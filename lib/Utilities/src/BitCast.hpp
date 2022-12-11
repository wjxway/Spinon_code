/**
 * @file BitCast.hpp
 * @brief bit cast, but a really unsafe version. I don't know why some macros
 * cannot work here, so it's up to the user to make sure that it can work.
 */
#ifndef BITCAST_HPP__
#define BITCAST_HPP__

#if __cplusplus < 202002L
namespace std
{
    /**
     * @brief bit cast from one type to another.
     *
     * @tparam To to which type
     * @tparam From from which type
     * @param src input data with input type
     * @return To output data with output type
     *
     * @note bit cast, but a really unsafe version. I don't know why some macros
     * cannot work here, so it's up to the user to make sure that it can work.
     *
     * @note this utility is included in C++20.
     */
    template <class To, class From>
    To bit_cast(const From &src) noexcept
    {
        To dst;
        memcpy(&dst, &src, sizeof(To));
        return dst;
    }
} // namespace std
#endif

#endif