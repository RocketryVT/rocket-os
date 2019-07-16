#ifndef LAMBDA_ECHELON_HPP
#define LAMBDA_ECHELON_HPP

#include <sstream>
#include <algorithm>

#include <matrix.hpp>

/*! 
    \file
    \brief Implements RREF matrix reduction, as well as
           some convenient matrix manipulation operations.
*/

namespace lambda
{

/// \brief Swap two rows in a matrix.
template <size_t M, size_t N>
matrix<M, N> swap_rows(const matrix<M, N> &m, size_t r1, size_t r2)
{
    if (r1 >= M)
    {
        std::stringstream ss;
        ss << "Row " << r1 << " is out of bounds of "
            << M << "x" << N << " matrix";
        throw std::out_of_range(ss.str());
    }
    if (r2 >= M)
    {
        std::stringstream ss;
        ss << "Row " << r2 << " is out of bounds of "
            << M << "x" << N << " matrix";
        throw std::out_of_range(ss.str());
    }

    auto ret = m;
    for (size_t i = 0; i < N; ++i)
    {
        std::swap(ret(r1, i), ret(r2, i));
    }
    return ret;
}

/// \brief Swap two columns in a matrix.
template <size_t M, size_t N>
matrix<M, N> swap_cols(const matrix<M, N> &m, size_t c1, size_t c2)
{
    if (c1 >= M)
    {
        std::stringstream ss;
        ss << "Column " << c1 << " is out of bounds of "
            << M << "x" << N << " matrix";
        throw std::out_of_range(ss.str());
    }
    if (c2 >= M)
    {
        std::stringstream ss;
        ss << "Column " << c2 << " is out of bounds of "
            << M << "x" << N << " matrix";
        throw std::out_of_range(ss.str());
    }

    auto ret = m;
    for (size_t i = 0; i < M; ++i)
    {
        std::swap(ret(i, c1), ret(i, c2));
    }
    return ret;
}

/// \brief Get the RREF form of a matrix.
template <size_t M, size_t N>
matrix<M, N> rref(const matrix<M, N> &m)
{
    auto ret = m;
    size_t lead = 0;
    for (size_t r = 0; r < M; ++r)
    {
        size_t i = r;
        while (ret(i, lead) == 0)
        {
            ++i;
            if (M == i)
            {
                i = r;
                ++lead;
                if (N == lead)
                {
                    return ret;
                }
            }
        }
        swap_rows(ret, i, r);
        if (ret(r, lead) != 0)
        {
            auto div = ret(r, lead);
            for (size_t k = 0; k < N; ++k)
            {
                ret(r, k) /= div;
            }
        }
        for (size_t p = 0; p < M; ++p)
        {
            if (p != r)
            {
                auto mult = ret(p, lead);
                for (size_t k = 0; k < N; ++k)
                {
                    ret(p, k) -= mult*ret(r, k);
                }
            }
        }
        ++lead;
    }
    return ret;
}

/// \brief Get the RREF of a 1x1 matrix.
matrix<1, 1> rref(const matrix<1, 1> &m);

/// \brief Compute the inverse of a square matrix.
template <size_t N>
matrix<N, N> inverse(const matrix<N, N> &mat)
{
    if (!is_invertible(mat))
    {
        std::stringstream ss;
        ss << "Cannot invert matrix " << mat
            << ", which is singular";
        throw std::domain_error(ss.str());
    }
    auto reduced = rref(augment(mat, identity<N, N>()));
    matrix<N, N> inv;
    for (size_t r = 0; r < N; ++r)
    {
        for (size_t c = 0; c < N; ++c)
        {
            inv(r, c) = reduced(r, N + c);
        }
    }
    return inv;
}

} // namespace lambda

#endif // LAMBDA_ECHELON_HPP
