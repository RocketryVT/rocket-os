#ifndef LAMBDA_DECOMPOSITION_HPP
#define LAMBDA_DECOMPOSITION_HPP

#include <tuple>
#include <matrix.hpp>

/*! 
    \file
    \brief Defines LU and LDU decomposition of matrices.
*/

namespace lambda
{

/// \brief Computes the LU decomposition of a square matrix.
template <size_t N>
std::tuple<matrix<N, N>,
           matrix<N, N>>
decompose_lu(const matrix<N, N> &m)
{
    // TODO

    matrix<N, N> L, U;
    return std::make_tuple(L, U);
}

/// \brief Computes the LDU decomposition of a square matrix.
template <size_t M, size_t N>
std::tuple<matrix<M, M>,
           matrix<M, M>,
           matrix<M, N>>
decompose_ldu(const matrix<N, N> &m)
{
    // TODO

    matrix<M, M> L, D;
    matrix<M, N> U;
    return std::make_tuple(L, D, U);
}

} // namespace lambda

#endif // LAMBDA_DECOMPOSITION_HPP

