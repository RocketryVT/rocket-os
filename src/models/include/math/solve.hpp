#ifndef LAMBDA_SOLVE_HPP
#define LAMBDA_SOLVE_HPP

#include <functional>
#include <limits>
#include <vector>

/*!
    \file
    \brief Defines numerical methods for solving and for zeroes,
           computing derivatives, etc.
*/

namespace lambda
{

template <typename T>
std::vector<double> range(T begin, T end, T step = 1)
{
    std::vector<double> vec;
    for (T iter = begin; iter < end; iter += step)
    {
        vec.push_back(iter);
    }
    return vec;
}

/// \brief Compute the zero of a function
///        using the Newton-Raphson method.
double newton_raphson(const std::function<double(double)> &f,
    double x0, size_t maxiter = 1000,
    double epsilon = std::numeric_limits<double>::epsilon());

/// \brief Numerically compute the derivative of a function.
double ddx(const std::function<double(double)> &f,
    double x, double epsilon = 0.01);

} // namespace lambda

#endif // LAMBDA_SOLVE_HPP
