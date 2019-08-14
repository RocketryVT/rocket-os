// secant_root_solve.cpp

#include <cmath>
#include <tuple>
#include <functional>

namespace rvt
{

/*
 * Solves for the roots of a function using the secant method
 * @arg
 * f       - Anonymous Function
 *           Function handle to solve root for
 * a       - double
 *           Upper bound
 * b       - double
 *           Lower bound
 * max_iters - int (optional)
 *           Maximum number of iterations
 * epsilon - double (optional)
 *           Minimum error stopping criteria (in difference)
 * @return
 * x       - double
 *           Function root
 * niter   - int
 *           Nnumber of iterations
 * erra    - double
 *           Root error
 * @author: Matt Marti
 * @date: 2019-04-21
 */

std::tuple<double, size_t, double>
secant_root_solve(std::function<double(double)> function,
    double lower_bound, double upper_bound,
    size_t max_iters, double epsilon)
{
    // Check that root exists (Intermediate value theorem)
    double f_upper = function(upper_bound);
    double f_lower = function(lower_bound);
    // assert(f(upper_bound) * f(lower_bound) <= 0, 'Root does not exist');

    // Initialize loop
    double x_est = lower_bound;
    if (std::abs(f_upper) < std::abs(f_lower)) x_est = upper_bound;

    double error = std::abs(function(x_est));
    size_t iters = 0;
    for (; error > epsilon && iters < max_iters; ++iters)
    {
        double xi = upper_bound - f_upper * (lower_bound - upper_bound)
            / (f_lower - f_upper);

        // Assign this guess to next bounds
        double y = function(xi);
        if (y*f_upper < 0) lower_bound = xi;
        else if (y*f_lower < 0) upper_bound = xi;

        // Measure error
        error = std::abs((xi - x_est) / x_est);

        // Iterate
        x_est = xi;
    }

    return std::make_tuple(x_est, iters, error);
}

} // namespace rvt
