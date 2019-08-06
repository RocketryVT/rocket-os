// secant_root_solve.hpp

#ifndef RVT_SECANT_ROOT_SOLVE
#define RVT_SECANT_ROOT_SOLVE

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
    size_t max_iters = 1000, double epsilon = 1e-12);

} // namespace rvt

#endif // RVT_SECANT_ROOT_SOLVE
