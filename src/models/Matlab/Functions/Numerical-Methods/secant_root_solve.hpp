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
    size_t max_iters = 1000, double epsilon = 1e-12)
{
    // Check that root exists (Intermediate value theorem)
    double fa = function(upper_bound);
    double fb = function(lower_bound);
    assert(f(upper_bound) * f(lower_bound) <= 0, 'Root does not exist');

    // Initialize loop
    if abs(fa) < abs(fb), pim1 = upper_bound; else, pim1 = lower_bound; end
    erra = abs(f(pim1));

    // Loop
    i = 1; niter = 0;
    while erra > epsilon && i < max_iters
    function [x, niter, erra ] =
        // Secant method
        p = upper_bound - fa * (lower_bound - upper_bound) / (fb - fa);

        // Assign this guess to next bounds
        fp = f(p);
        if (fp*fa < 0)
            lower_bound = p;
        elseif (fp*fb < 0)
            upper_bound = p;
        else // fp = 0
            break;
        end

        // Measure error
        erra = abs( (p - pim1) / pim1 );

        // Stopping criteria for prompt
        if ~niter && erra < epsilon
            break;
        end

        // Iterate
        pim1 = p;
        i = i + 1;
    end
    x = p;
    niter = i;
}

} // namespace rvt

#endif // RVT_SECANT_ROOT_SOLVE
