
#include <solve.hpp>

#include <limits>
#include <cmath>

namespace lambda
{

double newton_raphson(const std::function<double(double)> &f,
    double x0, size_t maxiter, double epsilon)
{
    for (size_t i = 0; i < maxiter && std::abs(f(x0)) > epsilon; i++)
    {
        double dfdx = ddx(f, x0);
        if (dfdx == 0) x0 += std::numeric_limits<double>::epsilon()*10;
        else x0 = x0 - f(x0)/ddx(f, x0);
    }
    return x0;
}

double ddx(const std::function<double(double)> &f,
           double x, double epsilon)
{
    double x0 = x - epsilon, x1 = x + epsilon;
    double y0 = f(x0), y1 = f(x1);
    return (y1 - y0)/(x1 - x0);
}

}
