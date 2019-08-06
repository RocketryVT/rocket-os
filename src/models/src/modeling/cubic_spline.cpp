// cubic_spline.cpp

#include <cubic_spline.hpp>

namespace rvt
{

/*
 * Cubic spline interpolation function
 * Interpolates function values at specified points using a cubic spline
 *
 * @arg
 * xkvec   - n x 1 double matrix
 *           Independent variable data points
 * fkvec   - n x 1 double matrix
 *           Dependent variable data points
 * xinter  - n x 1 double matrix
 *           Interpolation points
 * fslope  - 2 x 1 double matrix (optional)
 *           Function slope at boundary points
 *
 * @return
 * finter  - n x 1 double matrix
 *           Interpolated function value
 * dfinter - n x 1 double matrix
 *           Interpolated function derivative value
 * akvec   - n x 1 double matrix
 *           Spline coefficient vector: a
 * bkvec   - n x 1 double matrix
 *           Spline coefficient vector: b
 * ckvec   - n x 1 double matrix
 *           Spline coefficient vector: c
 * dkvec   - n x 1 double matrix
 *           Spline coefficient vector: d
 * xstar   - n x 1 double matrix
 *           Intermediate solution in tri-diagonal equations
 *
 * @author: Matt Marti
 * @date: 2019-05-06
 */
std::tuple<std::vector<double>, // function interpolated value
           std::vector<double>, // function derivatives
           std::vector<double>, // spline a cofficients
           std::vector<double>, // spline b cofficients
           std::vector<double>, // spline c cofficients
           std::vector<double>, // spline d cofficients
           std::vector<double>> // intmdt sol in tri-diagonal equations
cubic_spline(const std::vector<double> &xkvec,
              const std::vector<double> &fkvec,
              const std::vector<double> &xinter)
{
    std::vector<double> finter, dfinter,
        akvec, bkvec, ckvec, dkvec, xstar;

    return std::make_tuple(finter, dfinter,
        akvec, bkvec, ckvec, dkvec, xstar);
}

} // namespace rvt
