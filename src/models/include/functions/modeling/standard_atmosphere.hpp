// standard_atmosphere.hpp

#ifndef RVT_STANDARD_ATMO_HPP
#define RVT_STANDARD_ATMO_HPP

#include <tuple>
#include <vector>
#include <cmath>

namespace rvt
{

/*
 * Computes the Temperature, Pressure, and Density of the Atmosphere
 * Uses the Geometric Alitude to compute the Temperature, Pressure, and
 * Density of the atmosphere according to the Standard Atmosphere Model
 * presented in John D. Anderson Jr. "Introduction to Flight", c 2016.
 *
 * This atmospheric model uses a gravity model that assumes a spherical,
 * non-rotating Earth. It uses the solutions to the following differential
 * equation from Anderson as the Standard Atmosphere Model:
 *
 *     dp/p = - g / ( R * T(h) ) * dh
 *
 * to solve for Pressure at a specific altitude. In this version of the
 * Standard Atmospheric calculations, the Geopotential Altitude is used to
 * simplify the integration.
 *
 * @arg
 * h   - double
 *       Altitude
 *
 * @return
 * P   - double
 *       Pressure at altitude
 * rho - double
 *       Density at altitude
 * T   - double
 *       Temperature at altitude
 *
 * @author: Matt Marti
 * @date: 2019-04-25
 */
std::tuple<double, double, double> standard_atmosphere(double altitude);

double to_geopotential(double geometric, double radius);

double to_geometric(double geopotential, double radius);

double pressure_altitude(double pressure);

} // namespace rvt

#endif // RVT_STANDARD_ATMO_HPP
