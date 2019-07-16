// constants.hpp

#ifndef RVT_MODELS_CONSTANTS_HPP
#define RVT_MODELS_CONSTANTS_HPP

/*
 * Defines global constants for various calculations
 *
 * @author: Matt Marti
 * @date: 2019-04-26
 */

namespace rvt
{

// earth and gravity parametersi

// [m^3/kg/s^2] universal gravitational constant
const double univ_grav = 6.674e-11;
const double mass_earth = 5.972e24; // [kg] Earth mass
// [m^3/s^2] Earth Gravity constant
const double mu_earth = univ_grav*mu_earth;
const double omega_earth = 7.292115e-5; // [rad/s]

// Atmospheric Parameters
const double R = 287.0429126675655880; // Standard Atmosphere Gas Constant

// Latitude and Longitude calculations
const double a_earth = 6378137.00000; // meters
const double b_earth = 6356752.31425; // meters
const double ECEF2LATLON_MAXITER = 100;
const double ECEF2LATLON_PRECISION = eps;

} // namespace rvt

#endif // RVT_MODELS_CONSTANTS_HPP
