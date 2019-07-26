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
const double universal_gravity = 6.674e-11;
const double mass_earth = 5.972e24; // [kg] Earth mass
// [m^3/s^2] Earth Gravity constant
const double mu_earth = universal_gravity*mu_earth;
const double omega_earth = 7.292115e-5; // [rad/s]
const double earth_standard_gravity = 9.80665; // [m/s^2]
const double earth_mean_radius = 63781e6; // [meters]

// Atmospheric Parameters
const double molar_gas_constant = 8.31446261815324; // [J/mol-K]
const double air_ideal_gas_constant = 287.0429126675655880; // [J/kg-K]
const double standard_pressure = 101325.0; // Pa
const double standard_temperature = 273.15; // Kelvin

// Latitude and Longitude calculations
const double a_earth = 6378137.00000; // meters
const double b_earth = 6356752.31425; // meters
const double ECEF2LATLON_MAXITER = 100;
// const double ECEF2LATLON_PRECISION = eps;

} // namespace rvt

#endif // RVT_MODELS_CONSTANTS_HPP
