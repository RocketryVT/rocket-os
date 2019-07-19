// hybrid_thrust.hpp

#ifndef RVT_HYBRID_THRUST_HPP
#define RVT_HYBRID_THRUST_HPP

#include <tuple>
#include <vector>
#include <functional>

namespace rvt
{

/*
 * Computes the thrust of a hybrid rocket motor
 * Computes the thrust of a hybrid rocket motor given the ambient
 * pressure, oxidizer flow rate, physical design of the nozzle, etc.
 *
 * Areas and anonymous functions for Oxidizer Mass Flux Rates are passed as
 * arguements to this function in order to keep it flexible to account for
 * any fuel grain geometry.
 *
 * arg
 *                - double
 *                   Gravity magnitude
 * fuel_density            - double
 *                   Fuel mass density
 * regress_fun     - Anonymous Function
 *                   Regression Rate as a function of Oxidizer Mass Velocity
 * rsplinevec      - Nr x 1 double vector
 *                   Fuel/Oxidizer Ratio vector for spline interpolation of
 *                   cstar and specific heat ratio.
 * cstarsplinedata - Nr x 1 double vector
 *                   cstar data vector as a function of Fuel/Oxidizer ratio
 *                   for spline interpolation.
 * ksplinedata     - Nr x 1 double vector
 *                   Specific heat ratio data vector as a function of
 *                   Fuel/Oxidizer ratio
 *                   for spline interpolation.
 * cstar_efficiency             - double
 *                   cstar efficiency value, scale 0.0 to 1.0
 * At              - double
 *                   Nozzle throat area
 * A2              - double
 *                   Nozzle exit area
 * Ap              - double
 *                   Total combustion port area
 * Ab              - double
 *                   Exposed fuel surface area
 * modot           - double
 *                   Oxidizer mass flow rate
 * p3              - double
 *                   Ambient Pressure
 *
 * @return
 * F               - double
 *                   Thrust
 * rdot            - double
 *                   Fuel Regression Rate
 * fuel_mass_flow_rate           - double
 *                   Fuel mass rate of change
 * mdot            - double
 *                   Mass Flow Rate
 * p1              - double
 *                   Combustion Chamber Pressure
 * specific_impulse             - double
 *                   Specific Impulse
 * CF              - double
 *                   Thrust coefficient
 * c               - double
 *                   Exhaust Velocity
 *
 * @author: Matt Marti
 * @date: 2019-04-25
 */

std::tuple<double, double, double, double, double, double, double, double>
hybrid_thrust(double g, double fuel_density,
    const std::function<double(double)> &regression,
    const std::vector<double> &mixing_ratio_spline,
    const std::vector<double> &cstar_spline,
    const std::vector<double> &ksplinedata,
    double cstar_efficiency, double throat_area, double A2, double Ap,
    double Ab, double modot, double p3);

} // namespace rvt

#endif // RVT_HYBRID_THRUST_HPP
