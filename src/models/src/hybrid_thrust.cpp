
#include <hybrid_thrust.hpp>
#include <secant_root_solve.hpp>
#include <cubic_spline.hpp>

#include <cmath>

namespace rvt
{

std::tuple<double, double, double, double, double, double, double, double>
hybrid_thrust(double g, double fuel_density,
    const std::function<double(double)> &regression,
    const std::vector<double> &mixing_ratio_spline,
    const std::vector<double> &cstar_spline,
    const std::vector<double> &ksplinedata,
    double cstar_efficiency, double throat_area, double A2, double Ap,
    double Ab, double modot, double p3)
{
    // If there is no oxidizer mass flow rate, thrust is 0
    if (modot == 0) return std::make_tuple(0, 0, 0, 0, p3, 0, 0, 0);

    double ox_mass_velocity = modot / Ap;
    double rdot = regression(ox_mass_velocity);
    double fuel_mass_flow_rate = fuel_density * Ab * rdot; // Eq 16-11
    double mixing_ratio = modot / fuel_mass_flow_rate;

    // Determine characteristic velocity and specific heats using cubic spline
    auto spline = rvt::cubic_spline(
        mixing_ratio_spline, cstar_spline, {mixing_ratio}); // [ft/s]
    double cstar_theory = std::get<0>(spline)[0];
    double cstar_real = cstar_efficiency * cstar_theory;
    spline = rvt::cubic_spline(
        mixing_ratio_spline, ksplinedata, {mixing_ratio}); // [ft/s]
    double k = std::get<0>(spline)[0];

    // Determine combustion chamber pressure
    double mdot = fuel_mass_flow_rate + modot;
    double p1 = mdot * cstar_real / throat_area; // Sutton 16-8

    auto area_ratio_function = [k] (double m)
    {
        return 1/m*std::pow(2*(1+0.5*(k-1) *
                   std::pow(m, 2))/(k+1), 0.5*(k+1)/(k-1)); // Sutton 3-14
    };

    double area_ratio = A2 / throat_area; // Sutton 3-19
    auto mach_error_function = [area_ratio, area_ratio_function] (double m)
    {
        return std::log(area_ratio_function(m)) - std::log(area_ratio);
    };

    // Determine Exit Mach number
    double exit_mach_number, error;
    size_t iters;
    std::tie(exit_mach_number, iters, error) = rvt::secant_root_solve(
        mach_error_function, 1, 5);

    // Compute Stagnation Pressure ratio (Stagnation Pressure P0 ~= p1)
    double p1_p2 = std::pow((1+0.5*(k-1)*std::pow(exit_mach_number, 2)), k/(k-1)); // Sutton 3-13
    double p2 = p1 / p1_p2;
    double p2_p1 = 1/p1_p2;
    double thrust_coefficient =
        sqrt(((2*std::pow(k, 2))/(k-1)) *
              std::pow(2/(k+1), (k+1)/(k-1)) *
              (1-std::pow(p2_p1, ((k-1)/k))) +
              (p2-p3)/p1*area_ratio);

    // Compute thrust
    double thrust = throat_area * thrust_coefficient * p1;
    double specific_impulse = thrust_coefficient * cstar_real / g;

    // Exhaust velocity
    double exhaust_velocity = thrust / mdot;

    return std::make_tuple(thrust, rdot, fuel_mass_flow_rate, mdot, p1,
             specific_impulse, thrust_coefficient, exhaust_velocity);
}

} // namespace rvt
