// standard_atmosphere.cpp

#include <standard_atmosphere.hpp>
#include <limits>
#include <constants.hpp>

namespace rvt
{

std::tuple<double, double, double>
    standard_atmosphere(double altitude)
{
    // shorthands for constants used
    const double radius = rvt::earth_mean_radius;
    const double g0 = rvt::earth_standard_gravity;
    const double R = rvt::air_ideal_gas_constant;

    const double hgp = to_geopotential(altitude, radius);

    // initial conditions
    double h0 = 0;
    double T0 = 288.16;
    double p0 = 1.01325e5;

    // standard atmosphere temperature gradient and associated altitudes
    const std::vector<double>
        temp_gradients { -6.5e-3, 0, 3e-3, 0, -4.5e-3, 0, 4e-3, 0 },
        altitudes { -radius, 11e3, 25e3, 47e3, 53e3, 79e3, 90e3, 105e3,
                     std::numeric_limits<double>::infinity() };

    // Iterate to given altitude
    for (size_t i = 0; i < altitudes.size() - 1 && hgp > altitudes[i]; ++i)
    {
        const double hi = std::min(hgp, altitudes[i+1]);
        const double ai = temp_gradients[i];
        const double dh = hi - h0;
        const double Ti = T0 + ai*dh;
        const double TiT0 = Ti / T0;
        double pi = p0 * std::pow(TiT0, -g0/(ai*R));
        if (ai == 0) pi = p0 * std::exp(-g0/(R*Ti)*dh);

        h0 = hi;
        p0 = pi;
        T0 = Ti;
    }

    const double density = p0 / (R * T0);
    return std::make_tuple(T0, p0, density);
}

double to_geopotential(double geometric, double radius)
{
    return geometric * radius / (radius + geometric);
}

double to_geometric(double geopotential, double radius)
{
    return geopotential * radius / (radius - geopotential);
}

} // namespace rvt
