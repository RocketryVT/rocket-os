
#include <standard_atmosphere.hpp>
#include <iostream>

int main()
{
    std::cout << "altitude "
              << "temperature "
              << "pressure "
              << "density" << std::endl;
    for (double altitude = 0; altitude < 100000 /* m */; altitude += 10)
    {
        double temperature, pressure, density;
        std::tie(temperature, pressure, density) = rvt::standard_atmosphere(altitude);
        std::cout << altitude << " "
                  << temperature << " "
                  << pressure << " "
                  << density << std::endl;
    }
}
