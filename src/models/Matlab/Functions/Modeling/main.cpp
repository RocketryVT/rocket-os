
#include <standard_atmosphere.hpp>
#include <iostream>

int main()
{
    std::cout << "altitude "
              << "pressure "
              << "density "
              << "temperature" << std::endl;
    for (double altitude = 0; altitude < 100000 /* m */; altitude += 10)
    {
        double pressure, density, temperature;
        std::tie(pressure, density, temperature) = rvt::standard_atmosphere(altitude);
        std::cout << altitude << " "
                  << pressure << " "
                  << density << " "
                  << temperature << std::endl;
    }
}
